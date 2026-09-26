#!/usr/bin/env python3
"""
Extract the wire layout of msp.c's config opcodes, for the manifest.

The configurator is moving off the hand-written MSP config catalogue before
step 5 deletes it (parameter-addressing-design.md). Rather than rewrite every
tab, it answers the legacy opcodes itself from PARAM_READ / PARAM_WRITE -- and
to do that it needs, per opcode, which parameter-group bytes each wire field
is. That is exactly what the serializers in msp.c say, so this reads them
instead of transcribing them by hand.

Input is msp.c *preprocessed for the target*: every #if resolved, every
opcode macro a number. Each case body is parsed with pycparser and run
through a small symbolic executor that understands the shapes the catalogue
is written in:

    sbufWriteU16(dst, gyroConfig()->gyro_lpf1_static_hz);         out field
    for (int i = 0; i < 3; i++) sbufWriteU8(dst, x()->a[i]);      unrolled
    armingConfigMutable()->auto_disarm_delay = sbufReadU8(src);    in field
    if (sbufBytesRemaining(src) >= 1) { ... }                      optional tail
    i = sbufReadU8(src); if (i < 42) { r = rangesMutable(i); ... } indexed
    currentPidProfile->pid_mode                                    selected profile

Anything else makes the opcode *manual*: no codec is emitted, and the reason
is recorded. A wrong codec would write the wrong bytes, so the executor
refuses rather than guesses; the configurator falls back to the real opcode.

Calls that touch neither buffer (validateAndFixGyroConfig(), ...) are side
effects. They are recorded, not replayed: MSP_EEPROM_WRITE runs
validateAndFixConfig() and activateConfig(), and every tab saves after
writing, so the save re-applies them.
"""

import re

try:
    from pycparser import c_parser, c_ast
except ImportError:  # the manifest still builds; it just carries no codecs
    c_parser = c_ast = None

# Dispatch functions whose case bodies are serializers, and their direction.
FUNCTIONS = {
    'mspCommonProcessOutCommand': 'out',
    'mspProcessOutCommand': 'out',
    'mspFcProcessOutCommandWithArg': 'out',
    'mspProcessInCommand': 'in',
    'mspCommonProcessInCommand': 'in',
}

# Pointers msp.c uses for "the profile currently selected".
PROFILE_POINTERS = {
    'currentPidProfile': ('pidProfiles', 'pid'),
    'currentControlRateProfile': ('controlRateProfiles', 'rate'),
    'currentTvPidProfile': ('tvPidProfiles', 'tv'),
}


class Manual(Exception):
    """The body does something the executor does not model."""


# --- source slicing ---------------------------------------------------------

def function_body(src, name):
    m = re.search(r'\b' + name + r'\s*\([^;{]*\)\s*\{', src)
    if not m:
        return None
    depth, i = 1, m.end()
    while depth and i < len(src):
        depth += {'{': 1, '}': -1}.get(src[i], 0)
        i += 1
    return src[m.end():i - 1]


def case_bodies(body):
    """Top-level cases of the function's switch: {opcode: text}.

    Only labels at the switch's own indentation count, so a nested switch
    inside a case cannot split it. Fall-through labels share the next body.
    """
    labels = list(re.finditer(r'\n    case\s+(\d+|0x[0-9a-fA-F]+)\s*:', body))
    ends = [m.start() for m in labels[1:]]
    default = re.search(r'\n    default\s*:', body)
    ends.append(default.start() if default else len(body))
    out, pending = {}, []
    for m, end in zip(labels, ends):
        text = body[m.end():end]
        pending.append(int(m.group(1), 0))
        if text.strip():
            for op in pending:
                out.setdefault(op, text)
            pending = []
    return out


# --- manifest field resolution ------------------------------------------------

class Groups(object):
    """Parameter groups by accessor name, with byte-offset resolution."""

    def __init__(self, pgs):
        self.by_symbol = {pg['symbol']: pg for pg in pgs if pg.get('symbol')}

    def group(self, accessor):
        base = accessor[:-len('Mutable')] if accessor.endswith('Mutable') else accessor
        for suffix, is_array in (('_System', False), ('_SystemArray', True)):
            pg = self.by_symbol.get(base + suffix)
            if pg:
                return pg, is_array
        raise Manual('no parameter group behind %s()' % accessor)

    @staticmethod
    def resolve(fields, path):
        """(offset, size, signed, count) of a path like 'pid[2].P' or 'mountTrim.roll'.

        Manifest field names are dotted paths of their own ('range.startStep'),
        so segments are matched greedily against the longest name that fits.
        """
        segs = path
        offset = 0
        while True:
            best = None
            for f in fields:
                parts = f['name'].split('.') if f['name'] else []
                names = [s[0] for s in segs[:len(parts)]]
                if parts and names == parts and all(s[1] is None for s in segs[:len(parts) - 1]):
                    if best is None or len(parts) > len(best[1]):
                        best = (f, parts)
            if best is None:
                # an anonymous repeat (an array group's element) has name ''
                anon = [f for f in fields if f['name'] == '' and f['kind'] == 'repeat']
                if anon:
                    fields = anon[0]['fields']
                    continue
                raise Manual('field %s not in the manifest' % '.'.join(s[0] for s in segs))
            f, parts = best
            index = segs[len(parts) - 1][1]
            rest = segs[len(parts):]
            if f['kind'] == 'repeat':
                offset += f['off'] + (index or 0) * f['stride']
                if not rest:
                    raise Manual('whole struct %s used as a value' % f['name'])
                fields, segs = f['fields'], rest
                continue
            if f['kind'] == 'array':
                if rest:
                    raise Manual('member of array element %s' % f['name'])
                if index is None:
                    return offset + f['off'], f['elem_size'], f['elem_kind'] == 'int', f['count']
                if index >= f['count']:
                    raise Manual('%s[%d] is past the array' % (f['name'], index))
                return offset + f['off'] + index * f['elem_size'], f['elem_size'], f['elem_kind'] == 'int', None
            if rest or index is not None:
                raise Manual('scalar %s indexed or dereferenced' % f['name'])
            return offset + f['off'], f['size'], f['kind'] == 'int', None


# --- symbolic executor ---------------------------------------------------------

class Ref(object):
    """An lvalue in a parameter group: group, element, path."""

    def __init__(self, pg, element=None, relative=None, path=()):
        self.pg, self.element, self.relative, self.path = pg, element, relative, tuple(path)

    def member(self, name, index=None):
        return Ref(self.pg, self.element, self.relative, self.path + ((name, index),))

    def index_last(self, index):
        name, old = self.path[-1]
        if old is not None:
            raise Manual('two-dimensional index')
        return Ref(self.pg, self.element, self.relative, self.path[:-1] + ((name, index),))


INDEX = 'INDEX'  # the value of a leading index byte, in an indexed setter


class Executor(object):
    def __init__(self, groups, direction, constants=None):
        self.groups = groups
        self.direction = direction
        self.constants = constants or {}
        self.checks = {}        # {'len': k} / {'min_len': k} from dataSize guards
        self.ops = []
        self.env = {}
        self.index = None       # {'w': 1, 'max': N} for indexed setters
        self.optional = False   # inside if (sbufBytesRemaining(src) >= k)
        self.side_effects = []
        self.done = False

    # expressions -----------------------------------------------------------

    def value(self, node):
        """An int, a Ref, INDEX, or ('read', width, signed)."""
        if isinstance(node, c_ast.Constant):
            if node.type in ('int', 'unsigned int', 'long', 'unsigned long'):
                return int(node.value.rstrip('uUlL'), 0)
            raise Manual('constant %s' % node.value)
        if isinstance(node, c_ast.Cast):
            return self.value(node.expr)
        if isinstance(node, c_ast.UnaryOp):
            if node.op == '-':
                v = self.value(node.expr)
                if isinstance(v, int):
                    return -v
            if node.op == '&':
                return self.value(node.expr)
            if node.op == 'sizeof':
                raise Manual('sizeof')
            raise Manual('unary %s' % node.op)
        if isinstance(node, c_ast.BinaryOp):
            # Only constant folding: `1 + 4 * 16` in a size, `i + 1` in a loop.
            a, b = self.value(node.left), self.value(node.right)
            fold = {'+': lambda: a + b, '-': lambda: a - b, '*': lambda: a * b,
                    '<<': lambda: a << b, '|': lambda: a | b}
            if isinstance(a, int) and isinstance(b, int) and node.op in fold:
                return fold[node.op]()
            raise Manual('computed value (%s)' % node.op)
        if isinstance(node, c_ast.ID):
            if node.name in self.env:
                return self.env[node.name]
            if node.name in self.constants:
                return self.constants[node.name]
            if node.name in PROFILE_POINTERS:
                accessor, sel = PROFILE_POINTERS[node.name]
                pg, _ = self.groups.group(accessor)
                return Ref(pg, relative=sel)
            raise Manual('identifier %s' % node.name)
        if isinstance(node, c_ast.FuncCall):
            return self.call(node, want_value=True)
        if isinstance(node, c_ast.StructRef):
            base = self.value(node.name)
            if not isinstance(base, Ref):
                raise Manual('member of a non-group value')
            return base.member(node.field.name)
        if isinstance(node, c_ast.ArrayRef):
            base = self.value(node.name)
            sub = self.value(node.subscript)
            if not isinstance(sub, int):
                raise Manual('non-constant array index')
            if isinstance(base, Ref) and base.path:
                return base.index_last(sub)
            raise Manual('indexing a non-array')
        raise Manual('expression %s' % type(node).__name__)

    def call(self, node, want_value=False):
        name = node.name.name if isinstance(node.name, c_ast.ID) else None
        args = node.args.exprs if node.args else []
        if name is None:
            raise Manual('call through a pointer')

        m = re.fullmatch(r'sbufRead([US])(8|16|32)', name)
        if m:
            return ('read', int(m.group(2)) // 8, m.group(1) == 'S')

        m = re.fullmatch(r'sbufWrite([US])(8|16|32)', name)
        if m:
            self.emit_out(int(m.group(2)) // 8, args[1])
            return None
        if name == 'sbufWriteData':
            ref = self.value(args[1])
            length = self.value(args[2])
            if not isinstance(ref, Ref) or not isinstance(length, int):
                raise Manual('sbufWriteData of a computed range')
            off, size, _, count = self.place(ref)
            if count is None or length > size * count:
                raise Manual('sbufWriteData past its array')
            self.ops.append(self.field_op({'kind': 'data', 'len': length}, ref, off))
            return None
        if name in ('sbufReadData', 'sbufWriteString', 'sbufReadString'):
            raise Manual(name)
        if name == 'sbufBytesRemaining':
            raise Manual('sbufBytesRemaining outside an optional-tail check')

        # accessor()->  /  accessor(i)->
        if want_value and (name.endswith('Mutable') or not args or len(args) == 1):
            try:
                pg, is_array = self.groups.group(name)
            except Manual:
                pg = None
            if pg is not None:
                if is_array:
                    if len(args) != 1:
                        raise Manual('array group %s() without an index' % name)
                    idx = self.value(args[0])
                    if idx == INDEX:
                        return Ref(pg, element=INDEX)
                    if not isinstance(idx, int):
                        raise Manual('group element by computed index')
                    return Ref(pg, element=idx)
                if args:
                    raise Manual('%s() takes no index' % name)
                return Ref(pg)

        if want_value:
            raise Manual('value from %s()' % name)
        # A call touching neither buffer: a side effect the save re-applies.
        for a in args:
            if self.mentions_buffer(a):
                raise Manual('%s() consumes the buffer' % name)
        self.side_effects.append(name)
        return None

    @staticmethod
    def mentions_buffer(node):
        class V(c_ast.NodeVisitor):
            hit = False

            def visit_ID(self, n):
                if n.name in ('src', 'dst'):
                    V.hit = True
        V().visit(node)
        return V.hit

    # placement -------------------------------------------------------------

    def place(self, ref):
        segs = list(ref.path)
        if not segs:
            raise Manual('whole group used as a value')
        off, size, signed, count = self.groups.resolve(ref.pg['fields'], segs)
        return off, size, signed, count

    def field_op(self, op, ref, off):
        op = dict(op)
        op['pgn'] = ref.pg['pgn']
        elem = ref.pg['size'] // ref.pg['length']
        if ref.relative:
            op['profile'] = ref.relative
            op['off'] = off
        elif ref.element == INDEX:
            op['indexed'] = True
            op['off'] = off
        else:
            op['off'] = off + (ref.element or 0) * elem
        if self.optional:
            op['optional'] = True
        return op

    def emit_out(self, width, expr):
        v = self.value(expr)
        if isinstance(v, int):
            self.ops.append({'kind': 'const', 'w': width, 'value': v})
            return
        if isinstance(v, Ref):
            off, size, signed, count = self.place(v)
            if count is not None:
                raise Manual('array written as a scalar')
            self.ops.append(self.field_op({'kind': 'field', 'w': width, 'size': size, 'signed': signed}, v, off))
            return
        raise Manual('write of a computed value')

    def assign(self, lhs, rhs):
        v = self.value(rhs)
        if isinstance(lhs, c_ast.ID) and lhs.name not in self.env and isinstance(v, tuple) and v[0] == 'read':
            # i = sbufReadU8(src): the element index of an indexed setter
            if self.ops or self.index is not None:
                raise Manual('index read after data')
            self.index = {'w': v[1], 'max': None}
            self.env[lhs.name] = INDEX
            return
        if isinstance(v, tuple) and v[0] == 'local':
            # field = local: the parked wire value lands here
            ref = self.value(lhs)
            op = self.ops[v[1]]
            if not isinstance(ref, Ref) or op.get('kind') != 'skip':
                raise Manual('wire local stored twice or into a local')
            off, size, signed, count = self.place(ref)
            if count is not None:
                raise Manual('read into a whole array')
            placed = self.field_op({'kind': 'field', 'w': op['w'], 'wire_signed': v[2],
                                    'size': size, 'signed': signed}, ref, off)
            placed.pop('optional', None)
            if op.get('optional'):
                placed['optional'] = True
            if 'check' in op:
                placed['check'] = op['check']
            self.ops[v[1]] = placed
            return
        if isinstance(v, tuple) and v[0] == 'read':
            ref = self.value(lhs)
            if not isinstance(ref, Ref):
                raise Manual('read into a local')
            off, size, signed, count = self.place(ref)
            if count is not None:
                raise Manual('read into a whole array')
            self.ops.append(self.field_op({'kind': 'field', 'w': v[1], 'wire_signed': v[2],
                                           'size': size, 'signed': signed}, ref, off))
            return
        raise Manual('assignment that is not a wire read')

    # statements --------------------------------------------------------------

    def run(self, node):
        if self.done or node is None:
            return
        if isinstance(node, c_ast.Compound):
            for item in node.block_items or []:
                self.run(item)
                if self.done:
                    return
            return
        if isinstance(node, c_ast.Break) or isinstance(node, c_ast.Return):
            if isinstance(node, c_ast.Return) and node.expr is not None and not self._is_ok(node.expr):
                raise Manual('returns an error on a path the codec would take')
            self.done = True
            return
        if isinstance(node, c_ast.FuncCall):
            v = self.call(node)
            if isinstance(v, tuple) and v[0] == 'read':
                self.ops.append({'kind': 'skip', 'w': v[1], **({'optional': True} if self.optional else {})})
            return
        if isinstance(node, c_ast.Assignment):
            if node.op != '=':
                raise Manual('compound assignment')
            self.assign(node.lvalue, node.rvalue)
            return
        if isinstance(node, c_ast.Decl):
            if node.init is None:
                self.env[node.name] = None
                return
            v = self.value(node.init)
            if isinstance(v, tuple) and v[0] == 'read':
                if self.ops or self.index is not None:
                    # A wire value parked in a local, stored into a field
                    # later (after a range check): hold its place.
                    self.ops.append({'kind': 'skip', 'w': v[1], 'local': node.name,
                                     **({'optional': True} if self.optional else {})})
                    self.env[node.name] = ('local', len(self.ops) - 1, v[2])
                    return
                self.index = {'w': v[1], 'max': None}
                self.env[node.name] = INDEX
                return
            self.env[node.name] = v
            return
        if isinstance(node, c_ast.For):
            self.run_for(node)
            return
        if isinstance(node, c_ast.If):
            self.run_if(node)
            return
        if isinstance(node, c_ast.EmptyStatement):
            return
        raise Manual('statement %s' % type(node).__name__)

    @staticmethod
    def _is_ok(expr):
        return isinstance(expr, c_ast.ID) and expr.name == 'MSP_RESULT_ACK'

    @staticmethod
    def is_error_exit(stmt):
        items = stmt.block_items if isinstance(stmt, c_ast.Compound) else [stmt]
        return (len(items or []) == 1 and isinstance(items[0], c_ast.Return)
                and isinstance(items[0].expr, c_ast.ID) and items[0].expr.name == 'MSP_RESULT_ERROR')

    def guard(self, c):
        """`if (<c>) return MSP_RESULT_ERROR;` -- record what it refuses."""
        if not isinstance(c, c_ast.BinaryOp):
            raise Manual('guard shape')
        if c.op == '||':
            self.guard(c.left)
            self.guard(c.right)
            return
        left = c.left
        bound = self.value(c.right)
        if not isinstance(bound, int):
            raise Manual('guard bound not constant')
        if isinstance(left, c_ast.ID) and left.name == 'dataSize':
            if c.op == '!=':
                self.checks['len'] = bound
            elif c.op == '<':
                self.checks['min_len'] = bound
            else:
                raise Manual('dataSize guard %s' % c.op)
            return
        if isinstance(left, c_ast.ID) and self.env.get(left.name) == INDEX and self.index is not None:
            if c.op != '>=':
                raise Manual('index guard %s' % c.op)
            self.index['max'] = bound if self.index['max'] is None else min(bound, self.index['max'])
            return
        v = self.env.get(left.name) if isinstance(left, c_ast.ID) else None
        if isinstance(v, tuple) and v[0] == 'local':
            op = self.ops[v[1]]
            check = op.setdefault('check', {})
            if c.op == '<':
                check['min'] = bound
            elif c.op == '>':
                check['max'] = bound
            elif c.op == '>=':
                check['max'] = bound - 1
            else:
                raise Manual('value guard %s' % c.op)
            return
        raise Manual('guard on a computed value')

    def run_for(self, node):
        init, cond, nxt = node.init, node.cond, node.next
        decls = init.decls if isinstance(init, c_ast.DeclList) else None
        if not decls or len(decls) != 1 or not isinstance(cond, c_ast.BinaryOp) or cond.op != '<':
            raise Manual('loop shape')
        var = decls[0].name
        start = self.value(decls[0].init)
        end = self.value(cond.right)
        if not (isinstance(cond.left, c_ast.ID) and cond.left.name == var):
            raise Manual('loop condition')
        if not (isinstance(nxt, c_ast.UnaryOp) and nxt.op in ('p++', '++') and nxt.expr.name == var):
            raise Manual('loop step')
        if not isinstance(start, int) or not isinstance(end, int) or end - start > 256:
            raise Manual('loop bound not constant')
        for i in range(start, end):
            self.env[var] = i
            self.run(node.stmt)
            if self.done:
                break
        self.env.pop(var, None)

    def run_if(self, node):
        c = node.cond
        # if (sbufBytesRemaining(src) >= k) { optional tail }
        if (isinstance(c, c_ast.BinaryOp) and c.op == '>=' and isinstance(c.left, c_ast.FuncCall)
                and isinstance(c.left.name, c_ast.ID) and c.left.name.name == 'sbufBytesRemaining'):
            if node.iffalse is not None or self.direction != 'in':
                raise Manual('optional tail with an else')
            saved, self.optional = self.optional, True
            self.run(node.iftrue)
            self.optional = saved
            return
        if self.is_error_exit(node.iftrue) and node.iffalse is None:
            self.guard(c)
            return
        # if (i < N) { ... } else { return MSP_RESULT_ERROR; }  -- the index bound
        if (isinstance(c, c_ast.BinaryOp) and c.op == '<' and isinstance(c.left, c_ast.ID)
                and self.env.get(c.left.name) == INDEX and self.index is not None and not self.ops):
            bound = self.value(c.right)
            if not isinstance(bound, int):
                raise Manual('index bound not constant')
            self.index['max'] = bound
            self.run(node.iftrue)
            return
        raise Manual('condition')


# --- manifest encoding ------------------------------------------------------------
#
# Codecs are unrolled, so the ops are many and alike; as keyed objects they
# more than doubled the manifest. In the manifest each op is a positional
# array instead:
#
#   ["f", w, pgn, off, size, flags]   a group field on the wire as w bytes
#   ["c", w, value, flags]            a constant (out) the firmware writes
#   ["s", w, flags]                   wire bytes the firmware ignores (in)
#   ["d", len, pgn, off, flags]       raw group bytes (sbufWriteData)
#
# flags is a string of letters:
#   o  optional: only present if the request still has bytes (in)
#   i  indexed: off is within the element the leading index byte selects
#   P R T  off is within the selected pid / rate / tv profile's element
#   s  the group field is signed          w  the wire value is signed (in)
# and a range check, when the firmware refused values outside it, follows as
# a seventh element {"min": a, "max": b} on "f" ops.

def _flags(op):
    return ''.join(letter for key, letter in (('optional', 'o'), ('indexed', 'i'), ('signed', 's'),
                                              ('wire_signed', 'w')) if op.get(key)) + \
        {'pid': 'P', 'rate': 'R', 'tv': 'T'}.get(op.get('profile'), '')


def compact(codec):
    ops = []
    for op in codec['ops']:
        kind = op['kind']
        if kind == 'field':
            item = ['f', op['w'], op['pgn'], op['off'], op['size'], _flags(op)]
            if 'check' in op:
                item.append(op['check'])
        elif kind == 'const':
            item = ['c', op['w'], op['value'], _flags(op)]
        elif kind == 'skip':
            item = ['s', op['w'], _flags(op)]
        else:
            item = ['d', op['len'], op['pgn'], op['off'], _flags(op)]
        ops.append(item)
    out = dict(codec)
    out['ops'] = ops
    return out


# --- driver ------------------------------------------------------------------

def extract(preprocessed_path, pgs, wanted=None, constants=None):
    """{opcode: codec} for the opcodes that are mechanical, and
    {opcode: reason} for the rest. `wanted` limits it to a set of opcodes;
    by default every case of the dispatch functions is tried."""
    if c_parser is None:
        return {}, {op: 'pycparser not installed' for op in wanted}

    with open(preprocessed_path, encoding='utf-8', errors='ignore') as f:
        src = f.read()
    groups = Groups(pgs)
    parser = c_parser.CParser()
    codecs, manual = {}, {}

    for fn, direction in FUNCTIONS.items():
        body = function_body(src, fn)
        if body is None:
            continue
        for op, text in case_bodies(body).items():
            if (wanted is not None and op not in wanted) or op in codecs:
                continue
            types = set(re.findall(r'\b(\w+_t|\w+_e)\b', text)) | {'uint8_t', 'uint16_t', 'uint32_t',
                                                                   'int8_t', 'int16_t', 'int32_t', 'uint', 'bool'}
            stub = ''.join('typedef int %s;\n' % t for t in sorted(types))
            code = stub + 'void f(void) {\n' + re.sub(r'\bconst\b', '', text) + '\n}\n'
            try:
                ast = parser.parse(code)
            except Exception as e:  # noqa: BLE001 -- any parse failure means manual
                manual[op] = 'does not parse (%s)' % str(e).split(':')[-1].strip()[:60]
                continue
            fdef = ast.ext[-1]
            ex = Executor(groups, direction, constants)
            try:
                ex.run(fdef.body)
                if not ex.ops:
                    raise Manual('no wire fields')
                if ex.index is not None and ex.index['max'] is None:
                    raise Manual('index without a bound check')
            except Manual as why:
                manual[op] = str(why)
                continue
            if any(o.get('local') and o['kind'] == 'skip' and 'check' in o for o in ex.ops):
                manual[op] = 'range-checked value never stored'
                continue
            for o in ex.ops:
                o.pop('local', None)
            codec = {'dir': direction, 'ops': ex.ops}
            codec.update(ex.checks)
            if ex.index is not None:
                codec['index'] = ex.index
            if ex.side_effects:
                codec['side_effects'] = sorted(set(ex.side_effects))
            codecs[op] = codec

    for op in wanted or ():
        if op not in codecs and op not in manual:
            manual[op] = 'not compiled into this target'
    return codecs, manual
