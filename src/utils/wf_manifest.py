#!/usr/bin/env python3
#
# This file is part of Wingflight.
#
# Wingflight is free software. You can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Wingflight is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software. If not, see <https://www.gnu.org/licenses/>.

"""Extract a parameter manifest from a linked Wingflight ELF.

The firmware describes its own configuration twice: once in the parameter
group registry (.pg_registry, which it needs at runtime to load and save
config) and once in cli/settings.c (~20 KB of names, ranges and labels, which
only off-board consumers ever read). This tool reconstructs the second from
the first plus DWARF, so that the second can stop being linked into flash.

The output is keyed by (pgn, offset), which is exactly how the firmware's
parameter read/write opcodes address a field, and deliberately contains no
absolute addresses -- the manifest must stay stable across relinks that do
not change struct layout, because the build ID is a hash of it.

Requires a non-LTO build with debug info:

    make TARGET=STM32F411 DEBUG=INFO
    python3 src/utils/wf_manifest.py obj/main/wingflight_STM32F411.elf manifest.json

LTO collapses the per-CU DWARF and no group will resolve; the tool says so
rather than emitting a half-empty manifest.
"""

import hashlib
import json
import struct
import sys

try:
    from elftools.elf.elffile import ELFFile
except ImportError:
    sys.stderr.write('pyelftools is required: pip3 install pyelftools\n')
    sys.exit(2)


# DW_ATE_* -> the kind name used in the manifest. Anything not listed here is
# not something a config field should be made of, and shows up as '?' so the
# gap is visible rather than silently mistyped.
DWARF_ENCODING = {
    0x02: 'bool',
    0x04: 'float',
    0x05: 'int',
    0x06: 'int',      # signed char
    0x07: 'uint',
    0x08: 'uint',     # unsigned char
    0x0d: 'int',      # signed fixed
    0x0e: 'uint',     # unsigned fixed
}

# Type DIEs that are transparent for layout purposes: the thing they wrap sits
# at the same offset and has the same size.
TRANSPARENT_TAGS = (
    'DW_TAG_typedef',
    'DW_TAG_const_type',
    'DW_TAG_volatile_type',
    'DW_TAG_restrict_type',
    'DW_TAG_atomic_type',
)

MANIFEST_SCHEMA = 1

# Truncated SHA-256. This is a collision guard between builds, not a security
# boundary; 64 bits is ample and keeps the MSP reply small.
BUILD_ID_BYTES = 8


def attr(die, name, default=None):
    a = die.attributes.get(name)
    return a.value if a is not None else default


def name_of(die):
    value = attr(die, 'DW_AT_name')
    return value.decode('utf-8') if isinstance(value, bytes) else value


class Elf(object):
    """Section data and the symbol table, by name."""

    def __init__(self, path):
        self.path = path
        self._file = open(path, 'rb')
        self.elf = ELFFile(self._file)
        self.little_endian = self.elf.little_endian
        self.pointer_size = self.elf.elfclass // 8

        self.symbols = {}
        for section in self.elf.iter_sections():
            if section.header['sh_type'] not in ('SHT_SYMTAB', 'SHT_DYNSYM'):
                continue
            for symbol in section.iter_symbols():
                if symbol.name and symbol['st_info']['type'] == 'STT_OBJECT':
                    self.symbols[symbol.name] = symbol['st_value']

    def section_data(self, name):
        section = self.elf.get_section_by_name(name)
        return section.data() if section is not None else None

    def read_at(self, address, length):
        """Bytes at a virtual address, or None if nothing is mapped there."""
        for section in self.elf.iter_sections():
            header = section.header
            start = header['sh_addr']
            if not start or not (start <= address < start + header['sh_size']):
                continue
            if header['sh_type'] == 'SHT_NOBITS':
                return None    # .bss: no contents in the file
            offset = address - start
            return section.data()[offset:offset + length]
        return None

    def read_pointer(self, address):
        raw = self.read_at(address, self.pointer_size)
        if raw is None or len(raw) < self.pointer_size:
            return None
        return int.from_bytes(raw, 'little' if self.little_endian else 'big')

    def read_cstring(self, address, limit=256):
        raw = self.read_at(address, limit)
        if not raw:
            return None
        end = raw.find(b'\0')
        if end < 0:
            return None
        return raw[:end].decode('utf-8', 'replace')


class Dwarf(object):
    """The DWARF type graph, indexed by the variable names we care about.

    Type DIEs are referenced by CU-relative offset, so the owning CU has to be
    carried alongside every DIE; `resolve` does that bookkeeping.
    """

    def __init__(self, elf, wanted):
        if not elf.elf.has_dwarf_info():
            raise SystemExit('%s has no debug info -- rebuild with DEBUG=INFO' % elf.path)

        self.dwarf = elf.elf.get_dwarf_info()
        self.variables = {}   # name -> (die, cu)
        self.structs = {}     # struct/union name -> (die, cu), first definition wins

        for cu in self.dwarf.iter_CUs():
            for die in cu.get_top_DIE().iter_children():
                if die.tag == 'DW_TAG_variable':
                    identifier = name_of(die)
                    if identifier in wanted and 'DW_AT_type' in die.attributes:
                        # A tentative definition can appear in several CUs; the
                        # first one carrying a type is as good as any, they
                        # describe the same object.
                        self.variables.setdefault(identifier, (die, cu))
                elif die.tag in ('DW_TAG_structure_type', 'DW_TAG_union_type'):
                    identifier = name_of(die)
                    if identifier and 'DW_AT_byte_size' in die.attributes:
                        self.structs.setdefault(identifier, (die, cu))

    def resolve(self, die, cu):
        """Follow DW_AT_type once."""
        ref = attr(die, 'DW_AT_type')
        if ref is None:
            return None, None
        target = self.dwarf.get_DIE_from_refaddr(cu.cu_offset + ref, cu)
        return target, target.cu

    def strip(self, die, cu):
        """Drop typedefs and cv-qualifiers down to the type that has a layout."""
        while die is not None and die.tag in TRANSPARENT_TAGS:
            die, cu = self.resolve(die, cu)
        return die, cu

    def type_of(self, die, cu):
        return self.strip(*self.resolve(die, cu))

    def walk(self, die, cu, path, base, out, depth=0):
        """Flatten a type into manifest field records.

        `path` is the dotted field path so far and `base` the byte offset from
        the start of the parameter group. Records are emitted for leaves only;
        nested structs contribute their prefix and disappear.
        """
        if die is None or depth > 16:
            return

        if die.tag in ('DW_TAG_structure_type', 'DW_TAG_union_type'):
            for member in die.iter_children():
                if member.tag != 'DW_TAG_member':
                    continue
                if 'DW_AT_bit_size' in member.attributes:
                    # No config struct uses bitfields today. If one starts to,
                    # its offset is not a whole number of bytes and the wire
                    # protocol cannot address it -- say so loudly.
                    sys.stderr.write('warning: %s.%s is a bitfield and is not addressable\n'
                                     % (path or name_of(die), name_of(member)))
                    continue
                offset = attr(member, 'DW_AT_data_member_location', 0)
                identifier = name_of(member)
                child_path = '%s.%s' % (path, identifier) if path else identifier
                self.walk(*self.type_of(member, cu),
                          path=child_path, base=base + offset, out=out, depth=depth + 1)

        elif die.tag == 'DW_TAG_array_type':
            element, element_cu = self.type_of(die, cu)
            if element is None:
                return
            stride = attr(element, 'DW_AT_byte_size', 1)
            count = 0
            for child in die.iter_children():
                if child.tag == 'DW_TAG_subrange_type':
                    if 'DW_AT_upper_bound' in child.attributes:
                        count = attr(child, 'DW_AT_upper_bound') + 1
                    elif 'DW_AT_count' in child.attributes:
                        count = attr(child, 'DW_AT_count')

            if element.tag in ('DW_TAG_structure_type', 'DW_TAG_union_type'):
                # Repeated struct: describe one element plus the stride. The
                # client addresses element N as index * stride + field offset,
                # which is exactly what the firmware bounds-checks against.
                fields = []
                self.walk(element, element_cu, '', 0, fields, depth + 1)
                out.append({'name': path, 'off': base, 'kind': 'repeat',
                            'stride': stride, 'count': count, 'fields': fields})
            else:
                record = {'name': path, 'off': base, 'kind': 'array',
                          'elem_size': stride, 'count': count,
                          'elem_kind': DWARF_ENCODING.get(attr(element, 'DW_AT_encoding'), '?')}
                if element.tag == 'DW_TAG_enumeration_type':
                    record['elem_kind'] = 'enum'
                    record['values'] = self.enumerators(element)
                out.append(record)

        elif die.tag == 'DW_TAG_enumeration_type':
            out.append({'name': path, 'off': base, 'kind': 'enum',
                        'size': attr(die, 'DW_AT_byte_size', 4),
                        'signed': self.enum_is_signed(die, cu),
                        'values': self.enumerators(die)})

        elif die.tag == 'DW_TAG_base_type':
            out.append({'name': path, 'off': base,
                        'kind': DWARF_ENCODING.get(attr(die, 'DW_AT_encoding'), '?'),
                        'size': attr(die, 'DW_AT_byte_size')})

        elif die.tag == 'DW_TAG_pointer_type':
            # A pointer in a parameter group would be a bug: the value is a RAM
            # address, meaningless to a client and not portable across builds.
            sys.stderr.write('warning: %s is a pointer and was skipped\n' % path)

    def enumerators(self, die):
        return {name_of(c): attr(c, 'DW_AT_const_value')
                for c in die.iter_children() if c.tag == 'DW_TAG_enumerator'}

    def enum_is_signed(self, die, cu):
        """An enum with a fixed underlying type carries it; otherwise infer."""
        underlying, _ = self.type_of(die, cu)
        if underlying is not None and underlying.tag == 'DW_TAG_base_type':
            return DWARF_ENCODING.get(attr(underlying, 'DW_AT_encoding')) == 'int'
        return any(v < 0 for v in self.enumerators(die).values())

    def struct_layout(self, identifier):
        """Member name -> (offset, size) for a named struct."""
        entry = self.structs.get(identifier)
        if entry is None:
            raise SystemExit('%s not found in debug info' % identifier)
        die, cu = entry
        layout = {}
        for member in die.iter_children():
            if member.tag != 'DW_TAG_member':
                continue
            member_type, _ = self.type_of(member, cu)
            layout[name_of(member)] = (attr(member, 'DW_AT_data_member_location', 0),
                                       attr(member_type, 'DW_AT_byte_size', 0) if member_type else 0)
        return layout, attr(die, 'DW_AT_byte_size')


# Registry field masks, mirroring pgRegistryInternal_e in pg/pg.h.
PGR_PGN_MASK = 0x0fff
PGR_SIZE_MASK = 0x0fff


def read_registry(elf, dwarf):
    """Parse .pg_registry into one record per parameter group.

    The record layout is taken from DWARF rather than hardcoded, because it
    depends on pointer size: 28 bytes on 32-bit ARM, 56 on a native x86-64
    SITL build. Reading it out of the debug info means the tool works on both
    without a target-specific table.
    """
    data = elf.section_data('.pg_registry')
    if data is None:
        raise SystemExit('no .pg_registry section -- is this a linked firmware ELF?')

    layout, record_size = dwarf.struct_layout('pgRegistry_s')
    if not record_size or len(data) % record_size:
        raise SystemExit('.pg_registry is %d bytes, not a multiple of the %s-byte record'
                         % (len(data), record_size))

    endian = '<' if elf.little_endian else '>'
    scalar = {1: 'B', 2: 'H', 4: 'I', 8: 'Q'}

    def field(record, member):
        offset, size = layout[member]
        return struct.unpack_from(endian + scalar[size], record, offset)[0]

    groups = []
    for start in range(0, len(data), record_size):
        record = data[start:start + record_size]
        pgn = field(record, 'pgn')
        size = field(record, 'size')
        length = field(record, 'length') or 1
        groups.append({
            'pgn': pgn & PGR_PGN_MASK,
            'version': pgn >> 12,
            'length': length,
            'size': size & PGR_SIZE_MASK,
            'address': field(record, 'address'),
        })
    return groups


def count_leaves(fields):
    total = 0
    for field in fields:
        total += count_leaves(field['fields']) if field['kind'] == 'repeat' else 1
    return total


def canonical(manifest):
    """Stable bytes for hashing: sorted keys, no incidental whitespace."""
    return json.dumps(manifest, sort_keys=True, separators=(',', ':')).encode('utf-8')


def build_id(manifest):
    """The build ID is the hash of the manifest with the ID field left out.

    Hashing the output rather than the inputs is what makes a candidate
    manifest checkable: there is no question of whether every input that
    affects struct layout was accounted for, because the manifest is the
    layout. If it hashes to what the board reports, it is the right one.
    """
    without_id = dict(manifest)
    without_id['build'] = {k: v for k, v in manifest['build'].items() if k != 'id'}
    return hashlib.sha256(canonical(without_id)).hexdigest()[:BUILD_ID_BYTES * 2]


def read_build_info(elf):
    """Target, version and revision, read back out of the binary.

    version.h declares these as `const char* const`, so the symbol holds a
    pointer and the string is somewhere else -- dereference once, and fall back
    to reading at the symbol itself in case one is ever a plain char array.
    """
    info = {}
    for symbol, key in (('targetName', 'target'),
                        ('shortGitRevision', 'git'),
                        ('buildDate', 'date'),
                        ('buildTime', 'time')):
        address = elf.symbols.get(symbol)
        if address is None:
            continue
        pointer = elf.read_pointer(address)
        value = elf.read_cstring(pointer) if pointer else None
        if value is None:
            value = elf.read_cstring(address)
        if value is not None:
            info[key] = value
    return info


def main(argv):
    if len(argv) != 3:
        sys.stderr.write('usage: %s <firmware.elf> <manifest.json>\n' % argv[0])
        return 2

    elf = Elf(argv[1])

    # PG_REGISTER names the group's storage <name>_System / <name>_SystemArray,
    # so the registry's address field maps straight back to a DWARF variable.
    address_to_symbol = {addr: name for name, addr in elf.symbols.items()
                         if name.endswith('_System') or name.endswith('_SystemArray')}

    wanted = set(address_to_symbol.values())
    wanted.add('pgRegistry_s')
    dwarf = Dwarf(elf, wanted)
    groups = read_registry(elf, dwarf)

    manifest = {'schema': MANIFEST_SCHEMA, 'build': read_build_info(elf), 'pgs': []}
    resolved = 0
    unresolved = []
    for group in groups:
        symbol = address_to_symbol.get(group['address'])
        entry = {'pgn': group['pgn'], 'version': group['version'],
                 'length': group['length'], 'size': group['size'],
                 'elem_size': group['size'] // group['length'],
                 'symbol': symbol, 'fields': []}
        variable = dwarf.variables.get(symbol) if symbol else None
        if variable:
            die, cu = variable
            dwarf.walk(*dwarf.type_of(die, cu),
                       path='', base=0, out=entry['fields'])
            if entry['fields']:
                resolved += 1
            else:
                unresolved.append(symbol)
        else:
            unresolved.append(symbol or '@0x%x' % group['address'])
        manifest['pgs'].append(entry)

    manifest['pgs'].sort(key=lambda pg: pg['pgn'])
    manifest['build']['id'] = build_id(manifest)

    leaves = sum(count_leaves(pg['fields']) for pg in manifest['pgs'])
    print('PGs in registry: %d   with DWARF fields: %d   leaf fields: %d'
          % (len(groups), resolved, leaves))
    print('build id: %s' % manifest['build']['id'])

    if resolved == 0:
        sys.stderr.write('\nNo group resolved. This is what an LTO build looks like - '
                         'rebuild with -flto removed and DEBUG=INFO.\n')
        return 1
    if unresolved:
        sys.stderr.write('\nWarning: %d group(s) had no DWARF type: %s\n'
                         % (len(unresolved), ', '.join(sorted(unresolved))))

    with open(argv[2], 'w') as out:
        json.dump(manifest, out, indent=1, sort_keys=True)
        out.write('\n')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
