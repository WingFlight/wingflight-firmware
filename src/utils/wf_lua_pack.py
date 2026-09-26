#!/usr/bin/env python3
"""
Write a manifest's MSP codecs as a Lua pack for wingflight-lua-ethos-suite.

The suite on the radio answers the legacy config opcodes from PARAM_READ /
PARAM_WRITE the way the configurator's virtual MSP layer does, and needs the
same codecs (wf_msp_codecs.py). The radio has no network and little RAM, so
each firmware release publishes this pack next to its manifest and the suite
release bundles the packs of the releases it supports, keyed by build ID.

The pack is a Lua module returning:

    {
      format = 2,                                             -- this encoding
      build = "<16 hex digits>", target = "STM32F7X2", git = "<revision>",
      selection = { pid = <off>, rate = <off>, tv = <off> },  -- in systemConfig
      groups = { [pgn] = { <size>, <length> }, ... },         -- groups codecs use
      codecs = { [opcode] = "<binary>", ... },
    }

Codecs are binary strings rather than Lua tables: ~2000 unrolled ops as
tables would cost the radio an order of magnitude more RAM than the strings,
which the suite decodes one opcode at a time. The encoding, all integers
little-endian:

    header   dir u8 (0 reply, 1 setter)
             index u8 (0 none, else the index width), index_max u16,
             index_stride u16 (0: the group's element size),
             index_miss u8 (0 refused, 1 accepted and nothing stored),
             nmap u8 (0, or the ids naming elements 0..nmap-1)
             len_kind u8 (0 none, 1 exact, 2 minimum), len u16
             nops u16
             map u16 * nmap
    op       kind u8, then by kind:
      1 field   w u8, pgn u16, off u16, size u8, flags u8
                [+ if flags & 0x80: min s32, max s32 -- absent bound as
                 -2^31 / 2^31-1]
      2 const   w u8, value s32
      3 skip    w u8, flags u8
      4 data    len u16, pgn u16, off u16, flags u8
      5 select  a field (as 1, never a check) in the element a stored
                selector picks: w u8, pgn u16, off u16, size u8, flags u8,
                sel_pgn u16, sel_off u16, sel_size u8, stride u16, count u16
      6 str out len u16, pgn u16, off u16, flags u8 -- bytes up to the NUL
      7 str in  max u16, field_len u16, pgn u16, off u16, flags u8 -- the
                field cleared, then up to max request bytes copied in

    An index, a reply's too, is the request's first bytes. A reply codec with
    an index answers a request of exactly its index (len, if given).

    flags   0x01 optional  0x02 indexed  0x04 field signed  0x08 wire signed
            0x10 pid profile  0x20 rate profile  0x40 tv profile  0x80 check

    python src/utils/wf_lua_pack.py <manifest.json> <pack.lua>
"""

import json
import struct
import sys

FLAG_BITS = {'o': 0x01, 'i': 0x02, 's': 0x04, 'w': 0x08, 'P': 0x10, 'R': 0x20, 'T': 0x40}
SYSTEM_CONFIG_PGN = 18
SELECTION_FIELDS = {'pid': 'pidProfileIndex', 'rate': 'activeRateProfile', 'tv': 'tvProfileIndex'}
S32_MIN, S32_MAX = -2 ** 31, 2 ** 31 - 1
FORMAT = 2


def flag_byte(letters):
    value = 0
    for letter in letters:
        value |= FLAG_BITS[letter]
    return value


def encode_codec(codec):
    out = bytearray()
    index = codec.get('index')
    if 'len' in codec:
        len_kind, length = 1, codec['len']
    elif 'min_len' in codec:
        len_kind, length = 2, codec['min_len']
    else:
        len_kind, length = 0, 0
    ids = (index or {}).get('map') or []
    out += struct.pack('<BBHHBBBHH', 0 if codec['dir'] == 'out' else 1,
                       index['w'] if index else 0, index['max'] if index else 0,
                       index.get('stride', 0) if index else 0,
                       1 if (index or {}).get('miss') == 'ignore' else 0, len(ids),
                       len_kind, length, len(codec['ops']))
    out += struct.pack('<%dH' % len(ids), *ids)
    for op in codec['ops']:
        kind = op[0]
        if kind == 'f':
            _, w, pgn, off, size, flags = op[:6]
            check = op[6] if len(op) > 6 else None
            fb = flag_byte(flags) | (0x80 if check else 0)
            out += struct.pack('<BBHHBB', 1, w, pgn, off, size, fb)
            if check:
                out += struct.pack('<ii', check.get('min', S32_MIN), check.get('max', S32_MAX))
        elif kind == 'c':
            _, w, value, _flags = op
            out += struct.pack('<BBi', 2, w, value)
        elif kind == 's':
            _, w, flags = op
            out += struct.pack('<BBB', 3, w, flag_byte(flags))
        elif kind == 'd':
            _, length, pgn, off, flags = op
            out += struct.pack('<BHHHB', 4, length, pgn, off, flag_byte(flags))
        elif kind == 'x':
            _, w, pgn, off, size, flags, sel_pgn, sel_off, sel_size, stride, count = op
            out += struct.pack('<BBHHBBHHBHH', 5, w, pgn, off, size, flag_byte(flags),
                               sel_pgn, sel_off, sel_size, stride, count)
        elif kind == 'z':
            _, length, pgn, off, flags = op
            out += struct.pack('<BHHHB', 6, length, pgn, off, flag_byte(flags))
        elif kind == 'Z':
            _, most, field_len, pgn, off, flags = op
            out += struct.pack('<BHHHHB', 7, most, field_len, pgn, off, flag_byte(flags))
        else:
            raise SystemExit('unknown codec op %r' % (op,))
    return bytes(out)


def lua_string(data):
    # Decimal escapes only: valid in every Lua version, and no byte of the
    # binary can be mistaken for a quote or a line break.
    return '"' + ''.join('\\%d' % b for b in data) + '"'


def pack(manifest):
    codecs = manifest.get('msp_codecs') or {}
    groups = {pg['pgn']: pg for pg in manifest['pgs']}
    used = set()
    for codec in codecs.values():
        for op in codec['ops']:
            if op[0] in ('f', 'd', 'z'):
                used.add(op[2])
            elif op[0] == 'x':
                used.update((op[2], op[6]))
            elif op[0] == 'Z':
                used.add(op[3])
    system = groups.get(SYSTEM_CONFIG_PGN)
    selection = {}
    for kind, field in SELECTION_FIELDS.items():
        match = [f for f in (system or {}).get('fields', []) if f['name'] == field]
        if match:
            selection[kind] = match[0]['off']

    build = manifest.get('build', {})
    lines = [
        '-- MSP codecs for Wingflight build %s (%s, %s).' % (build.get('id'), build.get('target'),
                                                           build.get('git')),
        '-- Generated by src/utils/wf_lua_pack.py from the build\'s manifest; do not edit.',
        'return {',
        '  format = %d,' % FORMAT,
        '  build = "%s",' % build.get('id'),
        '  target = "%s",' % build.get('target'),
        '  git = "%s",' % build.get('git'),
        '  selection = { %s },' % ', '.join('%s = %d' % kv for kv in sorted(selection.items())),
        '  groups = {',
    ]
    for pgn in sorted(used):
        pg = groups[pgn]
        lines.append('    [%d] = { %d, %d },' % (pgn, pg['size'], pg['length']))
    lines.append('  },')
    lines.append('  codecs = {')
    for op in sorted(codecs, key=int):
        lines.append('    [%d] = %s,' % (int(op), lua_string(encode_codec(codecs[op]))))
    lines.append('  },')
    lines.append('}')
    return '\n'.join(lines) + '\n'


def main(argv):
    if len(argv) != 3:
        sys.stderr.write('usage: wf_lua_pack.py <manifest.json> <pack.lua>\n')
        return 2
    with open(argv[1]) as f:
        manifest = json.load(f)
    text = pack(manifest)
    with open(argv[2], 'w', newline='\n') as f:
        f.write(text)
    print('wrote %s: %d codecs, %d bytes' % (argv[2], len(manifest.get('msp_codecs') or {}), len(text)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
