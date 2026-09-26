#!/usr/bin/env python3
"""
Read a PE/COFF image the way wf_manifest.py reads an ELF.

The SITL target built natively on Windows with MinGW is a PE executable, not
an ELF, but it carries the same DWARF sections and a COFF symbol table. This
gives wf_manifest.py the few things it needs from it -- section contents by
virtual address, symbol addresses, the DWARF -- behind the interface of its
Elf class, so a SITL manifest can be made on Windows too. That is what lets
the configurator's virtual MSP layer be verified against SITL without a
board (parameter-addressing-design.md §14).

COFF symbols carry no sizes. A symbol's size is taken as the distance to the
next symbol in the same section, which is exact for the arrays the table
readers size this way, and an overestimate at worst for the last one.
"""

import io
import struct

from elftools.dwarf.dwarfinfo import DWARFInfo, DebugSectionDescriptor, DwarfConfig

DWARF_SECTIONS = ['.debug_info', '.debug_aranges', '.debug_abbrev', '.debug_frame', '.eh_frame',
                  '.debug_str', '.debug_loc', '.debug_ranges', '.debug_line', '.debug_pubtypes',
                  '.debug_pubnames', '.debug_addr', '.debug_str_offsets', '.debug_line_str',
                  '.debug_loclists', '.debug_rnglists', '.debug_sup', '.gnu_debugaltlink', '.debug_types']

MACHINES = {0x8664: ('x64', 8), 0x14c: ('x86', 4), 0xaa64: ('AArch64', 8)}


def is_pe(path):
    with open(path, 'rb') as f:
        return f.read(2) == b'MZ'


class _Dwarf(object):
    """The two ELFFile methods wf_manifest.Dwarf calls."""

    def __init__(self, image):
        self.image = image

    def has_dwarf_info(self):
        return '.debug_info' in self.image.sections

    def get_dwarf_info(self):
        def descriptor(name):
            section = self.image.sections.get(name)
            if section is None:
                return None
            data = section['data']
            return DebugSectionDescriptor(io.BytesIO(data), name, section['raw_offset'], len(data), 0)

        arch, address_size = MACHINES.get(self.image.machine, ('x64', 8))
        config = DwarfConfig(little_endian=True, machine_arch=arch, default_address_size=address_size)
        return DWARFInfo(config, *[descriptor(name) for name in DWARF_SECTIONS])


class PeImage(object):
    """PE/COFF behind wf_manifest.Elf's interface."""

    def __init__(self, path):
        self.path = path
        with open(path, 'rb') as f:
            self.raw = f.read()
        raw = self.raw

        pe = struct.unpack_from('<I', raw, 0x3c)[0]
        if raw[pe:pe + 4] != b'PE\0\0':
            raise SystemExit('%s is not a PE image' % path)
        (self.machine, nsections, _, symtab, nsymbols, optsize, _) = struct.unpack_from('<HHIIIHH', raw, pe + 4)
        opt = pe + 24
        magic = struct.unpack_from('<H', raw, opt)[0]
        self.pointer_size = 8 if magic == 0x20b else 4
        self.image_base = struct.unpack_from('<Q', raw, opt + 24)[0] if magic == 0x20b \
            else struct.unpack_from('<I', raw, opt + 28)[0]
        self.little_endian = True

        strtab = symtab + nsymbols * 18

        def long_name(field):
            name = field.rstrip(b'\0')
            if name.startswith(b'/'):
                off = int(name[1:])
                end = raw.index(b'\0', strtab + off)
                return raw[strtab + off:end].decode('ascii', 'replace')
            return name.decode('ascii', 'replace')

        self.sections = {}
        self._by_index = []
        base = opt + optsize
        for i in range(nsections):
            h = base + i * 40
            name = long_name(raw[h:h + 8])
            vsize, vaddr, rawsize, rawptr = struct.unpack_from('<IIII', raw, h + 8)
            section = {'name': name, 'va': self.image_base + vaddr, 'vsize': vsize,
                       'raw_offset': rawptr, 'data': raw[rawptr:rawptr + min(rawsize, vsize or rawsize)]}
            self.sections.setdefault(name, section)
            self._by_index.append(section)

        # COFF symbols: name, value (section-relative), section number.
        self.symbols = {}
        by_section = {}
        i = 0
        while i < nsymbols:
            s = symtab + i * 18
            field = raw[s:s + 8]
            if field[:4] == b'\0\0\0\0':
                off = struct.unpack_from('<I', field, 4)[0]
                end = raw.index(b'\0', strtab + off)
                name = raw[strtab + off:end].decode('ascii', 'replace')
            else:
                name = field.rstrip(b'\0').decode('ascii', 'replace')
            value, secnum, _, _, naux = struct.unpack_from('<IhHBB', raw, s + 8)
            if secnum > 0 and name and not name.startswith('.'):
                section = self._by_index[secnum - 1]
                address = section['va'] + value
                self.symbols.setdefault(name, address)
                by_section.setdefault(secnum, set()).add(address)
            i += 1 + naux

        # Sizes: to the next symbol in the section, or the section's end.
        bounds = {}
        for secnum, addresses in by_section.items():
            section = self._by_index[secnum - 1]
            ordered = sorted(addresses) + [section['va'] + section['vsize']]
            for a, b in zip(ordered, ordered[1:]):
                bounds[a] = b - a
        self.symbol_sizes = {name: bounds.get(address, 0) for name, address in self.symbols.items()}

        self.elf = _Dwarf(self)

    def section_data(self, name):
        section = self.sections.get(name)
        return section['data'] if section else None

    def read_at(self, address, length):
        for section in self._by_index:
            if section['va'] <= address < section['va'] + section['vsize']:
                offset = address - section['va']
                if offset >= len(section['data']):
                    return None  # uninitialised data: nothing in the file
                return section['data'][offset:offset + length]
        return None

    def read_pointer(self, address):
        raw = self.read_at(address, self.pointer_size)
        if raw is None or len(raw) < self.pointer_size:
            return None
        return int.from_bytes(raw, 'little')

    def read_cstring(self, address, limit=256):
        raw = self.read_at(address, limit)
        if not raw:
            return None
        end = raw.find(b'\0')
        if end < 0:
            return None
        return raw[:end].decode('utf-8', 'replace')
