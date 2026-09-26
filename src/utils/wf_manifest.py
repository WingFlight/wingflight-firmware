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

import argparse
import hashlib
import json
import os
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
        self.symbol_sizes = {}
        for section in self.elf.iter_sections():
            if section.header['sh_type'] not in ('SHT_SYMTAB', 'SHT_DYNSYM'):
                continue
            for symbol in section.iter_symbols():
                if not symbol.name:
                    continue
                # fullTimerHardware is emitted as STT_FUNC on some builds
                # because it lands in .text; accept both rather than miss it.
                if symbol['st_info']['type'] in ('STT_OBJECT', 'STT_FUNC'):
                    self.symbols.setdefault(symbol.name, symbol['st_value'])
                    self.symbol_sizes.setdefault(symbol.name, symbol['st_size'])

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
        self._typedefs = {}   # typedef name -> (die, cu), for anonymous structs
        self.enum_constants = {}  # enumerator name -> value, for the MSP codec extractor

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
                elif die.tag == 'DW_TAG_enumeration_type':
                    # msp.c sizes and bounds things by enumerators
                    # (MIXER_IN_COUNT, PID_ROLL), which the preprocessor
                    # leaves as names; DWARF has their values.
                    for child in die.iter_children():
                        if child.tag == 'DW_TAG_enumerator':
                            self.enum_constants.setdefault(name_of(child), attr(child, 'DW_AT_const_value'))
                elif die.tag == 'DW_TAG_typedef':
                    # `typedef struct { ... } foo_t;` leaves the struct itself
                    # anonymous, so it is only reachable by the typedef name.
                    # Index those too, or such a type looks absent entirely.
                    identifier = name_of(die)
                    if identifier:
                        self._typedefs.setdefault(identifier, (die, cu))

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
                if identifier is None:
                    # An anonymous struct or union member: its fields belong to
                    # the enclosing struct and contribute no path segment.
                    # sensorAlignment_t does this to overlay roll/pitch/yaw on
                    # a raw[3] array.
                    child_path = path
                else:
                    child_path = '%s.%s' % (path, identifier) if path else identifier
                self.walk(*self.type_of(member, cu),
                          path=child_path, base=base + offset, out=out, depth=depth + 1)

        elif die.tag == 'DW_TAG_array_type':
            element, element_cu = self.type_of(die, cu)
            if element is None:
                return
            stride = attr(element, 'DW_AT_byte_size', 1)

            # A multi-dimensional array is one DW_TAG_array_type with one
            # subrange per dimension, so the dimensions have to be multiplied
            # rather than the last one taken: rpmFilterConfig's
            # notch_source[3][16] is 48 elements, not 16. Getting this wrong
            # under-reports where the array ends, and the client then believes
            # the next field starts inside it.
            dims = []
            for child in die.iter_children():
                if child.tag == 'DW_TAG_subrange_type':
                    if 'DW_AT_upper_bound' in child.attributes:
                        dims.append(attr(child, 'DW_AT_upper_bound') + 1)
                    elif 'DW_AT_count' in child.attributes:
                        dims.append(attr(child, 'DW_AT_count'))
                    else:
                        dims.append(0)    # flexible array member

            count = 1
            for dim in dims:
                count *= dim
            if not dims:
                count = 0

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
                if len(dims) > 1:
                    # Kept so a client can present [3][16] as the two
                    # dimensions it is; `count` stays the flat element total,
                    # which is what the addressing arithmetic needs.
                    record['dims'] = dims
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
        """Member name -> (offset, size) for a named struct or typedef."""
        entry = self.structs.get(identifier)
        if entry is None:
            # `typedef struct { ... } foo_t;` leaves the struct anonymous, so
            # it is reachable only through the typedef.
            typedef = self._typedefs.get(identifier)
            if typedef is not None:
                resolved = self.type_of(*typedef)
                if resolved[0] is not None and 'DW_AT_byte_size' in resolved[0].attributes:
                    entry = resolved
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
    # The linker script's own bounds, when it provides them: a PE section is
    # padded past its contents, so the section alone overstates the registry.
    start = elf.symbols.get('__pg_registry_start')
    end = elf.symbols.get('__pg_registry_end')
    if start is not None and end is not None and end >= start:
        data = elf.read_at(start, end - start) if end > start else b''
    else:
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


# cliValueFlag_e in cli/settings.h.
VALUE_TYPE_MASK = 0x07
VALUE_SECTION_MASK = 0x38
VALUE_MODE_MASK = 0x1C0

VALUE_TYPE = {
    0: ('uint', 1),   # VAR_UINT8
    1: ('int', 1),    # VAR_INT8
    2: ('uint', 2),   # VAR_UINT16
    3: ('int', 2),    # VAR_INT16
    4: ('uint', 4),   # VAR_UINT32
    5: ('int', 4),    # VAR_INT32
}

VALUE_SECTION = {
    0: 'master', 1: 'profile', 2: 'rate_profile', 3: 'hardware', 4: 'tv_profile',
}

VALUE_MODE = {
    0: 'direct', 1: 'lookup', 2: 'array', 3: 'bitset', 4: 'string',
}


def read_lookup_tables(elf, dwarf):
    """The enum label tables the CLI offers for MODE_LOOKUP settings.

    These are the dropdown contents for anything that is not a plain number,
    and the firmware is the only place they exist -- so a client that wants to
    show 'GYRO_HARDWARE_LPF_NORMAL' rather than '0' needs them from here.
    """
    table = elf.symbols.get('lookupTables')
    if table is None:
        return []

    layout, record_size = dwarf.struct_layout('lookupTableEntry_s')
    values_offset = layout['values'][0]
    count_offset = layout['valueCount'][0]

    tables = []
    index = 0
    while True:
        record = elf.read_at(table + index * record_size, record_size)
        if record is None or len(record) < record_size:
            break
        pointer = elf.read_pointer(table + index * record_size + values_offset)
        count = record[count_offset]
        # The array is not length-prefixed; it ends where the CLI's
        # LOOKUP_TABLE_COUNT says, which is not in the binary. A null values
        # pointer is the reliable end marker.
        if not pointer or count == 0 or count > 255:
            break
        labels = []
        for slot in range(count):
            label_pointer = elf.read_pointer(pointer + slot * elf.pointer_size)
            labels.append(elf.read_cstring(label_pointer) if label_pointer else None)
        tables.append(labels)
        index += 1
    return tables


def read_resource_table(elf, dwarf):
    """Which parameter group holds which pin assignment.

    `resource MOTOR 1 A09` is not runtime state, which is what makes it
    reproducible off-board: resourceTable maps an owner to a group and an
    offset inside it, and the pin bytes themselves are ordinary parameter group
    contents a client can already read. So the client needs the table and the
    owner names, and can format the rest itself.
    """
    table = elf.symbols.get('resourceTable')
    names = elf.symbols.get('ownerNames')
    if table is None or names is None:
        return []

    layout, record_size = dwarf.struct_layout('cliResourceValue_t')
    endian = '<' if elf.little_endian else '>'
    scalar = {1: 'B', 2: 'H', 4: 'I', 8: 'Q'}

    # ownerNames is indexed by the owner enum; read as far as the table needs.
    def owner_name(index):
        pointer = elf.read_pointer(names + index * elf.pointer_size)
        return elf.read_cstring(pointer) if pointer else None

    entries = []
    index = 0
    while True:
        record = elf.read_at(table + index * record_size, record_size)
        if record is None or len(record) < record_size:
            break

        def field(member, record=record):
            offset, size = layout[member]
            return struct.unpack_from(endian + scalar[size], record, offset)[0]

        owner = field('owner')
        pgn = field('pgn')
        # The array is not length-prefixed. A zero pgn is not a real group, so
        # it marks the end as reliably as anything available.
        if pgn == 0:
            break

        entries.append({
            'owner': owner,
            'name': owner_name(owner),
            'pgn': pgn,
            'stride': field('stride'),
            'off': field('offset'),
            # maxIndex 0 means a single pin rather than an array, which is what
            # RESOURCE_VALUE_MAX_INDEX() encoded in the firmware.
            'count': field('maxIndex') or 1,
        })
        index += 1
    return entries


def read_dmaopt_table(elf, dwarf):
    """Which parameter group holds which DMA option.

    The same shape as the resource table -- a device name, a group, and an
    offset inside it -- so `dma ADC 1 0` is reproducible off-board too. The
    per-pin lines (`dma pin A02 0`) do not need this: they come from
    timerIOConfig's own dmaopt byte.
    """
    table = elf.symbols.get('dmaoptEntryTable')
    if table is None:
        return []

    try:
        layout, record_size = dwarf.struct_layout('dmaoptEntry_s')
    except SystemExit:
        return []

    endian = '<' if elf.little_endian else '>'
    scalar = {1: 'B', 2: 'H', 4: 'I', 8: 'Q'}

    entries = []
    index = 0
    while True:
        record = elf.read_at(table + index * record_size, record_size)
        if record is None or len(record) < record_size:
            break

        def field(member, record=record):
            offset, size = layout[member]
            return struct.unpack_from(endian + scalar[size], record, offset)[0]

        device_pointer = elf.read_pointer(table + index * record_size + layout['device'][0])
        device = elf.read_cstring(device_pointer) if device_pointer else None
        pgn = field('pgn')
        if not device or not pgn:
            break

        entries.append({
            'device': device,
            'pgn': pgn,
            'stride': field('stride'),
            'off': field('offset'),
            'count': field('maxIndex') or 1,
        })
        index += 1
    return entries


def read_timer_hardware(elf, dwarf):
    """The pin-to-timer map, for rendering `timer A09 AF1`.

    This is the one piece of a board config that really is target hardware
    rather than configuration: which alternate function a pin needs for a given
    timer. It is a const table in the image, so it extracts like any other --
    the firmware does not have to grow an opcode to report it.

    Only what the CLI printed is kept: the pin and its alternate function, in
    array order, because timerGetByTagAndIndex() selects the Nth entry matching
    a tag and the client has to be able to do the same.
    """
    address = elf.symbols.get('fullTimerHardware')
    size = elf.symbol_sizes.get('fullTimerHardware')
    if not address or not size:
        return []

    try:
        layout, record_size = dwarf.struct_layout('timerHardware_s')
    except SystemExit:
        return []
    if not record_size or 'tag' not in layout or 'alternateFunction' not in layout:
        return []

    tag_offset = layout['tag'][0]
    af_offset = layout['alternateFunction'][0]

    entries = []
    for index in range(size // record_size):
        record = elf.read_at(address + index * record_size, record_size)
        if record is None or len(record) < record_size:
            break
        entries.append({'tag': record[tag_offset], 'af': record[af_offset]})
    return entries


def read_cli_tables(elf, dwarf):
    """What the `feature`, `serial`, `aux` and `map` lines need beyond PG bytes.

    Those lines are in every backup, and each is a parameter group rendered
    through a table: feature bits by name, baud rates by index, modes by their
    permanent id rather than the build-specific box id, rcmap as channel
    letters. The tables are const data in the image, so they extract like the
    timer map; featureNames and the aux channel count come from
    manifest/cli_tables.c, having had no other home once cli.c went.

    Anything missing is left out rather than guessed, and the client refuses
    the matching command.
    """
    endian = '<' if elf.little_endian else '>'
    tables = {}

    names = elf.symbols.get('featureNames')
    if names is not None:
        features = []
        for bit in range(32):
            pointer = elf.read_pointer(names + bit * elf.pointer_size)
            if not pointer:
                break
            name = elf.read_cstring(pointer)
            if name:
                features.append({'bit': bit, 'name': name})
        tables['features'] = features

    address = elf.symbols.get('baudRates')
    size = elf.symbol_sizes.get('baudRates')
    if address and size:
        raw = elf.read_at(address, size)
        tables['baud_rates'] = list(struct.unpack_from(endian + 'I' * (size // 4), raw))

    # The ports this target has, in portConfigs order. serialInit() marks
    # exactly these available, and `serial` skipped any other.
    variable = dwarf.variables.get('serialPortIdentifiers')
    address = elf.symbols.get('serialPortIdentifiers')
    if variable and address:
        fields = []
        dwarf.walk(*dwarf.type_of(*variable), path='', base=0, out=fields)
        if fields and fields[0].get('kind') == 'array':
            elem = fields[0]['elem_size']
            raw = elf.read_at(address, elem * fields[0]['count'])
            code = {1: 'b', 2: 'h', 4: 'i'}[elem]
            tables['serial_ports'] = list(struct.unpack_from(endian + code * fields[0]['count'], raw))

    # `aux` names a mode by its permanent id; the group stores the box id.
    address = elf.symbols.get('boxes')
    size = elf.symbol_sizes.get('boxes')
    if address and size:
        try:
            layout, record_size = dwarf.struct_layout('box_s')
        except SystemExit:
            layout, record_size = None, 0
        if record_size:
            boxes = []
            for index in range(size // record_size):
                base = address + index * record_size
                record = elf.read_at(base, record_size)
                pointer = elf.read_pointer(base + layout['boxName'][0])
                boxes.append({
                    'id': record[layout['boxId'][0]],
                    'perm': record[layout['permanentId'][0]],
                    'name': elf.read_cstring(pointer) if pointer else None,
                })
            tables['boxes'] = boxes

    address = elf.symbols.get('rcChannelLetters')
    if address:
        tables['rc_letters'] = elf.read_cstring(address)

    address = elf.symbols.get('cliAuxChannelCount')
    if address:
        tables['aux_channel_count'] = elf.read_at(address, 1)[0]

    # Name arrays indexed by enum value, sized by their symbol.
    for symbol, key in (('mixerInputNames', 'mixer_inputs'),
                        ('mixerOutputNames', 'mixer_outputs'),
                        ('mixerOpNames', 'mixer_ops'),
                        # `status`
                        ('mcuTypeNames', 'mcu_types'),
                        ('configurationStateNames', 'configuration_states'),
                        ('armingDisableFlagNames', 'arming_disable_flags'),
                        ('batteryStateStrings', 'battery_states')):
        address = elf.symbols.get(symbol)
        size = elf.symbol_sizes.get(symbol)
        if address and size:
            names = []
            for index in range(size // elf.pointer_size):
                pointer = elf.read_pointer(address + index * elf.pointer_size)
                names.append(elf.read_cstring(pointer) if pointer else None)
            tables[key] = names

    # `beeper` names a condition; the group stores 1 << (mode - 1). Table
    # order matters: the CLI special-cased the entry at index BEEPER_ALL - 1.
    address = elf.symbols.get('beeperTable')
    size = elf.symbol_sizes.get('beeperTable')
    if address and size:
        try:
            layout, record_size = dwarf.struct_layout('beeperTableEntry_s')
        except SystemExit:
            layout, record_size = None, 0
        if record_size:
            beepers = []
            for index in range(size // record_size):
                base = address + index * record_size
                record = elf.read_at(base, record_size)
                pointer = elf.read_pointer(base + layout['name'][0])
                beepers.append({'mode': record[layout['mode'][0]],
                                'name': elf.read_cstring(pointer) if pointer else None})
            tables['beepers'] = beepers

    # Every member of cliLimits is an int32_t; read them by name so the
    # struct's order is free to change.
    address = elf.symbols.get('cliLimits')
    if address:
        try:
            layout, record_size = dwarf.struct_layout('cliLimits_s')
        except SystemExit:
            layout = None
        if layout:
            raw = elf.read_at(address, record_size)
            tables['limits'] = {name: struct.unpack_from(endian + 'i', raw, offset)[0]
                                for name, (offset, size) in layout.items() if size == 4}

    return tables


def read_value_table(elf, dwarf, tables):
    """Decode valueTable -- the CLI's names, ranges and enum labels.

    Field *paths* come from DWARF, but the names people actually type
    (`gyro_overflow_detect`, not `checkOverflow`) exist only in this table.
    They are a permanent contract -- CLI-text backups and the presets repo are
    written against them -- so the manifest has to carry them, and today this
    is where they live.

    When settings.c goes, this is replaced by annotation records in .wf_meta
    and the manifest keeps the same shape.
    """
    table = elf.symbols.get('valueTable')
    count_symbol = elf.symbols.get('valueTableEntryCount')
    if table is None or count_symbol is None:
        return []

    layout, record_size = dwarf.struct_layout('clivalue_s')
    endian = '<' if elf.little_endian else '>'
    scalar = {1: 'B', 2: 'H', 4: 'I', 8: 'Q'}

    raw_count = elf.read_at(count_symbol, 2)
    count = struct.unpack(endian + 'H', raw_count)[0]

    settings = []
    for index in range(count):
        record = elf.read_at(table + index * record_size, record_size)
        if record is None or len(record) < record_size:
            break

        def field(member, record=record):
            offset, size = layout[member]
            return struct.unpack_from(endian + scalar[size], record, offset)[0]

        flags = field('type')
        kind, size = VALUE_TYPE.get(flags & VALUE_TYPE_MASK, ('?', 0))
        mode = VALUE_MODE.get((flags & VALUE_MODE_MASK) >> 6, '?')
        config_offset, config_size = layout['config']
        config = record[config_offset:config_offset + config_size]

        entry = {
            'name': elf.read_cstring(field('name')),
            'pgn': field('pgn'),
            'off': field('offset'),
            'kind': kind,
            'size': size,
            'mode': mode,
            'section': VALUE_SECTION.get((flags & VALUE_SECTION_MASK) >> 3, '?'),
        }

        if mode == 'lookup':
            table_index = config[0] | (config[1] << 8)
            if table_index < len(tables):
                entry['values'] = tables[table_index]
        elif mode == 'array':
            entry['count'] = config[0]
        elif mode == 'string':
            entry['min_length'] = config[0]
            entry['max_length'] = config[1]
        elif mode == 'bitset':
            entry['bit'] = config[0]
        elif kind == 'uint' and size == 4:
            entry['min'] = 0
            entry['max'] = struct.unpack_from(endian + 'I', config, 0)[0]
        elif kind == 'uint':
            entry['min'], entry['max'] = struct.unpack_from(endian + 'HH', config, 0)
        else:
            entry['min'], entry['max'] = struct.unpack_from(endian + 'hh', config, 0)

        settings.append(entry)
    return settings


def validate(manifest):
    """Check that every field actually fits inside the group that holds it.

    The registry and the DWARF type graph are two independent descriptions of
    the same memory, produced by different parts of the build. Nothing forces
    them to agree, and if they disagree the manifest hands the configurator
    offsets that read or write past the end of a parameter group. Catching that
    here costs nothing and is the difference between a build failure and a
    corrupted config on someone's aircraft.
    """
    problems = []

    def span_of(field):
        if field['kind'] == 'repeat':
            return field['stride'] * field['count']
        if field['kind'] == 'array':
            return field['elem_size'] * field['count']
        return field.get('size') or 0

    def check(fields, limit, prefix):
        for field in fields:
            end = field['off'] + span_of(field)
            if end > limit:
                problems.append('%s%s ends at %d, past the %d it lives in'
                                % (prefix, field['name'] or '[]', end, limit))
            if field['kind'] == 'repeat':
                # Nested offsets are relative to one element, not the array.
                check(field['fields'], field['stride'],
                      '%s%s[].' % (prefix, field['name']))

    for pg in manifest['pgs']:
        check(pg['fields'], pg['size'], '%s.' % (pg['symbol'] or pg['pgn']))

        # An array group should be exactly its elements, end to end.
        if pg['length'] > 1:
            covering = [f for f in pg['fields']
                        if f['kind'] in ('repeat', 'array') and f['off'] == 0]
            if covering and span_of(covering[0]) != pg['size']:
                problems.append('%s: %d elements of %d do not fill the %d the '
                                'registry reports'
                                % (pg['symbol'], covering[0]['count'],
                                   covering[0].get('stride') or covering[0].get('elem_size'),
                                   pg['size']))
    return problems


def canonical(manifest):
    """Stable bytes for hashing: sorted keys, no incidental whitespace."""
    return json.dumps(manifest, sort_keys=True, separators=(',', ':')).encode('utf-8')


def build_id(manifest):
    """The build ID is the hash of the described layout, and nothing else.

    Hashing the output rather than the inputs is what makes a candidate
    manifest checkable: there is no question of whether every input that
    affects struct layout was accounted for, because the manifest is the
    layout. If it hashes to what the board reports, it is the right one.

    The `build` block is deliberately excluded. It carries the build date and
    time, so hashing it would give every rebuild a new ID even when not one
    struct moved -- which would churn the configurator's manifest cache for no
    reason and mean a fresh manifest had to be published for every build.
    Two builds that describe the same layout *should* collide here: the
    manifest is then interchangeable between them, which is the point.
    """
    layout = {key: value for key, value in manifest.items() if key != 'build'}
    return hashlib.sha256(canonical(layout)).hexdigest()[:BUILD_ID_BYTES * 2]


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


BUILD_ID_HEADER_TEMPLATE = '''\
/*
 * Generated by src/utils/wf_manifest.py -- do not edit.
 *
 * Hash of the parameter manifest describing this build's configuration
 * layout. The firmware reports it over MSP2_WING_BUILD_ID so the configurator
 * can pick the manifest that matches, and refuse rather than guess if it
 * cannot find one. See docs/parameter-addressing-design.md.
 *
 * Layout only: it does not change when the build date does.
 */

#pragma once

#define WF_BUILD_ID "%s"
#define WF_BUILD_ID_BYTES %s
'''


def write_build_id_header(path, identifier):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    octets = ', '.join('0x%s' % identifier[i:i + 2]
                       for i in range(0, len(identifier), 2))
    body = BUILD_ID_HEADER_TEMPLATE % (identifier, octets)

    # Rewriting an unchanged header would rebuild everything that includes it.
    if os.path.exists(path):
        with open(path) as existing:
            if existing.read() == body:
                return False
    with open(path, 'w') as out:
        out.write(body)
    return True


def main(argv):
    parser = argparse.ArgumentParser(
        description='Extract a parameter manifest from a linked Wingflight ELF.')
    parser.add_argument('elf', help='linked firmware ELF (non-LTO, DEBUG=INFO)')
    parser.add_argument('manifest', help='manifest JSON to write')
    parser.add_argument('--msp-source', metavar='PATH',
                        help='msp.c preprocessed for this target; adds MSP codecs')
    parser.add_argument('--build-id-header', metavar='PATH',
                        help='also write a C header defining WF_BUILD_ID_BYTES')
    args = parser.parse_args(argv[1:])

    # The Windows SITL build is PE/COFF; everything else is ELF.
    import wf_pe
    elf = wf_pe.PeImage(args.elf) if wf_pe.is_pe(args.elf) else Elf(args.elf)

    # PG_REGISTER names the group's storage <name>_System / <name>_SystemArray,
    # so the registry's address field maps straight back to a DWARF variable.
    address_to_symbol = {addr: name for name, addr in elf.symbols.items()
                         if name.endswith('_System') or name.endswith('_SystemArray')}

    wanted = set(address_to_symbol.values())
    wanted.update(('pgRegistry_s', 'clivalue_s', 'lookupTableEntry_s',
                   'cliResourceValue_t', 'timerHardware_s', 'dmaoptEntry_s',
                   'serialPortIdentifiers'))
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
    manifest['settings'] = read_value_table(elf, dwarf, read_lookup_tables(elf, dwarf))
    manifest['resources'] = read_resource_table(elf, dwarf)
    manifest['timers'] = read_timer_hardware(elf, dwarf)
    manifest['dmaopts'] = read_dmaopt_table(elf, dwarf)
    manifest['cli'] = read_cli_tables(elf, dwarf)
    if args.msp_source:
        import wf_msp_codecs
        codecs, manual = wf_msp_codecs.extract(args.msp_source, manifest['pgs'],
                                               constants=dwarf.enum_constants)
        manifest['msp_codecs'] = {str(op): wf_msp_codecs.compact(codec) for op, codec in sorted(codecs.items())}
        manifest['msp_manual'] = {str(op): why for op, why in sorted(manual.items())}
        print('MSP codecs: %d mechanical, %d manual' % (len(codecs), len(manual)))
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

    problems = validate(manifest)
    if problems:
        sys.stderr.write('\nThe registry and the debug info disagree:\n')
        for problem in problems:
            sys.stderr.write('  %s\n' % problem)
        return 1

    largest = max(pg['size'] for pg in manifest['pgs'])
    print('largest group: %d bytes (%s)' % (
        largest, next(pg['symbol'] for pg in manifest['pgs'] if pg['size'] == largest)))

    # Written compact: with the MSP codecs unrolled, indentation alone
    # doubled the file, and the configurator caches it per build ID. The
    # build ID hashes canonical(), so the layout here does not affect it.
    with open(args.manifest, 'w') as out:
        json.dump(manifest, out, sort_keys=True, separators=(',', ':'))
        out.write('\n')

    if args.build_id_header:
        changed = write_build_id_header(args.build_id_header,
                                        manifest['build']['id'])
        print('%s %s' % ('wrote' if changed else 'unchanged',
                         args.build_id_header))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
