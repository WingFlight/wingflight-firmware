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

"""Check a generated manifest against the CLI's own settings table.

While cli/settings.c still exists, the firmware carries two independent
descriptions of the same configuration: valueTable, written by hand, and the
manifest, derived from DWARF. Anywhere they disagree, one of them is wrong --
and since the whole plan is to delete the hand-written one and trust the
derived one, that disagreement has to be found before anything depends on it.

This is deliberately a free regression test: it costs one CI job, needs no
firmware change, and stops being useful only when valueTable is deleted, at
which point it is deleted with it.

    python3 src/utils/wf_manifest_check.py firmware.elf manifest.json

valueTable is read out of the ELF rather than parsed out of the C, because the
C is full of preprocessor conditionals and the linked table is the thing that
actually ships.
"""

import argparse
import json
import struct
import sys

import wf_manifest


# Settings where valueTable and the struct genuinely disagree, and the manifest
# is the one telling the truth. Reported as warnings rather than failures, so
# that a known defect does not block the gate -- but they are real defects, not
# noise, and the list should only ever get shorter.
#
# Anything not listed here is a hard failure. It is empty, and should stay that
# way: the three align_board_* settings that were here were a genuine bug
# (VAR_INT16 declared against int32_t fields) and have been fixed rather than
# tolerated.
KNOWN_DISAGREEMENTS = {
}


def expected_span(entry):
    """How many bytes the CLI believes this setting occupies."""
    base = entry.get('size')
    if not base:
        return None
    if entry['mode'] == 'array':
        return base * entry.get('count', 1)
    if entry['mode'] == 'string':
        return entry.get('max_length', 0)
    # BITSET addresses a bit inside a value of the declared type; DIRECT and
    # LOOKUP are the value itself.
    return base


def index_manifest(manifest):
    """pgn -> every byte offset the CLI could legitimately name, and its size.

    Offsets are absolute within the parameter group, because that is how the
    CLI addresses them: PG_ARRAY_ELEMENT_OFFSET() in pg.h is
    `index * sizeof(element) + offsetof(element, member)`, so a setting in
    element 3 of a profile array has an offset well past the end of element 0.

    Both kinds of repetition are therefore expanded elementwise:

      - a repeated struct (a profile array), so each element's fields get their
        own absolute offsets;
      - a plain array, because the CLI names its elements individually --
        pitch_rc_rate is rcRates[1], not the start of rcRates.

    An array also keeps an entry at its base for the whole span, which is how
    MODE_ARRAY and MODE_STRING settings address it.
    """
    index = {}
    for pg in manifest['pgs']:
        exact = {}      # offset -> {sizes}, for scalar fields
        ranges = []     # (start, end, elem_size), for arrays

        def add(fields, base):
            for field in fields:
                start = base + field['off']
                if field['kind'] == 'repeat':
                    for element in range(field['count']):
                        add(field['fields'], start + element * field['stride'])
                elif field['kind'] == 'array':
                    ranges.append((start,
                                   start + field['elem_size'] * field['count'],
                                   field['elem_size']))
                else:
                    exact.setdefault(start, set()).add(field.get('size') or 0)

        add(pg['fields'], 0)
        index[pg['pgn']] = {
            'symbol': pg['symbol'],
            'size': pg['size'],
            'elem_size': pg['elem_size'],
            'exact': exact,
            'ranges': ranges,
        }
    return index


def locate(group, offset, span, mode):
    """Can the CLI legitimately address `span` bytes at `offset`?

    A scalar has to match a field exactly. Anything inside an array is a
    different question: the CLI names individual elements (pitch_rc_rate is
    rcRates[1]) and whole rows of a multi-dimensional array
    (gyro_rpm_notch_source_roll is notch_source[0], 16 of the 48 elements), so
    what matters is that the request lands on an element boundary and stays
    inside the array.

    One offset can also have several valid readings at once, because unions
    overlay them: accelerometerConfig's accZero is a union of a raw[4] array
    and named roll/pitch/yaw fields, so offset 4 is legitimately both a 2-byte
    scalar and the start of an 8-byte array. Every interpretation is tried, and
    only if none of them fits is this a disagreement.
    """
    sizes = group['exact'].get(offset, set())
    if span is None or span in sizes:
        if sizes:
            return None
    # A string field is a char array; its declared maxlength may or may not
    # count the terminator.
    if mode == 'string' and span is not None and span + 1 in sizes:
        return None

    for start, end, elem_size in group['ranges']:
        if not (start <= offset < end):
            continue
        if (offset - start) % elem_size:
            continue
        if span is not None and offset + span > end:
            continue
        return None

    if sizes:
        return ('CLI says %d bytes, manifest says %s'
                % (span, '/'.join(str(s) for s in sorted(sizes))))
    if any(start <= offset < end for start, end, _ in group['ranges']):
        return ('%d bytes at offset %d does not fit the array it lands in'
                % (span, offset))
    return 'no field at offset %d in %s' % (offset, group['symbol'])


def compare_registries(manifest, shipping_elf, dwarf):
    """Check the shipping firmware's registry against the manifest's.

    The manifest is extracted from a non-LTO build, because LTO collapses the
    per-compilation-unit DWARF it needs. The firmware people actually flash is
    the LTO build. Nothing automatically guarantees those two describe the same
    configuration layout, and if they diverge the manifest is wrong for the
    binary it claims to describe -- offsets that read and write the wrong
    fields.

    What can be checked: every group's number, version, element count and size,
    which is all of .pg_registry that is not a link-time address. What cannot:
    field offsets inside a group, since the LTO build has no per-CU DWARF to
    read them from. Those are fixed by the ABI rather than by the optimiser, so
    equal group sizes across both builds is a sound proxy -- a struct whose
    layout changed would have to keep exactly the same total size to slip
    through, which a real divergence essentially never does.

    A release build carries no debug info at all, so the pgRegistry_t record
    layout is taken from the manifest build's DWARF. That is not a shortcut:
    the two builds share a target and therefore an ABI, and if they did not,
    the group sizes compared below would not line up either.
    """
    elf = wf_manifest.Elf(shipping_elf)
    groups = wf_manifest.read_registry(elf, dwarf)

    shipped = {g['pgn']: g for g in groups}
    described = {pg['pgn']: pg for pg in manifest['pgs']}

    problems = []
    for pgn in sorted(set(shipped) | set(described)):
        here, there = described.get(pgn), shipped.get(pgn)
        if there is None:
            problems.append('pgn %d is in the manifest but not in the firmware' % pgn)
        elif here is None:
            problems.append('pgn %d is in the firmware but not in the manifest' % pgn)
        else:
            for field in ('version', 'size', 'length'):
                if here[field] != there[field]:
                    problems.append('pgn %d: manifest %s=%s, firmware %s=%s'
                                    % (pgn, field, here[field], field, there[field]))
    return problems, len(groups)


def main(argv):
    parser = argparse.ArgumentParser(
        description="Check a manifest against the CLI's settings table.")
    parser.add_argument('elf', help='the same ELF the manifest was built from')
    parser.add_argument('manifest', help='manifest JSON')
    parser.add_argument('--shipping-elf', metavar='ELF',
                        help='also check the manifest against the LTO build '
                             'that actually ships')
    parser.add_argument('--verbose', action='store_true',
                        help='also list settings that matched')
    args = parser.parse_args(argv[1:])

    elf = wf_manifest.Elf(args.elf)
    dwarf = wf_manifest.Dwarf(
        elf, {'clivalue_s', 'lookupTableEntry_s', 'pgRegistry_s'})
    entries = wf_manifest.read_value_table(
        elf, dwarf, wf_manifest.read_lookup_tables(elf, dwarf))

    with open(args.manifest) as handle:
        manifest = json.load(handle)
    index = index_manifest(manifest)

    missing_group = []
    mismatched = []
    known = []
    matched = 0

    for entry in entries:
        group = index.get(entry['pgn'])
        if group is None:
            missing_group.append(entry)
            continue

        span = expected_span(entry)
        mode = entry['mode']
        reason = locate(group, entry['off'], span, mode)

        if reason and entry['name'] in KNOWN_DISAGREEMENTS:
            known.append((entry, reason))
        elif reason:
            mismatched.append((entry, mode, span, reason))
        else:
            matched += 1
            if args.verbose:
                print('ok    %-40s pgn=%-5d off=%-5d %s'
                      % (entry['name'], entry['pgn'], entry['off'], mode))

    print('valueTable entries: %d   matched against manifest: %d   known bad: %d'
          % (len(entries), matched, len(known)))

    for entry, reason in known:
        print('  known: %-34s %s' % (entry['name'], KNOWN_DISAGREEMENTS[entry['name']]))

    for entry in missing_group:
        sys.stderr.write('  %-40s pgn %d is not in the manifest\n'
                         % (entry['name'], entry['pgn']))
    for entry, mode, span, reason in mismatched:
        sys.stderr.write('  %-40s pgn=%-5d off=%-5d %-7s %s\n'
                         % (entry['name'], entry['pgn'], entry['off'], mode, reason))

    problems = len(missing_group) + len(mismatched)
    if problems:
        sys.stderr.write('\n%d of %d settings do not agree with the manifest.\n'
                         % (problems, len(entries)))
        return 1

    print('the manifest agrees with valueTable on every setting')

    if args.shipping_elf:
        divergence, count = compare_registries(manifest, args.shipping_elf, dwarf)
        if divergence:
            sys.stderr.write('\nThe manifest does not describe the shipping build:\n')
            for problem in divergence:
                sys.stderr.write('  %s\n' % problem)
            return 1
        print('and describes the shipping build: %d groups agree' % count)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
