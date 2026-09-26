#!/usr/bin/env python3
"""
Tests for the MSP codec extractor (wf_msp_codecs.py).

A wrong codec writes the wrong configuration bytes, so the refusals matter
as much as the extractions: each shape the executor claims to understand is
checked, and so is each shape it must give up on.

    python src/utils/test_wf_msp_codecs.py
"""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wf_msp_codecs as w  # noqa: E402

# Parameter groups as the manifest describes them.
PGS = [
    {'pgn': 10, 'symbol': 'demoConfig_System', 'size': 12, 'length': 1, 'fields': [
        {'name': 'a', 'kind': 'uint', 'off': 0, 'size': 1},
        {'name': 'b', 'kind': 'int', 'off': 2, 'size': 2},
        {'name': 'c', 'kind': 'uint', 'off': 4, 'size': 4},
        {'name': 'arr', 'kind': 'array', 'off': 8, 'count': 3, 'elem_size': 1, 'elem_kind': 'uint'},
        {'name': 'inner.x', 'kind': 'uint', 'off': 11, 'size': 1},
    ]},
    {'pgn': 11, 'symbol': 'batteryConfig_System', 'size': 8, 'length': 1, 'fields': [
        {'name': 'batteryProfile', 'kind': 'uint', 'off': 0, 'size': 1},
        {'name': 'batteryCapacity', 'kind': 'array', 'off': 2, 'count': 3, 'elem_size': 2, 'elem_kind': 'uint'},
    ]},
    {'pgn': 30, 'symbol': 'wide_System', 'size': 8, 'length': 1, 'fields': [
        {'name': 'v', 'kind': 'uint', 'off': 0, 'size': 8},
    ]},
    {'pgn': 20, 'symbol': 'rows_SystemArray', 'size': 16, 'length': 4, 'fields': [
        {'name': '', 'kind': 'repeat', 'off': 0, 'stride': 4, 'count': 4, 'fields': [
            {'name': 'p', 'kind': 'uint', 'off': 0, 'size': 2},
            {'name': 'q', 'kind': 'int', 'off': 2, 'size': 1},
        ]},
    ]},
    {'pgn': 14, 'symbol': 'pidProfiles_SystemArray', 'size': 12, 'length': 2, 'fields': [
        {'name': '', 'kind': 'repeat', 'off': 0, 'stride': 6, 'count': 2, 'fields': [
            {'name': 'mode', 'kind': 'uint', 'off': 0, 'size': 1},
            {'name': 'pid', 'kind': 'repeat', 'off': 2, 'stride': 2, 'count': 2, 'fields': [
                {'name': 'P', 'kind': 'uint', 'off': 0, 'size': 1},
                {'name': 'I', 'kind': 'uint', 'off': 1, 'size': 1},
            ]},
        ]},
    ]},
]


# const tables in the image, as wf_manifest reads them
ARRAYS = {'meterIds': [10, 20, 30], 'flashTable': [7, 8]}


def extract(out_cases='', in_cases='', constants=None):
    src = ('static bool mspProcessOutCommand(int16_t cmdMSP, sbuf_t *dst)\n{\n  switch (cmdMSP) {\n'
           + out_cases + '\n    default:\n        return false;\n  }\n}\n'
           'static mspResult_e mspProcessInCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src)\n{\n'
           '  switch (cmdMSP) {\n' + in_cases + '\n    default:\n        return MSP_RESULT_ERROR;\n  }\n}\n')
    with tempfile.NamedTemporaryFile('w', suffix='.i', delete=False) as f:
        f.write(src)
    try:
        return w.extract(f.name, PGS, constants=constants, arrays=ARRAYS.get)
    finally:
        os.unlink(f.name)


def case(op, body):
    return '\n    case %d:\n%s\n        break;' % (op, body)


class Extraction(unittest.TestCase):

    def test_out_fields_consts_and_wire_widths(self):
        codecs, manual = extract(case(1, '''
        sbufWriteU8(dst, demoConfig()->a);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, demoConfig()->b);
        sbufWriteU8(dst, demoConfig()->c);'''))
        self.assertEqual(manual, {})
        ops = codecs[1]['ops']
        self.assertEqual(ops[0], {'kind': 'field', 'w': 1, 'size': 1, 'signed': False, 'pgn': 10, 'off': 0})
        self.assertEqual(ops[1], {'kind': 'const', 'w': 2, 'value': 0})
        self.assertEqual((ops[2]['off'], ops[2]['signed']), (2, True))
        self.assertEqual((ops[3]['w'], ops[3]['size']), (1, 4), 'a u32 field on the wire as one byte')

    def test_loop_unrolls_array_and_struct_paths(self):
        codecs, _ = extract(case(2, '''
        for (int i = 0; i < 3; i++) {
            sbufWriteU8(dst, demoConfig()->arr[i]);
        }
        sbufWriteU8(dst, demoConfig()->inner.x);'''))
        self.assertEqual([o['off'] for o in codecs[2]['ops']], [8, 9, 10, 11])

    def test_array_group_element_by_constant(self):
        codecs, _ = extract(case(3, '''
        for (int i = 0; i < 4; i++) {
            sbufWriteU16(dst, rows(i)->p);
        }'''))
        self.assertEqual([o['off'] for o in codecs[3]['ops']], [0, 4, 8, 12])

    def test_selected_profile(self):
        codecs, _ = extract(case(4, '''
        sbufWriteU8(dst, currentPidProfile->mode);
        for (int i = 0; i < 2; i++) {
            sbufWriteU8(dst, currentPidProfile->pid[i].I);
        }'''))
        ops = codecs[4]['ops']
        self.assertTrue(all(o['profile'] == 'pid' for o in ops))
        self.assertEqual([o['off'] for o in ops], [0, 3, 5], 'offsets within one profile element')

    def test_in_fields_and_optional_tail(self):
        codecs, _ = extract(in_cases=case(5, '''
        demoConfigMutable()->a = sbufReadU8(src);
        sbufReadU16(src);
        if (sbufBytesRemaining(src) >= 2) {
            demoConfigMutable()->b = sbufReadS16(src);
        }'''))
        ops = codecs[5]['ops']
        self.assertEqual(codecs[5]['dir'], 'in')
        self.assertEqual(ops[1], {'kind': 'skip', 'w': 2})
        self.assertTrue(ops[2]['optional'] and ops[2]['wire_signed'])
        self.assertNotIn('optional', ops[0])

    def test_indexed_setter_with_guard_and_side_effect(self):
        codecs, _ = extract(in_cases=case(6, '''
        i = sbufReadU8(src);
        if (i >= 4) {
            return MSP_RESULT_ERROR;
        }
        rowsMutable(i)->p = sbufReadU16(src);
        rowsMutable(i)->q = sbufReadU8(src);
        rowsReset(i);'''))
        c = codecs[6]
        self.assertEqual(c['index'], {'w': 1, 'max': 4})
        self.assertTrue(all(o['indexed'] for o in c['ops']))
        self.assertEqual([o['off'] for o in c['ops']], [0, 2])
        self.assertEqual(c['side_effects'], ['rowsReset'])

    def test_indexed_setter_with_alias_and_else_error(self):
        codecs, _ = extract(in_cases=case(7, '''
        i = sbufReadU8(src);
        if (i < 4) {
            rows_t *r = rowsMutable(i);
            r->p = sbufReadU16(src);
        } else {
            return MSP_RESULT_ERROR;
        }'''))
        self.assertEqual(codecs[7]['index']['max'], 4)
        self.assertEqual(codecs[7]['ops'][0]['off'], 0)

    def test_range_checked_local(self):
        codecs, _ = extract(in_cases=case(8, '''
        demoConfigMutable()->a = sbufReadU8(src);
        {
            uint8_t n = sbufReadU8(src);
            if (n < 2 || n > 6) {
                return MSP_RESULT_ERROR;
            }
            demoConfigMutable()->inner.x = n;
        }'''))
        op = codecs[8]['ops'][1]
        self.assertEqual((op['kind'], op['off'], op['check']), ('field', 11, {'min': 2, 'max': 6}))

    def test_length_guard(self):
        codecs, _ = extract(in_cases=case(9, '''
        if (dataSize != 3) {
            return MSP_RESULT_ERROR;
        }
        demoConfigMutable()->a = sbufReadU8(src);
        demoConfigMutable()->b = sbufReadU16(src);'''))
        self.assertEqual(codecs[9]['len'], 3)

    def test_enum_constants(self):
        codecs, manual = extract(case(10, '''
        for (int i = 0; i < ROW_COUNT; i++) {
            sbufWriteU8(dst, rows(i)->q);
        }'''), constants={'ROW_COUNT': 2})
        self.assertEqual(manual, {})
        self.assertEqual(len(codecs[10]['ops']), 2)

    def test_getter_alias_and_selected_element(self):
        codecs, manual = extract(case(13, 'sbufWriteU16(dst, getBatteryCapacity());'),
                                 in_cases=case(14, 'batteryConfigMutable()->batteryCapacity[batteryConfig()->batteryProfile] = sbufReadU16(src);'))
        self.assertEqual(manual, {})
        for op in (13, 14):
            x = codecs[op]['ops'][0]
            self.assertEqual((x['kind'], x['off'], x['stride'], x['count']), ('selected', 2, 2, 3))
            self.assertEqual(x['sel'], {'pgn': 11, 'off': 0, 'size': 1})

    def test_request_indexed_reply(self):
        codecs, manual = extract(case(15, '''
        {
            const int rem = sbufBytesRemaining(src);
            if (rem != 1) {
                return MSP_RESULT_ERROR;
            }
            const uint8_t i = sbufReadU8(src);
            if (i >= 4) {
                return MSP_RESULT_ERROR;
            }
            sbufWriteU16(dst, rows(i)->p);
        }'''))
        self.assertEqual(manual, {})
        c = codecs[15]
        self.assertEqual((c['dir'], c['index'], c['len']), ('out', {'w': 1, 'max': 4}, 1))
        self.assertTrue(c['ops'][0]['indexed'])

    def test_strings_out_and_in(self):
        codecs, manual = extract(case(16, '''
        {
            const int n = strlen(demoConfig()->arr);
            for (int i = 0; i < n; i++) {
                sbufWriteU8(dst, demoConfig()->arr[i]);
            }
        }'''), in_cases=case(17, '''
        memset(demoConfigMutable()->arr, 0, 3);
        for (unsigned int i = 0; i < __extension__ ({ __typeof__ (2U) _a = (2U); __typeof__ (dataSize) _b = (dataSize); _a < _b ? _a : _b; }); i++) {
            demoConfigMutable()->arr[i] = sbufReadU8(src);
        }'''))
        self.assertEqual(manual, {})
        self.assertEqual(codecs[16]['ops'], [{'kind': 'string', 'len': 3, 'pgn': 10, 'off': 8}])
        self.assertEqual(codecs[17]['ops'], [{'kind': 'string_in', 'len': 2, 'field_len': 3, 'pgn': 10, 'off': 8}])

    def test_u64_write(self):
        codecs, manual = extract(case(18, 'sbufWriteU64(dst, wide()->v);'))
        self.assertEqual(manual, {})
        self.assertEqual((codecs[18]['ops'][0]['w'], codecs[18]['ops'][0]['size']), (8, 8))

    def test_fall_through_labels_share_a_body(self):
        codecs, _ = extract('\n    case 11:\n    case 12:\n        sbufWriteU8(dst, demoConfig()->a);\n        break;')
        self.assertEqual(codecs[11]['ops'], codecs[12]['ops'])


class Refusals(unittest.TestCase):
    """Shapes the executor must not guess at."""

    def assertManual(self, codecs, manual, op, fragment):
        self.assertNotIn(op, codecs)
        self.assertIn(fragment, manual[op])

    def test_runtime_condition(self):
        self.assertManual(*extract(case(20, '''
        if (featureIsEnabled(1)) {
            sbufWriteU8(dst, demoConfig()->a);
        }''')), 20, 'condition')

    def test_computed_value(self):
        self.assertManual(*extract(case(21, 'sbufWriteU16(dst, demoConfig()->a * 10);')), 21, 'computed')

    def test_value_from_a_function(self):
        self.assertManual(*extract(case(22, 'sbufWriteU16(dst, getBatteryVoltage());')), 22, 'getBatteryVoltage')

    def test_call_consuming_the_buffer(self):
        self.assertManual(*extract(in_cases=case(23, 'featureConfigReplace(sbufReadU32(src));')), 23, 'consumes')

    def test_unknown_group(self):
        self.assertManual(*extract(case(24, 'sbufWriteU8(dst, nosuchConfig()->a);')), 24, 'nosuchConfig')

    def test_unknown_field(self):
        self.assertManual(*extract(case(25, 'sbufWriteU8(dst, demoConfig()->nope);')), 25, 'not in the manifest')

    def test_index_without_bound(self):
        self.assertManual(*extract(in_cases=case(26, '''
        i = sbufReadU8(src);
        rowsMutable(i)->p = sbufReadU16(src);''')), 26, 'bound')

    def test_out_body_reading_request_arguments(self):
        # A leading index is a request-indexed reply; a read after data is not.
        self.assertManual(*extract(case(28, '''
        sbufWriteU8(dst, demoConfig()->a);
        const uint8_t page = sbufReadU8(src);''')), 28, 'request arguments')

    def test_string_copy_without_clearing(self):
        self.assertManual(*extract(in_cases=case(29, '''
        for (unsigned int i = 0; i < MIN_(3U, dataSize); i++) {
            demoConfigMutable()->arr[i] = sbufReadU8(src);
        }''')), 29, 'clearing')

    def test_nested_switch_does_not_split_the_case(self):
        codecs, manual = extract(case(27, '''
        switch (demoConfig()->a) {
        case 1:
            sbufWriteU8(dst, 1);
            break;
        }'''))
        self.assertNotIn(1, codecs)
        self.assertIn(27, manual)

    def test_sizeof_in_a_length_guard(self):
        codecs, manual = extract(in_cases=case(40, '''
        if (dataSize != 2 * sizeof(uint16_t) + sizeof(uint8_t)) {
            return MSP_RESULT_ERROR;
        }
        demoConfigMutable()->b = sbufReadU16(src);
        demoConfigMutable()->c = sbufReadU16(src);
        demoConfigMutable()->a = sbufReadU8(src);'''))
        self.assertEqual(manual, {})
        self.assertEqual(codecs[40]['len'], 5)

    def test_sizeof_of_a_struct(self):
        self.assertManual(*extract(in_cases=case(41, '''
        if (dataSize != sizeof(demo_t)) {
            return MSP_RESULT_ERROR;
        }
        demoConfigMutable()->a = sbufReadU8(src);''')), 41, 'sizeof')

    def test_const_table_elements_are_constants(self):
        codecs, manual = extract(case(42, '''
        for (int i = 0; i < 2; i++) {
            sbufWriteU8(dst, meterIds[i]);
            sbufWriteU16(dst, rows(i)->p);
        }'''))
        self.assertEqual(manual, {})
        self.assertEqual(w.compact(codecs[42])['ops'], [
            ['c', 1, 10, ''], ['f', 2, 20, 0, 2, ''], ['c', 1, 20, ''], ['f', 2, 20, 4, 2, '']])

    def test_id_lookup_through_a_const_table(self):
        # MSP_SET_VOLTAGE_METER_CONFIG: the element is where the id sits in
        # the table; an unknown id is consumed and ignored.
        codecs, manual = extract(in_cases=case(43, '''
        uint8_t id = sbufReadU8(src);
        int index;
        for (index = 0; index < 3; index++) {
            if (id == meterIds[index])
                break;
        }
        if (index < 3) {
            rowsMutable(index)->p = sbufReadU16(src);
            rowsMutable(index)->q = sbufReadU8(src);
        } else {
            sbufReadU16(src);
            sbufReadU8(src);
        }'''))
        self.assertEqual(manual, {})
        self.assertEqual(codecs[43]['index'], {'w': 1, 'max': 3, 'map': [10, 20, 30], 'miss': 'ignore'})
        self.assertEqual(w.compact(codecs[43])['ops'], [['f', 2, 20, 0, 2, 'i'], ['f', 1, 20, 2, 1, 'is']])

    def test_index_miss_that_skips_another_width(self):
        self.assertManual(*extract(in_cases=case(44, '''
        uint8_t id = sbufReadU8(src);
        int index;
        for (index = 0; index < 3; index++) {
            if (id == meterIds[index])
                break;
        }
        if (index < 3) {
            rowsMutable(index)->p = sbufReadU16(src);
        } else {
            sbufReadU8(src);
        }''')), 44, 'different width')

    def test_index_miss_that_is_an_error(self):
        codecs, manual = extract(in_cases=case(45, '''
        i = sbufReadU8(src);
        if (i < 4) {
            rowsMutable(i)->p = sbufReadU16(src);
        } else {
            return MSP_RESULT_ERROR;
        }'''))
        self.assertEqual(manual, {})
        self.assertNotIn('miss', codecs[45]['index'])

    def test_index_miss_without_else_is_ignored(self):
        codecs, manual = extract(in_cases=case(46, '''
        i = sbufReadU8(src);
        if (i < 4) {
            rowsMutable(i)->p = sbufReadU16(src);
        }'''))
        self.assertEqual(manual, {})
        self.assertEqual(codecs[46]['index']['miss'], 'ignore')

    def test_non_const_table(self):
        # a table the image does not hold as const data is not a constant
        self.assertManual(*extract(case(47, 'sbufWriteU8(dst, runtimeTable[0]);')), 47, 'runtimeTable')

    def test_const_table_by_computed_index(self):
        self.assertManual(*extract(case(48, 'sbufWriteU8(dst, flashTable[demoConfig()->a]);')), 48,
                          'computed')


if __name__ == '__main__':
    unittest.main()
