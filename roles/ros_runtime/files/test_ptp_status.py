import unittest
from ptp_status import parse_pmc, check_mapping

TEXT = '''currentUtcOffset 37
ptpTimescale 1
currentUtcOffsetValid 0
portState MASTER
'''

class MappingTests(unittest.TestCase):
    def test_static_master_offset_is_independently_verified(self):
        state = parse_pmc(TEXT)
        self.assertFalse(state['utc_offset_valid'])
        self.assertTrue(check_mapping(state,37_000_000_010,37_000_000_005,1_000_000)[0])
    def test_utc_phc_misconfiguration_fails(self):
        self.assertFalse(check_mapping(parse_pmc(TEXT),0,37_000_000_000,1_000_000)[0])
    def test_stale_or_inconsistent_leap_offset_fails(self):
        self.assertFalse(check_mapping(parse_pmc(TEXT),37_000_000_000,38_000_000_000,1_000_000)[0])
    def test_missing_management_fields_fail(self):
        with self.assertRaises(ValueError): parse_pmc('currentUtcOffset 37')
    def test_wrong_clock_role_fails(self):
        state=parse_pmc(TEXT.replace('MASTER','SLAVE'))
        self.assertFalse(check_mapping(state,37_000_000_000,37_000_000_000,1_000_000)[0])
    def test_unknown_offset_fails(self):
        with self.assertRaises(ValueError): parse_pmc(TEXT.replace('37','0'))

if __name__ == '__main__': unittest.main()
