import io

import unittest

from unittest.mock import patch

import review


class ReviewTests(unittest.TestCase):
    def test_session_groups_auto_to_teleop_and_marks_interruption(self):
        rows = [(1, review.ENABLED, 'boolean', True),
                (16, review.ENABLED, 'boolean', False),
                (18, review.ENABLED, 'boolean', True),
                (40, review.AUTO, 'boolean', False)]
        self.assertEqual(review.sessions(rows), [(1, 40, True)])

    def test_brownout_has_exact_source_and_window(self):
        rows = [(10, review.ENABLED, 'boolean', True),
                (12.25, review.BROWN, 'boolean', True),
                (13, review.BROWN, 'boolean', False),
                (20, review.ENABLED, 'boolean', False)]
        result = review.analyze(rows, 10, 20, False, 'test', 'test.wpilog')
        incident = result['incidents'][0]
        self.assertEqual(incident['raw_time_s'], 12.25)
        self.assertEqual(incident['window_s'], [10.25, 15.25])
        self.assertEqual(incident['signals'], [review.BROWN])

    def test_ai_fallback_without_credentials(self):
        with patch.dict('os.environ', {}, clear=True):
            self.assertIsNone(review.ai_summary({}))

    def test_shooter_gap_and_slow_loop_are_flagged(self):
        rows = [(10, review.ENABLED, 'boolean', True),
                (10, review.TARGET, 'double', 2000),
                (10, review.ACTUAL, 'double', 900),
                (11, review.CYCLE, 'double', 55),
                (13, review.ACTUAL, 'double', 1950),
                (15, review.ENABLED, 'boolean', False)]
        result = review.analyze(rows, 10, 15, False, 'test', 'test.wpilog')
        shooter = next(i for i in result['incidents'] if i['subsystem'] == 'Shooter')
        self.assertEqual(shooter['signals'], [review.TARGET, review.ACTUAL])
        self.assertEqual(shooter['raw_time_s'], 10)
        self.assertTrue(any(i['subsystem'] == 'Robot loop' for i in result['incidents']))

    def test_short_shooter_gap_is_not_flagged(self):
        rows = [(10, review.ENABLED, 'boolean', True),
                (10, review.TARGET, 'double', 2000),
                (10, review.ACTUAL, 'double', 900),
                (11, review.ACTUAL, 'double', 1950)]
        result = review.analyze(rows, 10, 11, True, 'test', 'test.wpilog')
        self.assertFalse(any(i['subsystem'] == 'Shooter' for i in result['incidents']))

    def test_malformed_log_rejected(self):
        with patch('builtins.open', return_value=io.BytesIO(b'WPILOG')):
            with self.assertRaises(ValueError):
                review.load('bad.wpilog')

    def test_html_escapes_log_names(self):
        data = review.analyze([(1, review.ENABLED, 'boolean', True)], 1, 1, True,
                              '<script>', '<log>')
        with patch.dict('os.environ', {}, clear=True):
            result = review.render(data)
        self.assertNotIn('<script>', result)
        self.assertIn('&lt;script&gt;', result)
        self.assertIn('Automated recap', result)

if __name__ == '__main__':
    unittest.main()

