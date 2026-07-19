#!/usr/bin/env python3
import unittest

from decode_route_log import decode_record, extract_dc_payload


KNOWN_HEX = (
    "0102030118FDBFFCFC0388FF5B0062014D0004000000000000"
    "B40088FFBFFC980302FEBDFCD403EEFD88FF4B"
)


class RouteLogDecoderTests(unittest.TestCase):
    def test_decodes_known_record(self) -> None:
        record = decode_record(KNOWN_HEX)

        self.assertEqual(record["version"], 1)
        self.assertEqual(record["run"], 2)
        self.assertEqual(record["direction"], "R")
        self.assertTrue(record["gray_valid"])
        self.assertEqual(record["status"], 1)
        self.assertEqual(record["gray_mask"], "18")
        self.assertEqual(record["correction_half_cm"], -3)
        self.assertEqual(record["turn_in_mrad"], -833)
        self.assertEqual(record["straight_mm"], 1020)
        self.assertEqual(record["turn_out_mrad"], -120)
        self.assertEqual(record["phase_ticks"], [91, 354, 77])
        self.assertEqual(record["event_count"], 4)
        self.assertEqual(
            record["events"][3],
            {"x_mm": 980, "y_mm": -530, "yaw_mrad": -120},
        )

    def test_rejects_bad_crc(self) -> None:
        with self.assertRaisesRegex(ValueError, "CRC"):
            decode_record(KNOWN_HEX[:-2] + "00")

    def test_rejects_wrong_length_and_non_hex(self) -> None:
        with self.assertRaisesRegex(ValueError, "88"):
            decode_record(KNOWN_HEX[:-2])
        with self.assertRaisesRegex(ValueError, "hex"):
            decode_record("Z" * 88)

    def test_extracts_payload_from_timestamped_serial_line(self) -> None:
        line = f"[2026-07-19 21:30:00] DC,{KNOWN_HEX}\r\n"
        self.assertEqual(extract_dc_payload(line), KNOWN_HEX)
        self.assertIsNone(extract_dc_payload("RR,8,1,0"))


if __name__ == "__main__":
    unittest.main()
