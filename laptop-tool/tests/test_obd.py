"""Tests ohne Auto und ohne Adapter.

Aufruf im Ordner laptop-tool:  py -m unittest discover tests
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import obd_decode as dec                 # noqa: E402
from elm327 import ELM327, ElmError      # noqa: E402
from live import LiveReader              # noqa: E402
from simulator import VIN, SimulatedELM  # noqa: E402


class DecodeTest(unittest.TestCase):
    def test_obd_with_and_without_spaces(self):
        self.assertEqual(dec.parse_obd("41 0C 1A F8", "410C"), [0x1A, 0xF8])
        self.assertEqual(dec.parse_obd("410C1AF8", "410C"), [0x1A, 0xF8])
        self.assertEqual(dec.parse_obd("SEARCHING...\n410C1AF8", "410C"), [0x1A, 0xF8])
        self.assertIsNone(dec.parse_obd("NO DATA", "410C"))
        self.assertIsNone(dec.parse_obd("?", "410C"))
        self.assertIsNone(dec.parse_obd("", "410C"))

    def test_obd_several_ecus(self):
        r = "41 01 83 07 65 00\n41 01 00 04 00 00"
        self.assertEqual(dec.parse_obd_all(r, "4101"), [[0x83, 7, 0x65, 0], [0, 4, 0, 0]])
        self.assertEqual(dec.parse_obd(r, "4101"), [0x83, 7, 0x65, 0])

    def test_dtc_can_single_frame(self):
        self.assertEqual(dec.parse_dtcs("43 02 01 23 04 56"), ["P0123", "P0456"])
        self.assertEqual(dec.parse_dtcs("43 00"), [])

    def test_dtc_can_multi_frame(self):
        r = "00A\n0: 43 04 01 23 04 56\n1: 07 89 0A BC 00 00"
        self.assertEqual(dec.parse_dtcs(r), ["P0123", "P0456", "P0789", "P0ABC"])

    def test_dtc_multi_frame_padding_ignored(self):
        # Füllbytes 55 hinter der angegebenen Länge dürfen keinen Code ergeben
        r = "008\n0: 43 03 02 99 04 20\n1: 00 16 55 55 55 55 55"
        self.assertEqual(dec.parse_dtcs(r), ["P0299", "P0420", "P0016"])

    def test_dtc_two_ecus_and_duplicates(self):
        r = "43 00\n43 02 04 20 04 20"
        self.assertEqual(dec.parse_dtcs(r), ["P0420"])

    def test_one_byte_answer(self):
        # Antwort auf 04 (Fehler löschen) ist nur ein Byte und keine Längenzeile
        self.assertEqual(dec.messages("44"), ["44"])
        self.assertEqual(dec.messages("44\n44"), ["44", "44"])

    def test_dtc_kline(self):
        self.assertEqual(dec.parse_dtcs("43 01 23 04 56 00 00", is_can=False), ["P0123", "P0456"])

    def test_dtc_other_services_and_letters(self):
        self.assertEqual(dec.parse_dtcs("47 01 C1 00", service="47"), ["U0100"])
        self.assertEqual(dec.parse_dtcs("4A 01 41 23", service="4A"), ["C0123"])
        self.assertEqual(dec.parse_dtcs("43 01 92 34"), ["B1234"])

    def test_dtc_no_data_and_garbage(self):
        self.assertEqual(dec.parse_dtcs("NO DATA"), [])
        self.assertIsNone(dec.parse_dtcs("?"))
        self.assertIsNone(dec.parse_dtcs("CAN ERROR"))

    def test_uds(self):
        st, d = dec.parse_uds("62 11 BD 0C 6F", 0x11BD)
        self.assertEqual(st, dec.UDS_OK)
        self.assertAlmostEqual(((d[0] << 8) | d[1]) / 10.0 - 273.1, 45.2, places=1)
        self.assertEqual(dec.parse_uds("7F 22 31", 0x11BD), (dec.UDS_NEG, 0x31))
        self.assertEqual(dec.parse_uds("NO DATA", 0x11BD), (dec.UDS_NONE, None))

    def test_vin_can(self):
        r = ("014\n0: 49 02 01 57 56 57\n1: 5A 5A 5A 36 4A 5A 41\n"
             "2: 57 30 31 32 33 34 35")
        self.assertEqual(dec.parse_vin(r), "WVWZZZ6JZAW012345")

    def test_vin_kline(self):
        r = ("49 02 01 00 00 00 57\n49 02 02 56 57 5A 5A\n49 02 03 5A 36 4A 5A\n"
             "49 02 04 41 57 30 31\n49 02 05 32 33 34 35")
        self.assertEqual(dec.parse_vin(r), "WVWZZZ6JZAW012345")

    def test_supported(self):
        # Beispiel aus der OBD-Norm: BE 1F A8 13
        pids = dec.parse_supported([0xBE, 0x1F, 0xA8, 0x13], 0)
        self.assertEqual(sorted(pids), [0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0C, 0x0D, 0x0E,
                                        0x0F, 0x10, 0x11, 0x13, 0x15, 0x1C, 0x1F, 0x20])

    def test_voltage(self):
        self.assertEqual(dec.parse_voltage("12.4V"), 12.4)
        self.assertEqual(dec.parse_voltage("14,1V"), 14.1)
        self.assertIsNone(dec.parse_voltage("?"))

    def test_dtc_description(self):
        self.assertIn("Ladedruck", dec.dtc_description("P0299"))
        self.assertIn("herstellerspezifisch", dec.dtc_description("P1297"))
        self.assertIn("Zündung", dec.dtc_description("P0399"))
        self.assertIn("Netzwerk", dec.dtc_description("U0400"))


class SimulatorTest(unittest.TestCase):
    def setUp(self):
        self.elm = ELM327(SimulatedELM(speed=20))
        self.info = self.elm.init()

    def tearDown(self):
        self.elm.close()

    def test_init(self):
        self.assertEqual(self.info["version"], "ELM327 v1.5")
        self.assertEqual(self.info["protocol"], "6")
        self.assertTrue(self.info["ecu_ok"])
        self.assertAlmostEqual(self.info["voltage"], 14.2, delta=0.2)

    def test_dtcs_read_and_clear(self):
        r = self.elm.read_dtcs()
        self.assertTrue(r["mil"])
        self.assertEqual(r["stored"], ["P0299", "P0420", "P0016"])
        self.assertEqual(r["pending"], ["P0301"])
        self.assertEqual(r["permanent"], ["P0420"])
        self.assertTrue(self.elm.clear_dtcs())
        r = self.elm.read_dtcs()
        self.assertFalse(r["mil"])
        self.assertEqual(r["stored"], [])
        self.assertEqual(r["permanent"], ["P0420"])

    def test_vehicle(self):
        self.assertEqual(self.elm.read_vin(), VIN)
        pids = self.elm.read_supported()
        self.assertIn(0x0C, pids)
        self.assertIn(0x4C, pids)

    def test_live(self):
        lr = LiveReader(self.elm)
        v = lr.cycle()
        for key in ("rpm", "boost", "oil", "coolant", "iat", "cat", "timing", "batt", "speed",
                    "map", "baro"):
            self.assertIsNotNone(v[key], key)
        self.assertEqual(v["_oil_source"], "22 11BD")
        self.assertTrue(v["_ecu_ok"])
        self.assertTrue(500 < v["rpm"] < 6000)
        self.assertTrue(-0.8 < v["boost"] < 2.0)
        # Header steht für die Live-Werte auf 7E0, Fehlercodes stellen kurz auf 7DF und zurück
        self.assertTrue(self.elm.header_phys)
        self.elm.read_dtcs()
        self.assertTrue(self.elm.header_phys)

    def test_ignition_off_then_on(self):
        elm = ELM327(SimulatedELM(ignition=False))
        info = elm.init()
        self.assertFalse(info["ecu_ok"])
        self.assertIsNone(info["protocol"])
        lr = LiveReader(elm)
        v = lr.cycle()
        self.assertFalse(v["_ecu_ok"])
        self.assertIsNone(v["rpm"])
        self.assertIsNotNone(v["batt"])         # Spannung misst der Adapter selbst
        with self.assertRaises(ElmError):
            elm.read_dtcs()
        elm.port.ignition = True                # Zündung an
        lr.cycle()
        self.assertEqual(elm.protocol, "6")
        v = lr.cycle()
        self.assertTrue(v["_ecu_ok"])
        self.assertIsNotNone(v["oil"])
        self.assertTrue(elm.header_phys)

    def test_adapter_gone(self):
        lr = LiveReader(self.elm)
        lr.cycle()
        self.elm.port.read = lambda size=1: b""     # Adapter schweigt
        self.elm.port.write = lambda data: len(data)
        for it in lr._items:
            it.due = 0
        with self.assertRaises(ElmError):
            lr.cycle()          # nach 3 Befehlen ohne Antwort: Fehlermeldung


if __name__ == "__main__":
    unittest.main()
