#!/usr/bin/env python3
"""
tests/test_adb_parsing.py
Unit tests for ADB devices output parsing and device selection logic.
"""

import os
import sys
import unittest
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.xr.adb_setup import parse_adb_devices_output, select_target_device

class TestAdbParsing(unittest.TestCase):

    def test_single_authorized_device(self):
        output = """List of devices attached
340YC10G7F09S1\tdevice
"""
        devices = parse_adb_devices_output(output)
        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0], ("340YC10G7F09S1", "device"))
        serial, state = select_target_device(devices)
        self.assertEqual(serial, "340YC10G7F09S1")
        self.assertEqual(state, "device")

    def test_unauthorized_device(self):
        output = """List of devices attached
192.168.1.100:5555\tunauthorized
"""
        devices = parse_adb_devices_output(output)
        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0], ("192.168.1.100:5555", "unauthorized"))
        serial, state = select_target_device(devices)
        self.assertEqual(serial, "192.168.1.100:5555")
        self.assertEqual(state, "unauthorized")

    def test_no_devices(self):
        output = """List of devices attached

"""
        devices = parse_adb_devices_output(output)
        self.assertEqual(len(devices), 0)
        serial, state = select_target_device(devices)
        self.assertIsNone(serial)
        self.assertEqual(state, "none")

    def test_multiple_devices(self):
        output = """List of devices attached
emulator-5554\toffline
340YC10G7F09S1\tdevice
1234567890ABCDEF\tunauthorized
"""
        devices = parse_adb_devices_output(output)
        self.assertEqual(len(devices), 3)
        # Should pick the first authorized 'device'
        serial, state = select_target_device(devices)
        self.assertEqual(serial, "340YC10G7F09S1")
        self.assertEqual(state, "device")

    def test_multiple_unauthorized_and_offline(self):
        output = """List of devices attached
emulator-5554\toffline
1234567890ABCDEF\tunauthorized
"""
        devices = parse_adb_devices_output(output)
        serial, state = select_target_device(devices)
        self.assertEqual(serial, "1234567890ABCDEF")
        self.assertEqual(state, "unauthorized")

if __name__ == "__main__":
    unittest.main()
