#!/usr/bin/env python3
"""Check that each gripper's URDF link masses add up to its datasheet weight.

Reads config/datasheet_masses.yaml, sums every <inertial><mass> in the URDF
(or xacro macro with literal masses) each entry names and exits 1 if any sum
is outside the tolerance.

Usage: python3 onrobot_gripper_description/scripts/check_datasheet_masses.py
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / 'onrobot_gripper_description' / 'config' / 'datasheet_masses.yaml'


def link_mass_sum(path):
    masses = ET.parse(path).getroot().iterfind('.//link/inertial/mass')
    return sum(float(m.get('value')) for m in masses)


def main():
    config = yaml.safe_load(CONFIG.read_text())
    tolerance = config['tolerance_pct']
    failed = []
    for name, entry in config['grippers'].items():
        total = link_mass_sum(REPO / entry['urdf'])
        diff_pct = 100.0 * (total - entry['mass_kg']) / entry['mass_kg']
        ok = abs(diff_pct) <= tolerance
        print(f"{'ok  ' if ok else 'FAIL'} {name}: URDF {total:.3f} kg, "
              f"datasheet {entry['mass_kg']} kg ({diff_pct:+.1f}%)")
        if not ok:
            failed.append(name)
    if failed:
        print(f"{len(failed)} gripper(s) differ from the datasheet by more than "
              f"{tolerance}%: {', '.join(failed)}")
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
