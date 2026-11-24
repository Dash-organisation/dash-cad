"""Regression tests: the pipeline must reproduce the committed models.

    python -m pytest pipeline/tests -q        (about 1 minute; repose test is the slow one)
"""
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "pipeline"))
import apply_mass_properties as amp  # noqa: E402
import close_loops  # noqa: E402

MODELS = REPO / "models"


def inertials(path_or_text):
    root = ET.fromstring(path_or_text) if isinstance(path_or_text, str) else ET.parse(path_or_text).getroot()
    out = {}
    for b in root.iter("body"):
        i = b.find("inertial")
        out[b.get("name")] = np.r_[float(i.get("mass")), np.fromstring(i.get("pos"), sep=" "),
                                   np.fromstring(i.get("fullinertia"), sep=" ")]
    return out


@pytest.mark.parametrize("raw", ["xshape-standing-f8/EXPORT_assembly-v8.xml",
                                 "v2-init-f7/EXPORT_assembly_homing-v7.xml"])
def test_close_loops_on_raw_exports(raw):
    assert close_loops.main([str(MODELS / raw), "--dry-run"]) == 0


@pytest.mark.parametrize("closed, mapping, total", [
    ("xshape-standing-f8/EXPORT_assembly-v8_closed.xml", "body_mapping_xshape.json", 14.537322),
    ("v2-standing-f34/EXPORT_assembly-v34_closed.xml", "body_mapping_v2.json", 14.594154),
])
def test_mass_properties_reproduce_committed_model(closed, mapping, total):
    src = MODELS / closed
    xml, audit = amp.build(src, REPO / "properties/mass_properties.json", REPO / "properties" / mapping)
    assert audit["total_mass_kg"] == pytest.approx(total, abs=1e-6)
    new = inertials(xml)
    ref = inertials(src.with_name(src.stem + "_mass.xml"))
    for name, v in ref.items():
        np.testing.assert_allclose(new[name], v, rtol=1e-9, atol=1e-15)


def test_repose_reproduces_init_model(tmp_path):
    out = MODELS / "xshape-init-f8" / ".test_repose.xml"  # next to meshes/ so it compiles
    try:
        subprocess.run([sys.executable, str(REPO / "pipeline/repose_init.py"),
                        str(MODELS / "v2-init-f7/dash01_homing_v7.xml"),
                        str(MODELS / "xshape-standing-f8/EXPORT_assembly-v8_closed.xml"),
                        "-o", str(out)], check=True, capture_output=True)
        a = {b.get("name"): b for b in ET.parse(out).getroot().iter("body")}
        b = {x.get("name"): x for x in ET.parse(MODELS / "xshape-init-f8/dash01_homing_v8.xml").getroot().iter("body")}
        assert set(a) == set(b)
        for k in a:
            for f in ("pos", "euler"):
                np.testing.assert_allclose(np.fromstring(a[k].get(f), sep=" "),
                                           np.fromstring(b[k].get(f), sep=" "), atol=1e-10)
    finally:
        out.unlink(missing_ok=True)


def test_static_stand_runs():
    r = subprocess.run([sys.executable, str(REPO / "pipeline/checks/static_stand.py"),
                        str(MODELS / "xshape-standing-f8/EXPORT_assembly-v8_closed_mass.xml")],
                       check=True, capture_output=True, text=True)
    assert "mass 14.537 kg" in r.stdout and "CoM inside support polygon: True" in r.stdout
