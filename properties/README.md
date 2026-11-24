# Mass properties

| File | Content |
| --- | --- |
| `inerties_masses.pdf` | Fusion 360 *Properties* of each part, saved 2026-10-14 for the **v2** design (Fusion v34). Source document, never edit. |
| `mass_properties.json` | the PDF values, extracted by hand: one entry per part, in the PDF's units, with page references and the PDF's SHA-256 |
| `body_mapping_v2.json` | v2 export bodies → PDF parts (all `pdf` method) |
| `body_mapping_xshape.json` | xshape export bodies → PDF parts: 9 unchanged parts `pdf`, 6 redesigned parts `scale_cad` |

Totals: PDF 13.594 kg (v2) + 1 kg torso allowance = 14.594 kg; xshape 14.537 kg. The robot weighs
15.14 kg: the remaining 0.6 kg is not located (cables, connectors, screws). The PDF masses **include
the motors and the battery** in the hip and torso parts (confirmed 2026-10-14): never add them again.

## `mass_properties.json` format

```jsonc
{
  "source": "inerties_masses.pdf",
  "sha256": "<sha256 of the PDF>",            // apply_mass_properties.py refuses a changed PDF
  "units": {"mass": "g", "com": "mm", "inertia": "g mm^2"},
  "parts": {
    "Achilles_left": {
      "mass_g": 57.13,
      "com_mm": [-253.588, -9.839, 4.582],      // in the part frame = export body frame
      "inertia_com_g_mm2":    {"Ixx": ..., "Iyy": ..., "Izz": ..., "Ixy": ..., "Ixz": ..., "Iyz": ...},
      "inertia_origin_g_mm2": {"Ixx": ..., ...}, // used to cross-check the COM tensor (parallel axis)
      "bbox_mm": [530.0, 22.5, 26.0],
      "pdf_pages": [1, 2]
    }
  }
}
```

Fusion prints the off-diagonal products with a sign convention: they are entered **as printed**
(signed matrix entries), and the parallel-axis check of `apply_mass_properties.py` fails if the
sign or a digit is wrong. Fusion rounds some values to 4 significant digits; the check tolerates 0.1 %.

## Known caveats

- The PDF was made for v2. For xshape, the thigh, foot and achilles changed: their values are
  `scale_cad` estimates. **Weigh these parts and redo the PDF for the current design.**
- One PDF entry `Foot` is used for both feet.
- The PDF tibia masses differ by 3.8 g left/right; kept as is, not forced symmetric.
- Right cam COM/tensor look like a 5° rotation of the mirrored left cam. Not corrected.
- An earlier check script assumed the PDF foot frame differs from the MJCF foot frame; the mapping
  uses the export frame (confirmed for v2). If you redo the PDF, check the foot frame explicitly.
