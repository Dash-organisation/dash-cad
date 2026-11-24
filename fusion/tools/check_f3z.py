"""Check that Fusion archives (.f3z) contain every design they reference.

    python fusion/tools/check_f3z.py fusion/*/*.f3z

A .f3z is a zip of .f3d files plus DesignDescription.json (the reference graph). For each archive:
root design, number of designs, designs listed but missing from the zip, references that point to
no embedded design. "missing files=[]" and "unresolved=0" mean the archive opens on its own.
A plain .f3d of an assembly with external references has no such index and no geometry of the
linked parts: export it as .f3z (see fusion/README.md).
"""
import json, sys, zipfile
for path in sys.argv[1:]:
    z = zipfile.ZipFile(path); names = set(z.namelist())
    g = json.loads(z.read("DesignDescription.json"))["designDescription"]["designGraphs"][0]
    objs = {o["id"]: o for o in g["designObjects"]}
    root = [objs[i]["displayName"] for i in g["rootIds"]]
    missing_file = [o["displayName"] for o in objs.values() if o["relativePath"] not in names]
    refs = [r for o in objs.values() for r in o.get("references", [])]
    ref_ids = [i for r in refs for i in r["ids"]]
    dangling = [r for r in ref_ids if r not in objs]
    print(f"{path}: root={root}, {len(objs)} designs, missing files={missing_file}, "
          f"references={len(refs)}, unresolved={len(dangling)}")
    print("   ", ", ".join(sorted(f"{o['displayName']} v{o['version']}" for o in objs.values())))
