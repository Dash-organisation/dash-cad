"""Export every design of a Fusion folder, not only what an assembly references.

Run from Fusion: Utilities > Scripts and Add-Ins > "+" > script from my computer > this folder.
1. In the Data Panel, open the folder to export (it becomes the active folder).
2. Run the script, answer the questions, pick an output folder on your computer.

For each design (.f3d in the cloud: parts and assemblies) of the folder and, optionally, its subfolders:
  <out>/<subfolder path>/<name>_v<version>.f3d    Fusion design
  <out>/<subfolder path>/<name>_v<version>.step   geometry only (optional)
Limitation: the API writes a plain .f3d, even for an assembly. An assembly whose components are
inserted from other files (external references) is then written WITHOUT their geometry: it only
reopens next to the cloud originals. Such designs are listed in the log under "Assemblies with
external references": export those by hand, File > Export > Fusion Archive (.f3z), from Fusion.
The parts themselves are exported on their own when they are in the folder, and the STEP of the
assembly contains all the geometry.
Drawings, PCBs and other non-design files are skipped. One failing file does not stop the run;
the list of failures is shown at the end and written to <out>/export_log.txt.
"""

import os
import re
import traceback

import adsk.core
import adsk.fusion


def safe(name):
    return re.sub(r'[\\/:*?"<>|]+', "_", name).strip() or "unnamed"


def walk(folder, rel, recursive):
    files = folder.dataFiles
    for i in range(files.count):
        yield rel, files.item(i)
    if recursive:
        subs = folder.dataFolders
        for i in range(subs.count):
            sub = subs.item(i)
            yield from walk(sub, os.path.join(rel, safe(sub.name)), recursive)


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        folder = app.data.activeFolder or app.data.activeProject.rootFolder
        yes = adsk.core.DialogResults.DialogYes
        cancel = adsk.core.DialogResults.DialogCancel
        q = adsk.core.MessageBoxButtonTypes.YesNoCancelButtonType

        answer = ui.messageBox(f'Export the designs of "{folder.name}".\n\nInclude subfolders?',
                               "Export folder", q)
        if answer == cancel:
            return
        recursive = answer == yes
        answer = ui.messageBox("Export the files?\n\nYes: export.\nNo: only list the assemblies "
                               "with external references (nothing exported).", "Export folder", q)
        if answer == cancel:
            return
        check_only = answer != yes
        with_step = False
        if not check_only:
            answer = ui.messageBox("Also export a STEP of each design?", "Export folder", q)
            if answer == cancel:
                return
            with_step = answer == yes

        dlg = ui.createFolderDialog()
        dlg.title = "Output folder (for export_log.txt only)" if check_only else "Output folder"
        if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        out = dlg.folder

        designs = [(rel, f) for rel, f in walk(folder, "", recursive) if f.fileExtension == "f3d"]
        if not designs:
            ui.messageBox(f'No design in "{folder.name}".')
            return

        progress = ui.createProgressDialog()
        progress.show("Export folder", "%v / %m  %p%", 0, len(designs), 0)
        done, failed, linked = [], [], []
        for n, (rel, f) in enumerate(designs):
            if progress.wasCancelled:
                break
            progress.message = f"{n + 1} / {len(designs)}: {f.name}"
            progress.progressValue = n
            adsk.doEvents()
            doc = None
            try:
                target = os.path.join(out, rel)
                base = os.path.join(target, f"{safe(f.name)}_v{f.versionNumber}")
                if not check_only:
                    os.makedirs(target, exist_ok=True)
                    old = base + ".f3z.f3d"  # name written by the first version of this script
                    if os.path.exists(old):
                        os.remove(old)
                doc = app.documents.open(f, True)
                design = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
                em = design.exportManager
                refs = [o for o in design.rootComponent.allOccurrences if o.isReferencedComponent]
                if refs:
                    linked.append(f"{os.path.join(rel, f.name)} ({len(refs)} referenced occurrences)")
                if check_only:
                    done.append(os.path.join(rel, f.name))
                    continue
                if not em.execute(em.createFusionArchiveExportOptions(base + ".f3d")):
                    raise RuntimeError("f3d export returned False")
                if with_step:
                    if not em.execute(em.createSTEPExportOptions(base + ".step", design.rootComponent)):
                        raise RuntimeError("STEP export returned False")
                done.append(os.path.join(rel, f.name))
            except Exception:
                failed.append(f"{os.path.join(rel, f.name)}: {traceback.format_exc().strip().splitlines()[-1]}")
            finally:
                if doc is not None:
                    doc.close(False)
                adsk.doEvents()
        progress.hide()

        with open(os.path.join(out, "check_log.txt" if check_only else "export_log.txt"), "w", encoding="utf-8") as log:
            log.write(f'Folder: {folder.name} (subfolders: {recursive}, STEP: {with_step}, '
                      f'check only: {check_only})\n\n')
            log.write(f"Exported ({len(done)}):\n" + "".join(f"  {d}\n" for d in done))
            log.write(f"\nFailed ({len(failed)}):\n" + "".join(f"  {d}\n" for d in failed))
            log.write(f"\nAssemblies with external references, export by hand as .f3z ({len(linked)}):\n"
                      + "".join(f"  {d}\n" for d in linked))
        msg = f"{len(done)} / {len(designs)} designs {'checked' if check_only else 'exported'}; log in\n{out}"
        if failed:
            msg += f"\n\n{len(failed)} failed (see the log):\n" + "\n".join(failed[:10])
        if linked:
            msg += (f"\n\n{len(linked)} assemblies reference other files: their .f3d lacks those parts. "
                    "Export them by hand (File > Export > .f3z):\n" + "\n".join(linked[:10]))
        ui.messageBox(msg, "Export folder")
    except Exception:
        ui.messageBox("Failed:\n" + traceback.format_exc())
