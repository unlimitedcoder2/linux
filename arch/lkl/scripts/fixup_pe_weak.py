#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0

import os
import re
import subprocess
import sys
import tempfile

WEAK_MARK = ".weak."


def main():
    if len(sys.argv) != 4:
        sys.stderr.write("usage: fixup_pe_weak.py <objfile> <objdump> <objcopy>\n")
        return 2
    obj, objdump, objcopy = sys.argv[1], sys.argv[2], sys.argv[3]

    syms = subprocess.run(
        [objdump, "-t", obj], capture_output=True, text=True, check=True
    ).stdout

    defined = set()
    undefined = set()
    defaults = {}
    line_re = re.compile(r"\(sec\s+(-?\d+)\).*?0x[0-9a-fA-F]+ (\S+)\s*$")
    for ln in syms.splitlines():
        m = line_re.search(ln)
        if not m:
            continue
        sec = int(m.group(1))
        name = m.group(2)
        (undefined if sec == 0 else defined).add(name)
        if sec != 0 and WEAK_MARK in name:
            i = name.index(WEAK_MARK)
            prefix = name[:i]
            rest = name[i + len(WEAK_MARK) :]
            base = rest.split(".", 1)[0]
            target = prefix + base
            defaults.setdefault(target, name)

    redef = []
    for target, dflt in defaults.items():
        if target in undefined and target not in defined:
            redef.append((dflt, target))

    if not redef:
        return 0

    with (
        tempfile.NamedTemporaryFile("w", suffix=".redef", delete=False) as rf,
        tempfile.NamedTemporaryFile("w", suffix=".glob", delete=False) as gf,
    ):
        for dflt, target in redef:
            rf.write("%s %s\n" % (dflt, target))
            gf.write("%s\n" % target)
        redef_path, glob_path = rf.name, gf.name

    try:
        out = obj + ".weakfix"
        subprocess.run(
            [
                objcopy,
                "--redefine-syms=" + redef_path,
                "--globalize-symbols=" + glob_path,
                obj,
                out,
            ],
            check=True,
        )
        os.replace(out, obj)
    finally:
        os.unlink(redef_path)
        os.unlink(glob_path)

    sys.stderr.write("  WEAKFIX %s (%d weak externals bound)\n" % (obj, len(redef)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
