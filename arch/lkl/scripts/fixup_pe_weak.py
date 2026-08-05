#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0

import os
import re
import subprocess
import sys
import tempfile

WEAK_MARK = ".weak."
WEAK_SCL_RE = re.compile(r"\(scl\s+105\)")


def weak_externals(objdump, path):
    syms = subprocess.run(
        [objdump, "-t", path], capture_output=True, text=True, check=True
    ).stdout
    return {ln.split()[-1] for ln in syms.splitlines() if WEAK_SCL_RE.search(ln)}


def find_sym(defined, base):
    if base in defined:
        return base
    cands = sorted(n for n in defined if n.endswith(base))
    return cands[0] if len(cands) == 1 else None


def main():
    if len(sys.argv) not in (4, 5):
        sys.stderr.write(
            "usage: fixup_pe_weak.py <objfile> <objdump> <objcopy> [sys_ni.o]\n"
        )
        return 2
    obj, objdump, objcopy = sys.argv[1], sys.argv[2], sys.argv[3]
    ni_obj = sys.argv[4] if len(sys.argv) == 5 else None

    syms = subprocess.run(
        [objdump, "-t", obj], capture_output=True, text=True, check=True
    ).stdout

    defined = set()
    undefined = set()
    defaults = {}
    where = {}
    sec_names = {}
    line_re = re.compile(r"\(sec\s+(-?\d+)\).*?0x([0-9a-fA-F]+) (\S+)\s*$")
    for ln in syms.splitlines():
        m = line_re.search(ln)
        if not m:
            continue
        sec = int(m.group(1))
        name = m.group(3)
        (undefined if sec == 0 else defined).add(name)
        if sec != 0:
            where.setdefault(name, (sec, int(m.group(2), 16)))
            if name.startswith("."):
                sec_names.setdefault(sec, name)
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

    add = []
    if ni_obj:
        stubs = sorted(
            s
            for s in weak_externals(objdump, ni_obj)
            if s in undefined and s not in defined
        )
        if stubs:
            ni_syscall = find_sym(defined, "sys_ni_syscall")
            sec, value = where.get(ni_syscall, (None, None))
            if ni_syscall is None or sec not in sec_names:
                sys.stderr.write(
                    "fixup_pe_weak: cannot locate sys_ni_syscall in %s, %d "
                    "cond_syscall stubs left unbound\n" % (obj, len(stubs))
                )
                return 1
            add = [
                "--add-symbol=%s=%s:0x%x,global,function"
                % (s, sec_names[sec], value)
                for s in stubs
            ]

    if not redef and not add:
        return 0

    tmps = []
    if redef:
        with (
            tempfile.NamedTemporaryFile("w", suffix=".redef", delete=False) as rf,
            tempfile.NamedTemporaryFile("w", suffix=".glob", delete=False) as gf,
        ):
            for dflt, target in redef:
                rf.write("%s %s\n" % (dflt, target))
                gf.write("%s\n" % target)
            tmps = [rf.name, gf.name]

    opts = add[:]
    if tmps:
        opts = ["--redefine-syms=" + tmps[0], "--globalize-symbols=" + tmps[1]] + opts

    try:
        out = obj + ".weakfix"
        subprocess.run([objcopy, *opts, obj, out], check=True)
        os.replace(out, obj)
    finally:
        for t in tmps:
            os.unlink(t)

    sys.stderr.write(
        "  WEAKFIX %s (%d weak externals bound, %d cond_syscall stubs)\n"
        % (obj, len(redef), len(add))
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
