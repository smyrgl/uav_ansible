#!/usr/bin/env python3
"""Make one extlinux.conf boot entry carry FDT + OVERLAYS, and make it the DEFAULT.

usage: uav-extlinux-overlays.py CONF LABEL FDT DTBO[,DTBO...]
Prints changed=1 when the file was rewritten, changed=0 otherwise.

Why the entry L4T calls "primary": its kernel postinst hook
(/etc/kernel/postinst.d/xx-nvidia-update-extlinux -> nv-update-extlinux)
forces "DEFAULT primary" on every kernel package update, and copies every
other line through untouched. Overlays kept in another entry (jetson-io's
"JetsonIO") are silently dropped by the next kernel update; overlays in
"primary" survive it.
"""
import os
import re
import shutil
import sys

conf, label, fdt = sys.argv[1], sys.argv[2], sys.argv[3]
dtbos = [d for d in sys.argv[4].split(",") if d]

with open(conf) as f:
    lines = f.read().splitlines()

start = next((i for i, l in enumerate(lines)
              if re.fullmatch(rf"\s*LABEL\s+{re.escape(label)}\s*", l)), None)
if start is None:
    sys.exit(f"no 'LABEL {label}' entry in {conf}")
end = next((i for i in range(start + 1, len(lines))
            if re.match(r"\s*LABEL\s", lines[i])), len(lines))
entry = lines[start:end]
indent = next((re.match(r"\s*", l).group(0) for l in entry[1:] if l.strip()), "\t")


def find(key):
    return next((i for i, l in enumerate(entry) if re.match(rf"\s*{key}\s", l)), None)


# FDT: the base DTB the overlays are applied to.
fdt_line = f"{indent}FDT {fdt}"
i = find("FDT")
if i is None:
    j = find("LINUX")
    entry.insert(j + 1 if j is not None else 1, fdt_line)
elif entry[i].split() != fdt_line.split():
    entry[i] = fdt_line

# OVERLAYS: add ours, keep any others already listed.
i = find("OVERLAYS")
if i is None:
    entry.insert(find("FDT") + 1, f"{indent}OVERLAYS {','.join(dtbos)}")
else:
    have = [d for d in re.split(r"[,\s]+", entry[i].strip())[1:] if d]
    want = have + [d for d in dtbos if d not in have]
    if want != have:
        entry[i] = f"{indent}OVERLAYS {','.join(want)}"

new = lines[:start] + entry + lines[end:]

d = next((i for i, l in enumerate(new) if re.match(r"\s*DEFAULT\s", l)), None)
if d is None:
    new.insert(0, f"DEFAULT {label}")
elif new[d].split(None, 1)[1].strip() != label:
    new[d] = re.sub(r"(\s*DEFAULT\s+).*", rf"\g<1>{label}", new[d])

if new == lines:
    print("changed=0")
    sys.exit(0)

# Boot-critical: keep the previous file, write the new one durably, then
# rename over the old one so a power cut never leaves a truncated config.
shutil.copyfile(conf, conf + ".uav-prev")
tmp = conf + ".uav-tmp"
with open(tmp, "w") as f:
    f.write("\n".join(new) + "\n")
    f.flush()
    os.fsync(f.fileno())
os.chmod(tmp, 0o644)
os.replace(tmp, conf)
print("changed=1")
