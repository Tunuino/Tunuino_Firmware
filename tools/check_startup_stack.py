"""Guard the nested flash-restore stack path before USB enumeration.

GCC -fstack-usage supplies each frame size. This is a focused regression guard,
not a whole-program stack proof. Reserve 2 KiB for unlisted callees and context.
"""
from pathlib import Path
import re
import sys


def check(build_dir, makefile):
    frames = {}
    for report in Path(build_dir).rglob("*.su"):
        for line in report.read_text().splitlines():
            location, size, kind = line.split("\t")
            name = location.rsplit(":", 1)[-1].split(".", 1)[0]
            frames[name] = max(frames.get(name, 0), int(size))
            if name in {"main", "board_io_start", "tunuino_journal_init",
                        "valid", "tunuino_device_validate",
                        "tunuino_device_validation_errors"}:
                if kind != "static":
                    raise ValueError(f"Unbounded or dynamic startup frame: {name}")
    match = re.search(r"^USE_PROCESS_STACKSIZE\s*=\s*(\S+)",
                      Path(makefile).read_text(), re.M)
    stack = int(match[1], 0)
    paths = {
        "flash scan": ["main", "board_io_start", "tunuino_journal_init", "valid",
                       "tunuino_device_validate", "tunuino_device_validation_errors"],
        "restore validation": ["main", "board_io_start", "tunuino_device_restore",
                               "tunuino_device_validate", "tunuino_device_validation_errors"],
        "restore publish": ["main", "board_io_start", "tunuino_device_restore", "publish"],
    }
    for label, path in paths.items():
        missing = set(path) - frames.keys()
        if missing:
            raise ValueError(f"Missing stack reports for {sorted(missing)}")
        used = sum(frames[name] for name in path)
        print(f"Startup {label}: {used} bytes + 2048 reserve / {stack} bytes")
        if used + 2048 > stack:
            raise ValueError(f"Startup {label} exceeds reserved stack budget")


if __name__ == "__main__":
    check(*sys.argv[1:])
