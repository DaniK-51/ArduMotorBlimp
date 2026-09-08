#!/usr/bin/env python3
"""Build and run the MotorBlimp controller unit tests against ArduPilot SITL."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys


def parse_args() -> argparse.Namespace:
    repository = Path(__file__).resolve().parents[1]
    default_ardupilot = repository.parent / "ardupilot-4.7"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ardupilot",
        type=Path,
        default=Path(os.environ.get("ARDUPILOT_DIR", default_ardupilot)),
        help="ArduPilot 4.7 checkout (default: sibling work/ardupilot-4.7)",
    )
    parser.add_argument(
        "--configure",
        action="store_true",
        help="run a SITL debug configure before building the test",
    )
    return parser.parse_args()


def python_for_ardupilot(repository: Path) -> Path:
    override = os.environ.get("ARDUPILOT_PYTHON")
    candidates = [
        Path(override) if override else None,
        repository.parent / "ardupilot-venv" / "bin" / "python",
        Path(sys.executable),
    ]
    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            # Do not resolve a venv's python symlink: invoking the real system
            # interpreter path bypasses pyvenv.cfg and loses installed modules.
            return candidate.expanduser().absolute()
    raise RuntimeError("No Python interpreter is available for ArduPilot waf")


def run(command: list[str], cwd: Path, environment: dict[str, str]) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=cwd, env=environment, check=True)


def main() -> int:
    args = parse_args()
    source_repository = Path(__file__).resolve().parents[1]
    ardupilot = args.ardupilot.expanduser().resolve()
    waf = ardupilot / "waf"
    if not waf.is_file():
        raise SystemExit(f"ArduPilot waf was not found at {waf}")

    mount_parent = ardupilot / "libraries" / "MotorBlimpControllerTests"
    mount = mount_parent / "tests"
    if mount_parent.exists() or mount_parent.is_symlink():
        raise SystemExit(
            f"Refusing to replace existing test mount {mount_parent}; remove it or choose another checkout"
        )

    python = python_for_ardupilot(ardupilot)
    environment = os.environ.copy()
    environment["ARDUMOTORBLIMP_SOURCE_DIR"] = str(source_repository)
    environment["PATH"] = f"{python.parent}{os.pathsep}{environment.get('PATH', '')}"

    mount_parent.mkdir()
    mount.symlink_to(source_repository / "tests", target_is_directory=True)
    try:
        configured = (ardupilot / "build" / "sitl" / "ap_config.h").is_file()
        if args.configure or not configured:
            run(
                [str(python), str(waf), "configure", "--board", "sitl", "--debug"],
                ardupilot,
                environment,
            )

        run(
            [
                str(python),
                str(waf),
                "build",
                "--targets=tests/test_motorblimp_control",
            ],
            ardupilot,
            environment,
        )
        test_binary = ardupilot / "build" / "sitl" / "tests" / "test_motorblimp_control"
        if not test_binary.is_file():
            raise RuntimeError(f"waf did not create {test_binary}")
        run([str(test_binary)], ardupilot, environment)
    finally:
        if mount.is_symlink():
            mount.unlink()
        if mount_parent.is_dir():
            shutil.rmtree(mount_parent)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
