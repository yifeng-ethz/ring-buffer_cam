#!/usr/bin/env python3
"""Run the implemented ring_buffer_cam doc cases in isolated mode with standalone UCDBs."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from collections import OrderedDict
from pathlib import Path


TB = Path(__file__).resolve().parents[1]
UVM_DIR = TB / "uvm"
BASE_TEST = UVM_DIR / "base_test.sv"
BUCKET_FILES = OrderedDict(
    [
        ("BASIC", TB / "DV_BASIC.md"),
        ("EDGE", TB / "DV_EDGE.md"),
        ("PROF", TB / "DV_PROF.md"),
        ("ERROR", TB / "DV_ERROR.md"),
    ]
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--rtl-variant", default="default_p2_pipe4")
    parser.add_argument("--verbosity", default="UVM_LOW")
    parser.add_argument("--case-list", default="")
    parser.add_argument("--case-regex", default="")
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--continue-on-fail", action="store_true")
    return parser.parse_args()


def doc_case_order() -> list[str]:
    cases: list[str] = []
    for doc in BUCKET_FILES.values():
        for raw in doc.read_text(encoding="utf-8").splitlines():
            match = re.match(r"^\|\s*([BEPX]\d{3})\s*\|", raw)
            if match:
                cases.append(match.group(1))
    return cases


def implemented_cases() -> set[str]:
    text = BASE_TEST.read_text(encoding="utf-8")
    return set(re.findall(r'case_id == "([BEPX]\d{3})"', text))


def ucdb_exists(case_id: str, rtl_variant: str, seed: int) -> bool:
    path = UVM_DIR / f"cov_{rtl_variant}" / f"{case_id}_s{seed}.ucdb"
    return path.exists() and path.stat().st_size > 0


def filter_cases(all_cases: list[str], args: argparse.Namespace) -> list[str]:
    selected = list(all_cases)
    if args.case_list:
        wanted = {case_id.strip() for case_id in args.case_list.split(",") if case_id.strip()}
        selected = [case_id for case_id in selected if case_id in wanted]
    if args.case_regex:
        pattern = re.compile(args.case_regex)
        selected = [case_id for case_id in selected if pattern.search(case_id)]
    return selected


def run_case(case_id: str, args: argparse.Namespace) -> tuple[int, str]:
    cmd = [
        "make",
        "-C",
        str(UVM_DIR),
        "run_case",
        f"CASE_ID={case_id}",
        f"SEED={args.seed}",
        f"RTL_VARIANT={args.rtl_variant}",
        f"VERBOSITY={args.verbosity}",
    ]
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return result.returncode, result.stdout


def main() -> int:
    args = parse_args()
    ordered = doc_case_order()
    supported = implemented_cases()
    cases = [case_id for case_id in ordered if case_id in supported]
    cases = filter_cases(cases, args)

    if not cases:
        print("no cases selected", file=sys.stderr)
        return 2

    print(
        f"[isolated] selected={len(cases)} seed={args.seed} rtl_variant={args.rtl_variant}",
        flush=True,
    )

    failures: list[str] = []
    executed = 0
    skipped = 0
    total = len(cases)

    for idx, case_id in enumerate(cases, start=1):
        if args.skip_existing and ucdb_exists(case_id, args.rtl_variant, args.seed):
            skipped += 1
            print(f"[isolated] skip {idx}/{total} {case_id}", flush=True)
            continue

        print(f"[isolated] run  {idx}/{total} {case_id}", flush=True)
        rc, out = run_case(case_id, args)
        executed += 1
        if rc != 0:
            failures.append(case_id)
            tail = "\n".join(out.splitlines()[-40:])
            print(f"[isolated] fail {case_id}\n{tail}", flush=True)
            if not args.continue_on_fail:
                break

    print(
        f"[isolated] done total={total} executed={executed} skipped={skipped} failures={len(failures)}",
        flush=True,
    )
    if failures:
        print("[isolated] failing cases:", flush=True)
        for case_id in failures:
            print(case_id, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
