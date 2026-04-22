#!/usr/bin/env python3
"""Run the implemented ring_buffer_cam doc cases in isolated mode with standalone UCDBs."""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from collections import OrderedDict
from pathlib import Path


TB = Path(__file__).resolve().parents[1]
UVM_DIR = TB / "uvm"
BASE_TEST = UVM_DIR / "base_test.sv"
WORK_LOGS = UVM_DIR / "work_uvm" / "logs"
PUB_LOGS = UVM_DIR / "logs"
PUB_COV = UVM_DIR / "cov_after"
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
    parser.add_argument(
        "--doc-build-matrix",
        action="store_true",
        help="use the per-case build variant declared in the DV docs instead of one global --rtl-variant",
    )
    parser.add_argument("--verbosity", default="UVM_LOW")
    parser.add_argument("--case-list", default="")
    parser.add_argument("--case-regex", default="")
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--continue-on-fail", action="store_true")
    return parser.parse_args()


def derive_build_tag(implementation: str) -> str:
    text = implementation.lower()
    if "p4_n4_pipe4" in text or "p4 variant" in text:
        return "p4_n4_pipe4"
    if "p2_pipe1" in text or "pipe_stages=1" in text:
        return "p2_pipe1"
    if "p2_pipe2" in text or "pipe_stages=2" in text:
        return "p2_pipe2"
    if "p2_pipe3" in text or "pipe_stages=3" in text:
        return "p2_pipe3"
    return "default_p2_pipe4"


def doc_case_matrix() -> list[dict[str, str]]:
    cases: list[dict[str, str]] = []
    for doc in BUCKET_FILES.values():
        for raw in doc.read_text(encoding="utf-8").splitlines():
            cols = [part.strip() for part in raw.strip().strip("|").split("|")]
            if not cols or cols[0] == "case_id":
                continue
            if len(cols) < 3:
                continue
            case_id = cols[0]
            if not re.fullmatch(r"[BEPX]\d{3}", case_id):
                continue
            cases.append(
                {
                    "case_id": case_id,
                    "rtl_variant": derive_build_tag(cols[2]),
                }
            )
    return cases


def implemented_cases() -> set[str]:
    text = BASE_TEST.read_text(encoding="utf-8")
    return set(re.findall(r'case_id == "([BEPX]\d{3})"', text))


def ucdb_exists(case_id: str, rtl_variant: str, seed: int) -> bool:
    path = UVM_DIR / f"cov_{rtl_variant}" / f"{case_id}_s{seed}.ucdb"
    return path.exists() and path.stat().st_size > 0


def filter_cases(all_cases: list[dict[str, str]], args: argparse.Namespace) -> list[dict[str, str]]:
    selected = list(all_cases)
    if args.case_list:
        wanted = {case_id.strip() for case_id in args.case_list.split(",") if case_id.strip()}
        selected = [item for item in selected if item["case_id"] in wanted]
    if args.case_regex:
        pattern = re.compile(args.case_regex)
        selected = [item for item in selected if pattern.search(item["case_id"])]
    return selected


def run_case(case_id: str, rtl_variant: str, args: argparse.Namespace) -> tuple[int, str]:
    cmd = [
        "make",
        "-C",
        str(UVM_DIR),
        "run_case",
        f"CASE_ID={case_id}",
        f"SEED={args.seed}",
        f"RTL_VARIANT={rtl_variant}",
        f"VERBOSITY={args.verbosity}",
    ]
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return result.returncode, result.stdout


def publish_case_artifacts(case_id: str, rtl_variant: str, args: argparse.Namespace) -> None:
    work_log = WORK_LOGS / f"test_case_engine_{case_id}_s{args.seed}.log"
    pub_log = PUB_LOGS / f"{case_id}_{rtl_variant}_s{args.seed}.log"
    work_ucdb = UVM_DIR / f"cov_{rtl_variant}" / f"{case_id}_s{args.seed}.ucdb"
    pub_ucdb = PUB_COV / f"{case_id}_s{args.seed}.ucdb"

    PUB_LOGS.mkdir(parents=True, exist_ok=True)
    PUB_COV.mkdir(parents=True, exist_ok=True)

    if work_log.exists():
        shutil.copy2(work_log, pub_log)
    if work_ucdb.exists() and work_ucdb.stat().st_size > 0:
        shutil.copy2(work_ucdb, pub_ucdb)


def main() -> int:
    args = parse_args()
    ordered = doc_case_matrix()
    if not args.doc_build_matrix:
        ordered = [
            {
                "case_id": item["case_id"],
                "rtl_variant": args.rtl_variant,
            }
            for item in ordered
        ]
    supported = implemented_cases()
    cases = [item for item in ordered if item["case_id"] in supported]
    cases = filter_cases(cases, args)

    if not cases:
        print("no cases selected", file=sys.stderr)
        return 2

    variants = sorted({item["rtl_variant"] for item in cases})
    print(
        f"[isolated] selected={len(cases)} seed={args.seed} "
        f"rtl_variant={'doc-build-matrix' if args.doc_build_matrix else args.rtl_variant} "
        f"variants={','.join(variants)}",
        flush=True,
    )

    failures: list[str] = []
    executed = 0
    skipped = 0
    total = len(cases)

    for idx, item in enumerate(cases, start=1):
        case_id = item["case_id"]
        rtl_variant = item["rtl_variant"]
        if args.skip_existing and ucdb_exists(case_id, rtl_variant, args.seed):
            skipped += 1
            print(f"[isolated] skip {idx}/{total} {case_id} [{rtl_variant}]", flush=True)
            continue

        print(f"[isolated] run  {idx}/{total} {case_id} [{rtl_variant}]", flush=True)
        rc, out = run_case(case_id, rtl_variant, args)
        executed += 1
        if rc != 0:
            failures.append(case_id)
            tail = "\n".join(out.splitlines()[-40:])
            print(f"[isolated] fail {case_id}\n{tail}", flush=True)
            if not args.continue_on_fail:
                break
        else:
            publish_case_artifacts(case_id, rtl_variant, args)

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
