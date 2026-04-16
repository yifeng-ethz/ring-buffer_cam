#!/usr/bin/env python3
"""Build ring_buffer_cam/tb/DV_REPORT.json from the plan docs and live logs."""

from __future__ import annotations

import argparse
import functools
import json
import re
import subprocess
from datetime import date
from pathlib import Path

TB = Path(__file__).resolve().parents[1]
WORK_LOGS = TB / "uvm" / "work_uvm" / "logs"
PUB_LOGS = TB / "uvm" / "logs"
PUB_COV = TB / "uvm" / "cov_after"
REPORT_JSON = TB / "DV_REPORT.json"
SKILL_REPORT_GEN = Path.home() / ".codex" / "skills" / "dv-workflow" / "scripts" / "dv_report_gen.py"

RTL_VARIANT = "default_p2_pipe4"
SEED = 1

BUCKET_FILES = {
    "BASIC": TB / "DV_BASIC.md",
    "EDGE": TB / "DV_EDGE.md",
    "PROF": TB / "DV_PROF.md",
    "ERROR": TB / "DV_ERROR.md",
}
CROSS_FILE = TB / "DV_CROSS.md"
BASE_TEST_SV = TB / "uvm" / "base_test.sv"


def normalize_alias(cell: str) -> str | None:
    cell = cell.strip()
    if not cell or cell.lower() == "none":
        return None
    match = re.search(r"`([^`]+)`", cell)
    if match and match.group(1).startswith("test_"):
        return match.group(1)
    if cell.startswith("test_"):
        return cell.split()[0]
    return None


def parse_markdown_table(path: Path) -> list[list[str]]:
    rows: list[list[str]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line.startswith("|"):
            continue
        cols = [part.strip() for part in line.strip("|").split("|")]
        if not cols:
            continue
        if all(set(col) <= {"-", ":"} for col in cols):
            continue
        rows.append(cols)
    return rows


def parse_bucket_cases(bucket: str, path: Path) -> list[dict]:
    cases: list[dict] = []
    for cols in parse_markdown_table(path):
        if not cols or cols[0] == "case_id":
            continue
        case_id = cols[0]
        if not re.fullmatch(r"[BEPX]\d{3}", case_id):
            continue
        method = cols[1]
        implementation = cols[2]
        alias = normalize_alias(cols[3])
        scenario = cols[4]
        primary_checks = cols[5]
        cases.append(
            {
                "case_id": case_id,
                "full_case_id": case_id,
                "bucket": bucket,
                "method": method,
                "implementation_decl": implementation,
                "alias": alias,
                "scenario": scenario,
                "description": scenario,
                "primary_checks": primary_checks,
                "contract_anchor": "",
                "seed": SEED,
                "build_tag": RTL_VARIANT,
                "isolated_effort": "smoke",
                "standalone_coverage": {},
                "isolated_cov_per_txn": {},
                "bucket_gain_by_case": {},
                "bucket_merged_total_after_case": {},
                "bucket_gain_per_txn": {},
            }
        )
    return cases


def parse_cross_rows(path: Path) -> list[dict]:
    runs: list[dict] = []
    for cols in parse_markdown_table(path):
        if not cols or cols[0] == "case_id":
            continue
        run_id = cols[0]
        if not run_id.startswith("CROSS-"):
            continue
        runs.append(
            {
                "run_id": run_id,
                "kind": cols[1],
                "implementation_decl": cols[2],
                "sequence_name": cols[3],
                "primary_checks": cols[4],
                "build_tag": RTL_VARIANT,
            }
        )
    return runs


@functools.lru_cache(maxsize=1)
def parse_implemented_engine_cases(path: Path) -> set[str]:
    text = path.read_text(encoding="utf-8")
    return set(re.findall(r'case_id == "([BEPX]\d{3})"', text))


def parse_log(log_path: Path) -> dict | None:
    if not log_path.is_file():
        return None

    text = log_path.read_text(encoding="utf-8", errors="replace")
    cov_match = re.search(r"cg_output=([0-9.]+)% cg_hit_data=([0-9.]+)%", text)
    scb_match = re.search(
        r"Summary: pushed=(\d+) popped=(\d+) remaining=(\d+) overwrites=(\d+)(?: unexpected=(\d+))?",
        text,
    )
    err_count_match = re.search(r"# UVM_ERROR :\s+(\d+)", text)
    throughput = re.findall(r"Throughput: (\d+) hits in (\d+) cycles \(([0-9.]+)%\)", text)
    case_markers = re.findall(r"\[CASE\] (CASE_(?:BEGIN|END) .*?)$", text, flags=re.MULTILINE)
    error_lines = re.findall(r"UVM_ERROR .*?\[[^\]]+\] (.*)", text)

    pushed = int(scb_match.group(1)) if scb_match else 0
    popped = int(scb_match.group(2)) if scb_match else 0
    remaining = int(scb_match.group(3)) if scb_match else 0
    overwrites = int(scb_match.group(4)) if scb_match else 0
    unexpected = int(scb_match.group(5)) if scb_match and scb_match.group(5) else 0
    uvm_errors = int(err_count_match.group(1)) if err_count_match else 0
    passed = "*** TEST PASSED ***" in text and uvm_errors == 0

    summary = {
        "pushed": pushed,
        "popped": popped,
        "remaining": remaining,
        "overwrites": overwrites,
        "unexpected_outputs": unexpected,
        "uvm_error_count": uvm_errors,
        "case_markers": len(case_markers),
    }
    if cov_match:
        summary["cg_output_pct"] = float(cov_match.group(1))
        summary["cg_hit_data_pct"] = float(cov_match.group(2))
    if throughput:
        summary["throughput_samples"] = len(throughput)
        summary["max_throughput_pct"] = max(float(item[2]) for item in throughput)
    if error_lines:
        summary["failure_reason"] = error_lines[0]

    return {
        "passed": passed,
        "observed_txn": pushed,
        "summary": summary,
    }


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def write_text(path: Path, content: str) -> None:
    ensure_parent(path)
    if not content.endswith("\n"):
        content += "\n"
    path.write_text(content, encoding="utf-8")


def case_log_candidates(case_data: dict) -> list[Path]:
    case_id = case_data["case_id"]
    alias = case_data.get("alias")
    candidates = [WORK_LOGS / f"test_case_engine_{case_id}_s{SEED}.log"]
    if alias:
        candidates.append(WORK_LOGS / f"{alias}_s{SEED}.log")
    return candidates


def cross_log_candidates(run_data: dict) -> list[Path]:
    run_id = run_data["run_id"]
    if run_id == "CROSS-001":
        return [WORK_LOGS / f"test_case_engine_BASIC_bucket_frame_s{SEED}.log"]
    if run_id == "CROSS-002":
        return [WORK_LOGS / f"test_case_engine_EDGE_bucket_frame_s{SEED}.log"]
    if run_id == "CROSS-003":
        return [WORK_LOGS / f"test_case_engine_PROF_bucket_frame_s{SEED}.log"]
    if run_id == "CROSS-004":
        return [WORK_LOGS / f"test_case_engine_ERROR_bucket_frame_s{SEED}.log"]
    return [WORK_LOGS / f"test_case_engine_{run_id}_s{SEED}.log"]


def publish_log_artifact(name: str, src: Path | None, scenario: str, implemented: bool) -> None:
    target = PUB_LOGS / f"{name}_{RTL_VARIANT}_s{SEED}.log"
    if target.exists() or target.is_symlink():
        target.unlink()

    if src and src.is_file():
        ensure_parent(target)
        target.symlink_to(src.resolve())
        return

    state = "implemented but not yet rerun" if implemented else "planned only"
    write_text(
        target,
        f"{name} has no live log artifact.\n"
        f"State: {state}\n"
        f"Scenario: {scenario}\n",
    )


def publish_cov_placeholder(name: str) -> None:
    path = PUB_COV / f"{name}_s{SEED}.ucdb"
    write_text(
        path,
        f"{name} has no standalone UCDB artifact yet.\n"
        "The current ring_buffer_cam workflow captures functional-covergroup percentages in logs,\n"
        "but code-coverage UCDB save/merge is still pending promotion.\n",
    )


def build_case(case_data: dict) -> dict:
    entry = dict(case_data)
    implemented_cases = parse_implemented_engine_cases(BASE_TEST_SV)
    implemented = entry["case_id"] in implemented_cases or entry.get("alias") is not None
    entry["implemented"] = implemented

    chosen_log: Path | None = None
    log_info = None
    for candidate in case_log_candidates(entry):
        log_info = parse_log(candidate)
        if log_info:
          chosen_log = candidate
          break

    if log_info:
        entry["passed"] = log_info["passed"]
        entry["observed_txn"] = log_info["observed_txn"]
        entry["log_summary"] = log_info["summary"]
        entry["implementation_mode"] = (
            "case_engine_log" if chosen_log and chosen_log.name.startswith("test_case_engine_")
            else "legacy_alias_log"
        )
    else:
        entry["observed_txn"] = 0
        entry["log_summary"] = {}
        entry["implementation_mode"] = "planned_only"

    publish_log_artifact(entry["case_id"], chosen_log, entry["scenario"], implemented)
    publish_cov_placeholder(entry["case_id"])
    return entry


def build_bucket(bucket_name: str, bucket_cases: list[dict]) -> tuple[dict, dict]:
    built_cases = [build_case(item) for item in bucket_cases]
    planned = len(built_cases)
    evidenced = sum(1 for item in built_cases if item.get("passed") in (True, False))
    passed = sum(1 for item in built_cases if item.get("passed") is True)

    bucket_data = {
        "planned_cases": planned,
        "evidenced_cases": evidenced,
        "merged_bucket_total": {},
        "cases": built_cases,
        "merge_trace": [
            {
                "step": idx,
                "case_id": item["case_id"],
                "full_case_id": item["full_case_id"],
                "merged_total_after_case": {},
            }
            for idx, item in enumerate(built_cases, start=1)
        ],
    }
    bucket_summary = {
        "bucket": bucket_name,
        "planned_cases": planned,
        "evidenced_cases": evidenced,
        "merged_bucket_total": {},
        "functional_coverage": {
            "pct": round((100.0 * passed / planned), 2) if planned else 0.0,
            "evidenced": passed,
            "planned": planned,
        },
    }
    return bucket_data, bucket_summary


def build_signoff_run(run_data: dict) -> dict:
    entry = dict(run_data)
    chosen_log: Path | None = None
    log_info = None
    for candidate in cross_log_candidates(entry):
        log_info = parse_log(candidate)
        if log_info:
            chosen_log = candidate
            break

    publish_log_artifact(entry["run_id"], chosen_log, entry["sequence_name"], chosen_log is not None)
    publish_cov_placeholder(entry["run_id"])

    if log_info:
        summary = log_info["summary"]
        entry["case_count"] = max(1, summary.get("case_markers", 0) // 2)
        entry["effort"] = "smoke"
        entry["iter_cap"] = "n/a"
        entry["payload_cap"] = "n/a"
        entry["code_coverage"] = {}
        entry["cross_summary"] = {
            "pct": summary.get("cg_output_pct", 0.0),
            "txns": log_info["observed_txn"],
            "queued_overlap": 0,
            "counter_checks_failed": 0 if log_info["passed"] else summary.get("uvm_error_count", 0),
            "unexpected_outputs": summary.get("unexpected_outputs", 0),
            "curve": "",
        }
    else:
        entry["case_count"] = 0
        entry["effort"] = "planned"
        entry["iter_cap"] = "pending"
        entry["payload_cap"] = "pending"
        entry["code_coverage"] = {}
        entry["cross_summary"] = {
            "pct": 0.0,
            "txns": 0,
            "queued_overlap": 0,
            "counter_checks_failed": 0,
            "unexpected_outputs": 0,
            "curve": "",
        }
    return entry


def build_report() -> dict:
    buckets: dict[str, dict] = {}
    bucket_summary: list[dict] = []
    all_cases: list[dict] = []

    for bucket_name, bucket_file in BUCKET_FILES.items():
        bucket_cases = parse_bucket_cases(bucket_name, bucket_file)
        bucket_data, bucket_sum = build_bucket(bucket_name, bucket_cases)
        buckets[bucket_name] = bucket_data
        bucket_summary.append(bucket_sum)
        all_cases.extend(bucket_data["cases"])

    signoff_runs = [build_signoff_run(run) for run in parse_cross_rows(CROSS_FILE)]

    implemented_count = sum(1 for item in all_cases if item.get("implemented"))
    failed_cases = [item["case_id"] for item in all_cases if item.get("passed") is False]
    passed_cases = sum(1 for item in all_cases if item.get("passed") is True)
    evidenced_cases = sum(1 for item in all_cases if item.get("passed") in (True, False))

    return {
        "report_title": "ring_buffer_cam",
        "dut_name": "ring_buffer_cam",
        "date": str(date.today()),
        "rtl_variant": RTL_VARIANT,
        "seed": SEED,
        "buckets": buckets,
        "bucket_summary": bucket_summary,
        "implementation_summary": {
            "implemented_count": implemented_count,
            "unimplemented_count": len(all_cases) - implemented_count,
            "stale_artifact_without_engine_marker_count": 0,
        },
        "failed_cases": failed_cases,
        "random_cases": [item for item in all_cases if item["method"] == "R"],
        "signoff_runs": signoff_runs,
        "totals": {
            "planned_cases": len(all_cases),
            "evidenced_cases": evidenced_cases,
            "excluded_cases": 0,
            "merged_total_code_coverage": {},
            "functional_coverage": {
                "pct": round((100.0 * passed_cases / len(all_cases)), 2) if all_cases else 0.0,
                "evidenced": passed_cases,
                "planned": len(all_cases),
            },
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json-only", action="store_true", help="write DV_REPORT.json only")
    args = ap.parse_args()

    PUB_LOGS.mkdir(parents=True, exist_ok=True)
    PUB_COV.mkdir(parents=True, exist_ok=True)
    (PUB_COV / "txn_growth").mkdir(parents=True, exist_ok=True)

    report = build_report()
    REPORT_JSON.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    if args.json_only:
        return 0

    subprocess.run(["python3", str(SKILL_REPORT_GEN), "--tb", str(TB)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
