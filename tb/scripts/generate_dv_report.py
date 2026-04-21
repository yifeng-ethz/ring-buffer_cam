#!/usr/bin/env python3
"""Build ring_buffer_cam/tb/DV_REPORT.json from the plan docs and real evidence."""

from __future__ import annotations

import argparse
import functools
import json
import os
import re
import shutil
import subprocess
import tempfile
from collections import OrderedDict
from datetime import date
from pathlib import Path

TB = Path(__file__).resolve().parents[1]
WORK_LOGS = TB / "uvm" / "work_uvm" / "logs"
PUB_LOGS = TB / "uvm" / "logs"
PUB_COV = TB / "uvm" / "cov_after"
REPORT_ROOT = TB / "REPORT"
REPORT_JSON = TB / "DV_REPORT.json"
SKILL_REPORT_GEN = Path.home() / ".codex" / "skills" / "dv-workflow" / "scripts" / "dv_report_gen.py"
RTL_DOC_STYLE_LINTER = Path.home() / ".codex" / "skills" / "rtl-doc-style" / "scripts" / "rtl_doc_style_check.py"

RTL_VARIANT = "default_p2_pipe4"
SEED = 1
PLACEHOLDER_LOG_MARKER = "has no live log artifact."
PLACEHOLDER_UCDB_MARKER = "has no standalone UCDB artifact yet."

VCOVER_CANDIDATES = [
    Path("/data1/questaone_sim/questasim/bin/vcover"),
    Path("/data1/questaone_sim/questasim/linux_x86_64/vcover"),
]
VCOVER_BIN = next((path for path in VCOVER_CANDIDATES if path.exists()), None)
if VCOVER_BIN is None:
    vcover_from_path = shutil.which("vcover")
    VCOVER_BIN = Path(vcover_from_path) if vcover_from_path else None

BUCKET_FILES = OrderedDict(
    {
        "BASIC": TB / "DV_BASIC.md",
        "EDGE": TB / "DV_EDGE.md",
        "PROF": TB / "DV_PROF.md",
        "ERROR": TB / "DV_ERROR.md",
    }
)
CROSS_FILE = TB / "DV_CROSS.md"
BUG_HISTORY_FILE = TB / "BUG_HISTORY.md"
BASE_TEST_SV = TB / "uvm" / "base_test.sv"

METRIC_ROWS = OrderedDict(
    [
        ("Statements", "stmt"),
        ("Branches", "branch"),
        ("Conditions", "cond"),
        ("Expressions", "expr"),
        ("FSM States", "fsm_state"),
        ("FSM Transitions", "fsm_trans"),
        ("Toggles", "toggle"),
    ]
)


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


def derive_build_tag(implementation: str, case_id: str) -> str:
    text = implementation.lower()
    if "p4_n4_pipe4" in text or "p4 variant" in text:
        return "p4_n4_pipe4"
    if "p2_pipe1" in text or "pipe_stages=1" in text:
        return "p2_pipe1"
    if "p2_pipe2" in text or "pipe_stages=2" in text:
        return "p2_pipe2"
    if "p2_pipe3" in text or "pipe_stages=3" in text:
        return "p2_pipe3"
    if "default_p2_pipe4" in text or "pipe_stages=4" in text:
        return "default_p2_pipe4"
    return RTL_VARIANT


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
                "build_tag": derive_build_tag(implementation, case_id),
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
        kind = cols[1] if len(cols) > 1 else "cross"
        sequence_name = cols[2] if len(cols) > 2 else ""
        primary_checks = cols[3] if len(cols) > 3 else ""
        runs.append(
            {
                "run_id": run_id,
                "kind": kind.strip("`"),
                "implementation_decl": "",
                "sequence_name": sequence_name,
                "primary_checks": primary_checks,
                "build_tag": RTL_VARIANT,
            }
        )
    return runs


def parse_bug_history(path: Path) -> list[dict]:
    if not path.is_file():
        return []

    bugs: list[dict] = []
    current_date = ""
    current: dict | None = None
    in_fix_status_block = False

    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.rstrip()
        if line.startswith("## "):
            current_date = line[3:].strip()
            continue

        bug_match = re.match(r"^###\s+(BUG-\d+-([HR])):\s+(.+)$", line)
        if bug_match:
            if current is not None:
                bugs.append(current)
            bug_id = bug_match.group(1)
            tag = bug_match.group(2)
            in_fix_status_block = False
            current = {
                "bug_id": bug_id,
                "class": "Harness" if tag == "H" else "RTL",
                "date": current_date,
                "title": bug_match.group(3).strip(),
                "fix_status": "",
                "commit": "pending",
            }
            continue

        if current is None:
            continue

        if line.startswith("- Fix status:"):
            current["fix_status"] = line.split(":", 1)[1].strip()
            in_fix_status_block = True
        elif in_fix_status_block and line.strip().startswith("- `state`:"):
            current["fix_status"] = line.split(":", 1)[1].strip()
        elif line.startswith("- Commit:"):
            current["commit"] = line.split(":", 1)[1].strip()
            in_fix_status_block = False
        elif line.startswith("- ") and not line.startswith("- Fix status:"):
            in_fix_status_block = False

    if current is not None:
        bugs.append(current)

    return bugs


@functools.lru_cache(maxsize=1)
def parse_implemented_engine_cases(path: Path) -> set[str]:
    text = path.read_text(encoding="utf-8")
    return set(re.findall(r'case_id == "([BEPX]\d{3})"', text))


def read_text_lossy(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def unique_paths(paths: list[Path]) -> list[Path]:
    out: list[Path] = []
    seen: set[Path] = set()
    for path in paths:
        if path in seen:
            continue
        seen.add(path)
        out.append(path)
    return out


def is_placeholder_log(path: Path) -> bool:
    return path.is_file() and PLACEHOLDER_LOG_MARKER in read_text_lossy(path)


def is_real_log(path: Path) -> bool:
    return path.is_file() and not is_placeholder_log(path)


def parse_cov_summary(text: str) -> dict[str, tuple[int, int]]:
    active_instance = None
    totals = {key: [0, 0] for key in METRIC_ROWS.values()}
    instance_match = re.compile(r"^=== Instance:\s+(.+)$")
    metric_match = re.compile(
        r"^\s+(Branches|Conditions|Expressions|FSM States|FSM Transitions|Statements|Toggles)\s+(\d+)\s+(\d+)\s+\d+"
    )

    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        match_instance = instance_match.match(line)
        if match_instance:
            active_instance = match_instance.group(1).strip()
            continue
        if not active_instance or not active_instance.startswith("/tb_top/dut"):
            continue
        match_metric = metric_match.match(line)
        if not match_metric:
            continue
        label = match_metric.group(1)
        bins = int(match_metric.group(2))
        hits = int(match_metric.group(3))
        key = METRIC_ROWS[label]
        totals[key][0] += hits
        totals[key][1] += bins
    return {key: (value[0], value[1]) for key, value in totals.items()}


def diff_cov(curr: dict[str, tuple[int, int]], prev: dict[str, tuple[int, int]]) -> dict[str, tuple[int, int]]:
    delta: dict[str, tuple[int, int]] = {}
    for key in METRIC_ROWS.values():
        curr_hits, curr_bins = curr.get(key, (0, 0))
        prev_hits, prev_bins = prev.get(key, (0, 0))
        bins = curr_bins or prev_bins
        if bins == 0:
            delta[key] = (0, 0)
            continue
        delta[key] = (max(curr_hits - prev_hits, 0), bins)
    return delta


def scale_cov(metrics: dict[str, tuple[int | float, int]], divisor: int) -> dict[str, tuple[float, int]]:
    scaled: dict[str, tuple[float, int]] = {}
    for key in METRIC_ROWS.values():
        hits, bins = metrics.get(key, (0, 0))
        if bins == 0:
            scaled[key] = (0.0, 0)
        else:
            scaled[key] = (float(hits) / divisor, bins)
    return scaled


def metrics_to_raw(metrics: dict[str, tuple[int | float, int]] | None) -> dict[str, dict[str, int | float | None]]:
    if not metrics:
        return {}
    payload: dict[str, dict[str, int | float | None]] = {}
    for key in METRIC_ROWS.values():
        hits, bins = metrics.get(key, (0, 0))
        payload[key] = {
            "hits": hits,
            "bins": bins,
            "pct": None if bins == 0 else round(float(hits) * 100.0 / bins, 2),
        }
    return payload


@functools.lru_cache(maxsize=None)
def summarize_ucdb(ucdb_path: str) -> dict[str, tuple[int, int]] | None:
    path = Path(ucdb_path)
    if not path.is_file() or VCOVER_BIN is None:
        return None

    header = path.read_bytes()[:256]
    header_text = header.decode("utf-8", errors="ignore")
    if PLACEHOLDER_UCDB_MARKER in header_text:
        return None

    try:
        result = subprocess.run(
            [str(VCOVER_BIN), "report", "-code", "bcesft", str(path)],
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError:
        return None
    return parse_cov_summary(result.stdout)


def merge_cov(vcover: Path, ucdbs: list[Path], temp_root: Path) -> dict[str, tuple[int, int]]:
    if not ucdbs:
        return {}
    temp_root.mkdir(parents=True, exist_ok=True)
    if len(ucdbs) == 1:
        summary = summarize_ucdb(str(ucdbs[0]))
        return summary or {}

    merged = temp_root / "merged.ucdb"
    if merged.exists():
        merged.unlink()
    subprocess.run(
        [str(vcover), "merge", str(merged), *[str(path) for path in ucdbs]],
        check=True,
        capture_output=True,
        text=True,
    )
    return summarize_ucdb(str(merged)) or {}


def merge_cov_incremental(
    vcover: Path,
    prev_merged_ucdb: Path | None,
    next_ucdb: Path,
    temp_root: Path,
) -> tuple[Path, dict[str, tuple[int, int]]]:
    if prev_merged_ucdb is None:
        return next_ucdb, summarize_ucdb(str(next_ucdb)) or {}

    temp_root.mkdir(parents=True, exist_ok=True)
    merged = temp_root / "merged.ucdb"
    if merged.exists():
        merged.unlink()
    subprocess.run(
        [str(vcover), "merge", str(merged), str(prev_merged_ucdb), str(next_ucdb)],
        check=True,
        capture_output=True,
        text=True,
    )
    return merged, summarize_ucdb(str(merged)) or {}


def parse_log(log_path: Path) -> dict | None:
    if not is_real_log(log_path):
        return None

    text = read_text_lossy(log_path)
    cov_match = re.search(r"cg_output=([0-9.]+)% cg_hit_data=([0-9.]+)%", text)
    summary_match = re.search(r"Summary:\s+([^\n\r]+)", text)
    err_count_match = re.search(r"# UVM_ERROR :\s+(\d+)", text)
    throughput = re.findall(r"Throughput: (\d+) hits in (\d+) cycles \(([0-9.]+)%\)", text)
    case_markers = re.findall(r"\[CASE\] (CASE_(?:BEGIN|END) .*?)$", text, flags=re.MULTILINE)
    error_lines = re.findall(r"UVM_ERROR .*?\[[^\]]+\] (.*)", text)

    summary_fields: dict[str, int] = {}
    if summary_match:
        for key, value in re.findall(r"([a-z_]+)=([0-9]+)", summary_match.group(1)):
            summary_fields[key] = int(value)

    pushed = summary_fields.get("pushed", 0)
    popped = summary_fields.get("popped", 0)
    remaining = summary_fields.get("remaining", 0)
    overwrites = summary_fields.get("overwrites", 0)
    unexpected = summary_fields.get("unexpected", 0)
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
    for key in ("pending_drain", "overlap_evicted", "overlap_fallback", "subheaders",
                "zero_hit_subheaders", "accepted", "cache_miss_outputs"):
        if key in summary_fields:
            summary[key] = summary_fields[key]
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


def symlink_relative(target: Path, src: Path) -> None:
    ensure_parent(target)
    rel = os.path.relpath(src.resolve(), start=target.parent.resolve())
    target.symlink_to(rel)


def materialize_artifact(target: Path, src: Path) -> None:
    """Publish a durable copy of a volatile run artifact into tb/uvm/."""
    ensure_parent(target)
    resolved_src = src.resolve()
    if target.exists() or target.is_symlink():
        if not target.is_symlink() and target.resolve() == resolved_src:
            return
        target.unlink()
    shutil.copy2(resolved_src, target)


def case_log_candidates(case_data: dict) -> list[Path]:
    case_id = case_data["case_id"]
    alias = case_data.get("alias")
    build_tag = case_data.get("build_tag", RTL_VARIANT)
    candidates = [
        WORK_LOGS / f"test_case_engine_{case_id}_s{SEED}.log",
        PUB_LOGS / f"{case_id}_{build_tag}_s{SEED}.log",
    ]
    if alias:
        candidates.extend(
            [
                WORK_LOGS / f"{alias}_s{SEED}.log",
                PUB_LOGS / f"{alias}_{build_tag}_s{SEED}.log",
            ]
        )
    return unique_paths(candidates)


def case_ucdb_candidates(case_data: dict) -> list[Path]:
    case_id = case_data["case_id"]
    build_tag = case_data.get("build_tag", RTL_VARIANT)
    return unique_paths(
        [
            TB / "uvm" / f"cov_{build_tag}" / f"{case_id}_s{SEED}.ucdb",
            PUB_COV / f"{case_id}_s{SEED}.ucdb",
        ]
    )


def checkpoint_ucdb_candidates(case_data: dict) -> list[tuple[int, Path]]:
    case_id = case_data["case_id"]
    candidates: list[tuple[int, Path]] = []
    pattern = re.compile(rf"^{re.escape(case_id)}_txn(\d+)_s{SEED}\.ucdb$")
    for candidate in sorted((PUB_COV / "txn_growth").glob(f"{case_id}_txn*_s{SEED}.ucdb")):
        match = pattern.match(candidate.name)
        if not match:
            continue
        candidates.append((int(match.group(1)), candidate))
    candidates.sort(key=lambda item: item[0])
    return candidates


def cross_log_candidates(run_data: dict) -> list[Path]:
    run_id = run_data["run_id"]
    work_names = []
    if run_id == "CROSS-001":
        work_names.append("test_case_engine_BASIC_bucket_frame_s1.log")
    elif run_id == "CROSS-002":
        work_names.append("test_case_engine_EDGE_bucket_frame_s1.log")
    elif run_id == "CROSS-003":
        work_names.append("test_case_engine_PROF_bucket_frame_s1.log")
    elif run_id == "CROSS-004":
        work_names.append("test_case_engine_ERROR_bucket_frame_s1.log")
    else:
        work_names.append(f"test_case_engine_{run_id}_s{SEED}.log")

    candidates = [WORK_LOGS / name for name in work_names]
    candidates.append(PUB_LOGS / f"{run_id}_{RTL_VARIANT}_s{SEED}.log")
    return unique_paths(candidates)


def cross_ucdb_candidates(run_data: dict) -> list[Path]:
    run_id = run_data["run_id"]
    return unique_paths(
        [
            TB / "uvm" / f"cov_{RTL_VARIANT}" / f"{run_id}_s{SEED}.ucdb",
            PUB_COV / f"{run_id}_s{SEED}.ucdb",
        ]
    )


def publish_log_artifact(
    name: str,
    src: Path | None,
    scenario: str,
    implemented: bool,
    build_tag: str,
) -> None:
    target = PUB_LOGS / f"{name}_{build_tag}_s{SEED}.log"
    if src and src.is_file():
        materialize_artifact(target, src)
        return

    if is_real_log(target):
        return

    if target.exists() or target.is_symlink():
        target.unlink()

    state = "implemented but not yet rerun" if implemented else "planned only"
    write_text(
        target,
        f"{name} has no live log artifact.\n"
        f"State: {state}\n"
        f"Scenario: {scenario}\n",
    )


def publish_cov_artifact(name: str, src: Path | None) -> None:
    target = PUB_COV / f"{name}_s{SEED}.ucdb"
    if src and src.is_file():
        materialize_artifact(target, src)
        return

    if summarize_ucdb(str(target)):
        return

    if target.exists() or target.is_symlink():
        target.unlink()

    write_text(
        target,
        f"{name} has no standalone UCDB artifact yet.\n"
        "The current ring_buffer_cam workflow has isolated case pages per the dv-workflow skill,\n"
        "but this testcase has not yet produced a real standalone code-coverage UCDB.\n",
    )


def build_case(case_data: dict) -> dict:
    entry = dict(case_data)
    implemented_cases = parse_implemented_engine_cases(BASE_TEST_SV)
    # A case counts as implemented only when the isolated case engine has a
    # real branch for it. Historical aliases are documentation hints, not
    # standalone evidence.
    implemented = entry["case_id"] in implemented_cases
    entry["implemented"] = implemented

    chosen_log: Path | None = None
    log_info = None
    for candidate in case_log_candidates(entry):
        log_info = parse_log(candidate)
        if log_info:
            chosen_log = candidate
            break

    chosen_ucdb: Path | None = None
    standalone_metrics: dict[str, tuple[int, int]] | None = None
    for candidate in case_ucdb_candidates(entry):
        standalone_metrics = summarize_ucdb(str(candidate))
        if standalone_metrics:
            chosen_ucdb = candidate
            break

    if log_info:
        entry["passed"] = log_info["passed"]
        entry["observed_txn"] = log_info["observed_txn"]
        entry["log_summary"] = log_info["summary"]
        entry["implementation_mode"] = (
            "case_engine_log" if chosen_log and chosen_log.name.startswith("test_case_engine_")
            else "published_case_log"
        )
    else:
        entry["observed_txn"] = 0
        entry["log_summary"] = {}
        entry["implementation_mode"] = "implemented_no_live_log" if implemented else "planned_only"

    if standalone_metrics:
        entry["standalone_coverage"] = metrics_to_raw(standalone_metrics)
        if entry["observed_txn"] > 0:
            entry["isolated_cov_per_txn"] = metrics_to_raw(
                scale_cov(standalone_metrics, entry["observed_txn"])
            )

    if entry["method"] == "R":
        txn_growth_curve = []
        for txn_count, candidate in checkpoint_ucdb_candidates(entry):
            checkpoint_metrics = summarize_ucdb(str(candidate))
            if not checkpoint_metrics:
                continue
            txn_growth_curve.append(
                {
                    "txn": txn_count,
                    "coverage": metrics_to_raw(checkpoint_metrics),
                }
            )
        if txn_growth_curve:
            entry["txn_growth_curve"] = txn_growth_curve

    publish_log_artifact(
        entry["case_id"],
        chosen_log,
        entry["scenario"],
        implemented,
        entry.get("build_tag", RTL_VARIANT),
    )
    publish_cov_artifact(entry["case_id"], chosen_ucdb)
    entry["_ucdb_src"] = chosen_ucdb
    entry["_standalone_metrics"] = standalone_metrics or {}
    return entry


def build_bucket(bucket_name: str, bucket_cases: list[dict]) -> tuple[dict, dict]:
    built_cases = [build_case(item) for item in bucket_cases]
    planned = len(built_cases)
    evidenced = sum(1 for item in built_cases if item.get("passed") in (True, False))
    passed = sum(1 for item in built_cases if item.get("passed") is True)

    merge_trace: list[dict] = []
    final_merged_summary: dict[str, tuple[int, int]] = {}
    prev_merged_summary: dict[str, tuple[int, int]] = {}
    prev_merged_ucdb: Path | None = None

    if VCOVER_BIN is not None:
        with tempfile.TemporaryDirectory(prefix=f"{bucket_name.lower()}_cov_") as temp_dir:
            temp_root = Path(temp_dir)
            for idx, item in enumerate(built_cases, start=1):
                merged_total_after_case = {}
                standalone_metrics = item.get("_standalone_metrics") or {}
                ucdb_src = item.get("_ucdb_src")
                if item.get("passed") is True and standalone_metrics and isinstance(ucdb_src, Path):
                    prev_merged_ucdb, merged_summary = merge_cov_incremental(
                        VCOVER_BIN,
                        prev_merged_ucdb,
                        ucdb_src,
                        temp_root / f"step_{idx:03d}",
                    )
                    increment_summary = diff_cov(merged_summary, prev_merged_summary)
                    item["bucket_gain_by_case"] = metrics_to_raw(increment_summary)
                    item["bucket_merged_total_after_case"] = metrics_to_raw(merged_summary)
                    if item["observed_txn"] > 0:
                        item["bucket_gain_per_txn"] = metrics_to_raw(
                            scale_cov(increment_summary, item["observed_txn"])
                        )
                    merged_total_after_case = item["bucket_merged_total_after_case"]
                    prev_merged_summary = merged_summary
                    final_merged_summary = merged_summary

                merge_trace.append(
                    {
                        "step": idx,
                        "case_id": item["case_id"],
                        "full_case_id": item["full_case_id"],
                        "merged_total_after_case": merged_total_after_case,
                    }
                )
    else:
        merge_trace = [
            {
                "step": idx,
                "case_id": item["case_id"],
                "full_case_id": item["full_case_id"],
                "merged_total_after_case": {},
            }
            for idx, item in enumerate(built_cases, start=1)
        ]

    for item in built_cases:
        item.pop("_ucdb_src", None)
        item.pop("_standalone_metrics", None)

    merged_bucket_total = metrics_to_raw(final_merged_summary)
    bucket_data = {
        "planned_cases": planned,
        "evidenced_cases": evidenced,
        "merged_bucket_total": merged_bucket_total,
        "cases": built_cases,
        "merge_trace": merge_trace,
    }
    bucket_summary = {
        "bucket": bucket_name,
        "planned_cases": planned,
        "evidenced_cases": evidenced,
        "merged_bucket_total": merged_bucket_total,
        "functional_coverage": {
            "pct": round((100.0 * passed / planned), 2) if planned else 0.0,
            "evidenced": passed,
            "planned": planned,
        },
    }
    return bucket_data, bucket_summary


def build_signoff_run(run_data: dict) -> dict | None:
    entry = dict(run_data)
    chosen_log: Path | None = None
    log_info = None
    for candidate in cross_log_candidates(entry):
        log_info = parse_log(candidate)
        if log_info:
            chosen_log = candidate
            break

    if not log_info:
        return None

    chosen_ucdb: Path | None = None
    code_coverage = {}
    for candidate in cross_ucdb_candidates(entry):
        summary = summarize_ucdb(str(candidate))
        if summary:
            chosen_ucdb = candidate
            code_coverage = metrics_to_raw(summary)
            break

    publish_log_artifact(entry["run_id"], chosen_log, entry["sequence_name"], True)
    publish_cov_artifact(entry["run_id"], chosen_ucdb)

    summary = log_info["summary"]
    entry["case_count"] = max(1, summary.get("case_markers", 0) // 2)
    entry["effort"] = "smoke"
    entry["iter_cap"] = "n/a"
    entry["payload_cap"] = "n/a"
    entry["code_coverage"] = code_coverage
    entry["cross_summary"] = {
        "pct": summary.get("cg_output_pct", 0.0),
        "txns": log_info["observed_txn"],
        "queued_overlap": 0,
        "counter_checks_failed": 0 if log_info["passed"] else summary.get("uvm_error_count", 0),
        "unexpected_outputs": summary.get("unexpected_outputs", 0),
        "curve": "",
    }
    return entry


def cleanup_generated_report_tree() -> None:
    for subdir in ("buckets", "cases", "cross", "txn_growth"):
        path = REPORT_ROOT / subdir
        if not path.is_dir():
            continue
        for child in path.glob("*.md"):
            child.unlink()


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

    signoff_runs = []
    for run in parse_cross_rows(CROSS_FILE):
        built = build_signoff_run(run)
        if built is not None:
            signoff_runs.append(built)

    implemented_count = sum(1 for item in all_cases if item.get("implemented"))
    failed_cases = [item["case_id"] for item in all_cases if item.get("passed") is False]
    passed_cases = sum(1 for item in all_cases if item.get("passed") is True)
    evidenced_cases = sum(1 for item in all_cases if item.get("passed") in (True, False))
    formal_rows: list[dict] = []

    for bucket_name, bucket_data in buckets.items():
        cases = bucket_data["cases"]
        executed_cases = sum(1 for item in cases if item.get("passed") in (True, False))
        failing_cases = sum(1 for item in cases if item.get("passed") is False)
        observed_txn = sum(int(item.get("observed_txn", 0) or 0) for item in cases)
        asserted_failures = sum(
            int((item.get("log_summary") or {}).get("uvm_error_count", 0) or 0) for item in cases
        )
        unexpected_outputs = sum(
            int((item.get("log_summary") or {}).get("unexpected_outputs", 0) or 0) for item in cases
        )
        planned_cases = len(cases)
        formal_rows.append(
            {
                "scope": bucket_name,
                "planned_cases": planned_cases,
                "executed_cases": executed_cases,
                "executed_ratio_pct": round((100.0 * executed_cases / planned_cases), 2)
                if planned_cases
                else 0.0,
                "observed_txn": observed_txn,
                "failing_cases": failing_cases,
                "asserted_failures": asserted_failures,
                "unexpected_outputs": unexpected_outputs,
            }
        )

    formal_rows.append(
        {
            "scope": "TOTAL",
            "planned_cases": len(all_cases),
            "executed_cases": evidenced_cases,
            "executed_ratio_pct": round((100.0 * evidenced_cases / len(all_cases)), 2)
            if all_cases
            else 0.0,
            "observed_txn": sum(int(item.get("observed_txn", 0) or 0) for item in all_cases),
            "failing_cases": len(failed_cases),
            "asserted_failures": sum(
                int((item.get("log_summary") or {}).get("uvm_error_count", 0) or 0)
                for item in all_cases
            ),
            "unexpected_outputs": sum(
                int((item.get("log_summary") or {}).get("unexpected_outputs", 0) or 0)
                for item in all_cases
            ),
        }
    )

    all_ucdbs: list[Path] = []
    if VCOVER_BIN is not None:
        for bucket in buckets.values():
            for item in bucket["cases"]:
                if item.get("passed") is not True:
                    continue
                case_id = item["case_id"]
                for candidate in case_ucdb_candidates({"case_id": case_id}):
                    if summarize_ucdb(str(candidate)):
                        all_ucdbs.append(candidate)
                        break

    merged_total_code_coverage = {}
    if VCOVER_BIN is not None and all_ucdbs:
        with tempfile.TemporaryDirectory(prefix="all_cov_") as temp_dir:
            merged_total_code_coverage = metrics_to_raw(
                merge_cov(VCOVER_BIN, all_ucdbs, Path(temp_dir))
            )

    return {
        "report_title": "ring_buffer_cam",
        "dut_name": "ring_buffer_cam",
        "date": str(date.today()),
        "rtl_variant": RTL_VARIANT,
        "seed": SEED,
        "bugs": parse_bug_history(BUG_HISTORY_FILE),
        "buckets": buckets,
        "bucket_summary": bucket_summary,
        "formal_summary": formal_rows,
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
            "merged_total_code_coverage": merged_total_code_coverage,
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
    cleanup_generated_report_tree()

    report = build_report()
    REPORT_JSON.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    if args.json_only:
        return 0

    subprocess.run(["python3", str(SKILL_REPORT_GEN), "--tb", str(TB)], check=True)
    if RTL_DOC_STYLE_LINTER.exists():
        subprocess.run(
            ["python3", str(RTL_DOC_STYLE_LINTER), str(TB), "--quiet", "--generator-only"],
            check=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
