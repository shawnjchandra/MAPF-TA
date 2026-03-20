#!/usr/bin/env python3
"""
test_preprocessing.py
=====================
Drop into your MAPF-TA repo root and run:

    python3 test_preprocessing.py

What it does
------------
1. Builds your project with cmake + make (Release mode).
2. Runs the binary on every .json problem file in example_problems/
   with --simulationTime 1  so it finishes immediately after preprocessing.
3. Captures any [PREPROCESS] key=value lines you print from initialize().
4. Writes: preprocessing_results.csv  and  preprocessing_report.txt

Custom params it passes (edit the CONFIG block below to change defaults):
    --numberOfCluster   (-k)
    --radius            (-r)
    --cPenalty

HOW TO EMIT METRICS FROM YOUR C++ CODE
---------------------------------------
Inside your initialize() or preprocess() function, print lines like:

    std::cout << "[PREPROCESS] time_ms="       << elapsed_ms        << std::endl;
    std::cout << "[PREPROCESS] gates="         << AG.gates.size()   << std::endl;
    std::cout << "[PREPROCESS] clusters="      << k                 << std::endl;
    std::cout << "[PREPROCESS] highway_edges=" << E_HW.size()       << std::endl;
    std::cout << "[PREPROCESS] intra_mb="      << intra_mem_mb      << std::endl;
    std::cout << "[PREPROCESS] inter_mb="      << inter_mem_mb      << std::endl;
    std::cout << "[PREPROCESS] n_cells="       << n                 << std::endl;

Multiple key=value pairs on one line also work:
    std::cout << "[PREPROCESS] gates=" << g << " clusters=" << k << std::endl;

Every key=value pair becomes its own CSV column automatically.
"""

import subprocess, sys, os, re, csv, time, json
from pathlib import Path

# ════════════════════════════════════════════════════════════
#  CONFIG  –  edit these to match your usual test parameters
# ════════════════════════════════════════════════════════════
REPO_ROOT           = Path(__file__).parent
BUILD_DIR           = REPO_ROOT / "build"          # reuse your existing build dir
BINARY_NAME         = "lifelong"
RESULTS_CSV         = REPO_ROOT / "preprocessing_results.csv"
RESULTS_TXT         = REPO_ROOT / "preprocessing_report.txt"

# Preprocessing-test run settings
SIMULATION_TIME     = 1        # 1 timestep → exits right after preprocessing
PLAN_TIME_LIMIT     = 0        # 0 ms → planning returns instantly
PREPROCESS_LIMIT    = 60000    # 60 s hard cap for preprocessing

# Your custom algorithm params (from your driver.cpp)
NUM_CLUSTERS        = 50       # --numberOfCluster
RADIUS              = 5        # --radius
C_PENALTY           = 2        # --cPenalty

# Where to look for .json problem files (recursive search)
PROBLEM_DIRS = [
    REPO_ROOT / "example_problems",
    REPO_ROOT / "problem_instances",
]
# Also pick up any .json directly in repo root (e.g. warehouse_small_20_100.json)
EXTRA_ROOT_PATTERNS = ["*.json"]

# Per-run timeout (seconds) before we kill the process
RUN_TIMEOUT = 120
# ════════════════════════════════════════════════════════════


def build():
    print(f"\n{'='*60}")
    print("STEP 1/3  –  Build")
    print(f"{'='*60}")
    BUILD_DIR.mkdir(exist_ok=True)

    r = subprocess.run(
        ["cmake", str(REPO_ROOT), "-B", str(BUILD_DIR), "-DCMAKE_BUILD_TYPE=Release"],
        capture_output=True, text=True
    )
    if r.returncode != 0:
        print("  cmake FAILED:\n", r.stderr[-2000:])
        return None
    print("  cmake  OK")

    r = subprocess.run(
        ["make", "-C", str(BUILD_DIR), "-j", str(os.cpu_count() or 4)],
        capture_output=True, text=True
    )
    if r.returncode != 0:
        print("  make FAILED:\n", r.stderr[-2000:])
        return None

    binary = BUILD_DIR / BINARY_NAME
    if not binary.exists():
        print(f"  Binary not found at {binary}")
        return None

    print(f"  make   OK  →  {binary}")
    return binary


def collect_problems():
    files = []
    seen  = set()
    for d in PROBLEM_DIRS:
        if d.exists():
            for f in d.rglob("*.json"):
                if f.resolve() not in seen:
                    seen.add(f.resolve())
                    files.append(f)
    for pat in EXTRA_ROOT_PATTERNS:
        for f in REPO_ROOT.glob(pat):
            if f.resolve() not in seen:
                seen.add(f.resolve())
                files.append(f)
    # filter out obvious non-problem jsons
    skip = {"output", "config", "result"}
    files = [f for f in files if not any(s in f.stem.lower() for s in skip)]
    return sorted(files)


def parse_json_info(path):
    try:
        data = json.loads(path.read_text())
        rows = data.get("mapSize", [0, 0])
        size = f"{rows[0]}x{rows[1]}" if isinstance(rows, list) and len(rows) >= 2 else "?"
        agents = str(data.get("agentNum", data.get("teamSize", "?")))
        domain = data.get("mapFile", path.stem)
        domain = Path(domain).stem
        return domain, size, agents
    except Exception:
        return path.stem, "?", "?"


def run_one(binary, problem_path):
    domain, size, agents = parse_json_info(problem_path)
    out_file = BUILD_DIR / "tmp_preprocess_test_out.json"

    cmd = [
        str(binary),
        "--inputFile",           str(problem_path),
        "-o",                    str(out_file),
        "--simulationTime",      str(SIMULATION_TIME),
        "--planTimeLimit",       str(PLAN_TIME_LIMIT),
        "--preprocessTimeLimit", str(PREPROCESS_LIMIT),
        "--numberOfCluster",     str(NUM_CLUSTERS),
        "--radius",              str(RADIUS),
        "--cPenalty",            str(C_PENALTY),
        "--outputScreen",        "3",   # suppress verbose simulation output
    ]

    label = f"{problem_path.name}  [{size}, {agents} agents]"
    print(f"  {label:<55}", end="", flush=True)

    t0 = time.perf_counter()
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=RUN_TIMEOUT
        )
        wall = time.perf_counter() - t0
        combined = proc.stdout + "\n" + proc.stderr
        rc = proc.returncode
    except subprocess.TimeoutExpired:
        wall = RUN_TIMEOUT
        combined = "TIMEOUT"
        rc = -999

    # Parse [PREPROCESS] key=value pairs from any output line
    metrics = {}
    for line in combined.splitlines():
        m = re.search(r'\[PREPROCESS\]\s+(.*)', line)
        if m:
            for kv in re.findall(r'(\w+)=([\S]+)', m.group(1)):
                metrics[kv[0]] = kv[1]

    status = ("OK"      if rc in (0, 1)
              else "TIMEOUT" if rc == -999
              else f"ERR({rc})")
    print(f"{status:10}  {wall:6.2f}s")

    # On error, show last few lines to help debug
    if rc not in (0, 1, -999):
        for l in combined.strip().splitlines()[-6:]:
            print(f"         | {l}")

    return {
        "problem":  str(problem_path.relative_to(REPO_ROOT)),
        "domain":   domain,
        "map_size": size,
        "agents":   agents,
        "status":   status,
        "wall_s":   f"{wall:.3f}",
        **metrics
    }


def write_csv(rows):
    standard = ["problem", "domain", "map_size", "agents", "status", "wall_s"]
    extra    = sorted({k for r in rows for k in r if k not in standard})
    fields   = standard + extra
    with open(RESULTS_CSV, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def write_report(rows):
    with open(RESULTS_TXT, "w") as f:
        f.write("PREPROCESSING TEST REPORT\n")
        f.write(f"Params: k={NUM_CLUSTERS}  r={RADIUS}  cPenalty={C_PENALTY}\n")
        f.write(f"simulationTime={SIMULATION_TIME}  planTimeLimit={PLAN_TIME_LIMIT}ms\n")
        f.write("=" * 68 + "\n\n")

        standard = {"problem", "domain", "map_size", "agents", "status", "wall_s"}
        for r in rows:
            f.write(f"  {r['domain']}  ({r['map_size']}, {r['agents']} agents)\n")
            f.write(f"    file   : {r['problem']}\n")
            f.write(f"    status : {r['status']}   wall time = {r['wall_s']}s\n")
            extras = {k: v for k, v in r.items() if k not in standard}
            if extras:
                f.write("    metrics:\n")
                for k, v in sorted(extras.items()):
                    f.write(f"      {k:<28} = {v}\n")
            else:
                f.write("    metrics: (none yet — see HOW TO EMIT METRICS at bottom)\n")
            f.write("\n")

        f.write("=" * 68 + "\n")
        f.write("HOW TO EMIT METRICS FROM initialize()\n")
        f.write("-" * 68 + "\n")
        f.write("Add any of these to your C++ initialize() function:\n\n")
        examples = [
            ("time_ms",       "elapsed_ms"),
            ("n_cells",       "non_obstacle_count"),
            ("clusters",      "k"),
            ("gates",         "AG.gates.size()"),
            ("highway_edges", "E_HW.size()"),
            ("intra_mb",      "intra_mem_mb"),
            ("inter_mb",      "inter_mem_mb"),
        ]
        for key, val in examples:
            f.write(f'  std::cout << "[PREPROCESS] {key}=" << {val} << std::endl;\n')
        f.write("\nMultiple pairs on one line also work:\n")
        f.write('  std::cout << "[PREPROCESS] gates=" << g << " clusters=" << k << std::endl;\n')
        f.write("\nFor timing, wrap your code like this:\n")
        f.write("  auto _t0 = std::chrono::high_resolution_clock::now();\n")
        f.write("  /* ... your preprocessing ... */\n")
        f.write("  auto _t1 = std::chrono::high_resolution_clock::now();\n")
        f.write("  long ms = std::chrono::duration_cast<std::chrono::milliseconds>(_t1-_t0).count();\n")
        f.write('  std::cout << "[PREPROCESS] time_ms=" << ms << std::endl;\n')


def main():
    print("=" * 60)
    print("MAPF-TA  —  Preprocessing Isolation Test")
    print(f"  k={NUM_CLUSTERS}  r={RADIUS}  cPenalty={C_PENALTY}")
    print(f"  simulationTime={SIMULATION_TIME}  planTimeLimit={PLAN_TIME_LIMIT}ms")
    print("=" * 60)

    binary = build()
    if binary is None:
        sys.exit(1)

    problems = collect_problems()
    if not problems:
        print("\nNo .json problem files found.")
        print(f"Expected them in: {[str(d) for d in PROBLEM_DIRS]}")
        sys.exit(1)

    print(f"\n{'='*60}")
    print(f"STEP 2/3  –  Run ({len(problems)} instance(s))")
    print(f"{'='*60}")
    rows = [run_one(binary, p) for p in problems]

    print(f"\n{'='*60}")
    print("STEP 3/3  –  Write results")
    print(f"{'='*60}")
    write_csv(rows)
    write_report(rows)
    print(f"  CSV    →  {RESULTS_CSV}")
    print(f"  Report →  {RESULTS_TXT}")

    ok = sum(1 for r in rows if r["status"] == "OK")
    print(f"\n  {ok}/{len(rows)} instances completed successfully.")

    if ok == 0:
        print("\n  [HINT] All runs failed. Try running manually to see the error:")
        print(f"    {binary} --inputFile example_problems/your_file.json -o /tmp/out.json \\")
        print(f"      --simulationTime 1 --planTimeLimit 0 --numberOfCluster {NUM_CLUSTERS}")


if __name__ == "__main__":
    main()