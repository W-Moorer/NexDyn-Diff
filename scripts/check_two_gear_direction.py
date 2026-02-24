import argparse
import csv
import subprocess
import sys
from pathlib import Path


def _sign(value: float, eps: float) -> int:
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


def _read_second_column(csv_path: Path) -> tuple[list[float], list[float]]:
    times: list[float] = []
    vals: list[float] = []
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        _ = next(reader, None)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                times.append(float(row[0]))
                vals.append(float(row[1]))
            except ValueError:
                continue
    return times, vals


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    default_model = root / "models" / "fuzajiaolian_two_gear_baseline.xml"
    default_out_drive = root / "output" / "Gear21_Vel_Rx.csv"
    default_out_free = root / "output" / "Link_Gear_Vel_Rx.csv"

    default_bin = root / "build" / "bin" / "Release" / "complex_gear_sim.exe"
    if not default_bin.exists():
        debug_bin = root / "build" / "bin" / "Debug" / "complex_gear_sim.exe"
        if debug_bin.exists():
            default_bin = debug_bin

    parser = argparse.ArgumentParser(
        description="Run two-gear baseline and verify opposite rotation direction."
    )
    parser.add_argument("--sim-bin", type=Path, default=default_bin)
    parser.add_argument("--model", type=Path, default=default_model)
    parser.add_argument("--eps", type=float, default=1e-6)
    parser.add_argument("--min-valid", type=int, default=100)
    args = parser.parse_args()

    sim_bin = args.sim_bin.resolve()
    model = args.model.resolve()
    if not sim_bin.exists():
        print(f"Missing simulator binary: {sim_bin}", file=sys.stderr)
        return 2
    if not model.exists():
        print(f"Missing model file: {model}", file=sys.stderr)
        print("Tip: run `python scripts/extract_rmd.py` first.", file=sys.stderr)
        return 2

    cmd = [str(sim_bin), str(model), "0", "1.0"]
    print(f"Running: {' '.join(cmd)}")
    run = subprocess.run(cmd, cwd=root, capture_output=True, text=True)
    if run.returncode != 0:
        print(run.stdout)
        print(run.stderr, file=sys.stderr)
        return run.returncode

    if run.stdout:
        lines = [ln for ln in run.stdout.splitlines() if ln.strip()]
        tail = lines[-6:] if len(lines) > 6 else lines
        print("\n".join(tail))

    if not default_out_drive.exists() or not default_out_free.exists():
        print("Expected output CSV files are missing.", file=sys.stderr)
        print(f"drive: {default_out_drive}", file=sys.stderr)
        print(f"free : {default_out_free}", file=sys.stderr)
        return 2

    t_drive, v_drive = _read_second_column(default_out_drive)
    t_free, v_free = _read_second_column(default_out_free)
    n = min(len(v_drive), len(v_free), len(t_drive), len(t_free))
    if n == 0:
        print("No samples found in output CSV files.", file=sys.stderr)
        return 2

    valid = 0
    same_sign = 0
    first_violations: list[tuple[int, float, float, float]] = []

    for i in range(n):
        sd = _sign(v_drive[i], args.eps)
        sf = _sign(v_free[i], args.eps)
        if sd == 0 or sf == 0:
            continue
        valid += 1
        if sd == sf:
            same_sign += 1
            if len(first_violations) < 10:
                first_violations.append((i, t_drive[i], v_drive[i], v_free[i]))

    print(f"samples={n}, valid={valid}, same_sign={same_sign}, eps={args.eps}")
    if valid < args.min_valid:
        print(f"FAIL: valid samples {valid} < min-valid {args.min_valid}", file=sys.stderr)
        return 3
    if same_sign > 0:
        print("FAIL: detected same-sign drive/freewheel velocity samples.", file=sys.stderr)
        for idx, t, vd, vf in first_violations:
            print(
                f"  idx={idx} t={t:.9f} drive={vd:.9f} freewheel={vf:.9f}",
                file=sys.stderr,
            )
        return 4

    print("PASS: two-gear baseline keeps opposite-sign transmission.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
