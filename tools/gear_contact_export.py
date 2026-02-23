#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT_DIR / "models" / "gear_selfcontained.xml"
DEFAULT_CSV = ROOT_DIR / "output" / "gear_example.csv"
DEFAULT_PNG = ROOT_DIR / "output" / "gear_example.png"


def require_id(model: mujoco.MjModel, objtype: mujoco.mjtObj, name: str) -> int:
    idx = mujoco.mj_name2id(model, objtype, name)
    if idx < 0:
        raise RuntimeError(f"Missing '{name}' in model.")
    return idx


def export_csv(model_path: Path, out_csv: Path, ctrl: float, sim_time: float, dt: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.timestep = dt
    data = mujoco.MjData(model)

    act_drive = require_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "drive")
    jnt_freewheel = require_id(model, mujoco.mjtObj.mjOBJ_JOINT, "freewheel")
    jnt_drive = require_id(model, mujoco.mjtObj.mjOBJ_JOINT, "drive")
    geom_gear1 = require_id(model, mujoco.mjtObj.mjOBJ_GEOM, "gear1")
    geom_gear2 = require_id(model, mujoco.mjtObj.mjOBJ_GEOM, "gear2")

    nstep = int(round(sim_time / dt))
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time",
                "fx",
                "fy",
                "fz",
                "fn_sum",
                "fworld_mag",
                "ncon",
                "freewheel_vel",
                "drive_vel",
            ]
        )

        for _ in range(nstep):
            data.ctrl[act_drive] = ctrl
            mujoco.mj_step(model, data)

            fx, fy, fz = 0.0, 0.0, 0.0
            fn_sum = 0.0
            ncon = 0

            for cidx in range(data.ncon):
                contact = data.contact[cidx]
                is_pair = (contact.geom1 == geom_gear1 and contact.geom2 == geom_gear2) or (
                    contact.geom1 == geom_gear2 and contact.geom2 == geom_gear1
                )
                if not is_pair:
                    continue

                ncon += 1

                force_local = np.zeros(6, dtype=np.float64)
                mujoco.mj_contactForce(model, data, cidx, force_local)
                fn_sum += force_local[0]

                frame = contact.frame
                fx += frame[0] * force_local[0] + frame[1] * force_local[1] + frame[2] * force_local[2]
                fy += frame[3] * force_local[0] + frame[4] * force_local[1] + frame[5] * force_local[2]
                fz += frame[6] * force_local[0] + frame[7] * force_local[1] + frame[8] * force_local[2]

            fworld_mag = float(np.sqrt(fx * fx + fy * fy + fz * fz))
            freewheel_vel = data.qvel[model.jnt_dofadr[jnt_freewheel]]
            drive_vel = data.qvel[model.jnt_dofadr[jnt_drive]]

            writer.writerow([data.time, fx, fy, fz, fn_sum, fworld_mag, ncon, freewheel_vel, drive_vel])


def summarize_csv(csv_path: Path) -> None:
    first_nonzero_row = None
    max_force_row = None
    max_force_mag = -1.0
    max_freewheel_vel = 0.0
    max_drive_vel = 0.0

    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            mag = float(row["fworld_mag"])
            ncon = int(row["ncon"])
            freewheel_vel = abs(float(row["freewheel_vel"]))
            drive_vel = abs(float(row["drive_vel"]))

            if first_nonzero_row is None and ncon > 0 and mag > 0:
                first_nonzero_row = row
            if mag > max_force_mag:
                max_force_mag = mag
                max_force_row = row
            if freewheel_vel > max_freewheel_vel:
                max_freewheel_vel = freewheel_vel
            if drive_vel > max_drive_vel:
                max_drive_vel = drive_vel

    print("first_nonzero:", first_nonzero_row)
    print("max_force_row:", max_force_row)
    print("max_abs_freewheel_vel:", max_freewheel_vel)
    print("max_abs_drive_vel:", max_drive_vel)


def plot_png(csv_path: Path, out_png: Path) -> None:
    data = np.genfromtxt(str(csv_path), delimiter=",", names=True)
    if data.shape == ():
        data = np.array([data], dtype=data.dtype)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 8))

    axes[0].plot(data["time"], data["fn_sum"], label="fn_sum")
    axes[0].plot(data["time"], data["fworld_mag"], label="fworld_mag")
    axes[0].set_ylabel("Force (N)")
    axes[0].legend()

    axes[1].plot(data["time"], data["ncon"], label="ncon")
    axes[1].set_ylabel("Contacts")
    axes[1].legend()

    axes[2].plot(data["time"], data["freewheel_vel"], label="freewheel_vel")
    axes[2].plot(data["time"], data["drive_vel"], label="drive_vel")
    axes[2].set_ylabel("Angular Vel (rad/s)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend()

    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulate the self-contained gear model and export CSV + plot."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="MJCF xml model path.")
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="Output CSV path.")
    parser.add_argument("--png", type=Path, default=DEFAULT_PNG, help="Output PNG path.")
    parser.add_argument("--ctrl", type=float, default=1.0, help="Motor control input.")
    parser.add_argument("--sim-time", type=float, default=5.0, help="Simulation duration in seconds.")
    parser.add_argument("--dt", type=float, default=0.001, help="Simulation timestep.")
    parser.add_argument("--plot-only", action="store_true", help="Skip simulation and only plot existing CSV.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model_path = args.model.resolve()
    csv_path = args.csv.resolve()
    png_path = args.png.resolve()

    if not args.plot_only:
        export_csv(model_path, csv_path, args.ctrl, args.sim_time, args.dt)
    summarize_csv(csv_path)
    plot_png(csv_path, png_path)

    print(f"model: {model_path}")
    print(f"csv:   {csv_path}")
    print(f"png:   {png_path}")


if __name__ == "__main__":
    main()
