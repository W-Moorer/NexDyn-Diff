from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


@dataclass
class CompareMetric:
    name: str
    max_abs_err: float
    mean_abs_err: float


def load_pos_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if df.shape[1] < 4:
        raise ValueError(f"{path} has fewer than 4 columns")
    df = df.iloc[:, :4].copy()
    df.columns = ["Time", "X", "Y", "Z"]
    return df


def load_vel_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if df.shape[1] < 2:
        raise ValueError(f"{path} has fewer than 2 columns")
    df = df.iloc[:, :2].copy()
    df.columns = ["Time", "Rz"]
    return df


def interp_series(time_src: np.ndarray, value_src: np.ndarray, time_dst: np.ndarray) -> np.ndarray:
    return np.interp(time_dst, time_src, value_src)


def compare_pos(
    baseline_path: Path,
    sim_path: Path,
    fig_path: Path,
    title: str,
) -> CompareMetric:
    base = load_pos_csv(baseline_path)
    sim = load_pos_csv(sim_path)

    t_base = base["Time"].to_numpy()
    t_sim = sim["Time"].to_numpy()

    sim_x = interp_series(t_sim, sim["X"].to_numpy(), t_base)
    sim_y = interp_series(t_sim, sim["Y"].to_numpy(), t_base)
    sim_z = interp_series(t_sim, sim["Z"].to_numpy(), t_base)

    err_x = np.abs(base["X"].to_numpy() - sim_x)
    err_y = np.abs(base["Y"].to_numpy() - sim_y)
    err_z = np.abs(base["Z"].to_numpy() - sim_z)
    all_err = np.concatenate([err_x, err_y, err_z])

    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(8.0, 8.0))
    ax[0].plot(t_base, base["X"], label="RecurDyn", linewidth=1.2)
    ax[0].plot(t_base, sim_x, label="simcore", linewidth=1.2)
    ax[0].set_ylabel("X (mm)")
    ax[0].grid(True, alpha=0.3)
    ax[0].legend(loc="best")

    ax[1].plot(t_base, base["Y"], label="RecurDyn", linewidth=1.2)
    ax[1].plot(t_base, sim_y, label="simcore", linewidth=1.2)
    ax[1].set_ylabel("Y (mm)")
    ax[1].grid(True, alpha=0.3)
    ax[1].legend(loc="best")

    ax[2].plot(t_base, base["Z"], label="RecurDyn", linewidth=1.2)
    ax[2].plot(t_base, sim_z, label="simcore", linewidth=1.2)
    ax[2].set_ylabel("Z (mm)")
    ax[2].set_xlabel("Time (s)")
    ax[2].grid(True, alpha=0.3)
    ax[2].legend(loc="best")

    fig.suptitle(title)
    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.96])
    fig.savefig(fig_path, dpi=180)
    plt.close(fig)

    return CompareMetric(
        name=title,
        max_abs_err=float(np.max(all_err)),
        mean_abs_err=float(np.mean(all_err)),
    )


def compare_vel(
    baseline_path: Path,
    sim_path: Path,
    fig_path: Path,
    title: str,
) -> CompareMetric:
    base = load_vel_csv(baseline_path)
    sim = load_vel_csv(sim_path)

    t_base = base["Time"].to_numpy()
    t_sim = sim["Time"].to_numpy()
    sim_rz = interp_series(t_sim, sim["Rz"].to_numpy(), t_base)

    err = np.abs(base["Rz"].to_numpy() - sim_rz)

    fig, ax = plt.subplots(1, 1, figsize=(8.0, 4.0))
    ax.plot(t_base, base["Rz"], label="RecurDyn", linewidth=1.2)
    ax.plot(t_base, sim_rz, label="simcore", linewidth=1.2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Rz (rad/s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=180)
    plt.close(fig)

    return CompareMetric(
        name=title,
        max_abs_err=float(np.max(err)),
        mean_abs_err=float(np.mean(err)),
    )


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    baseline_dir = repo_root / "assets" / "models" / "slider_crank"
    sim_dir = repo_root / "output" / "slider_crank"
    fig_dir = repo_root / "figures" / "slider_crank"

    fig_dir.mkdir(parents=True, exist_ok=True)
    sim_dir.mkdir(parents=True, exist_ok=True)

    pos_jobs = [
        ("Link_Body1_Pos.csv", "sim_Link_Body1_Pos.csv", "Body1_Pos_Comparison"),
        ("Link_Body2_Pos.csv", "sim_Link_Body2_Pos.csv", "Body2_Pos_Comparison"),
        ("Link_Body3_Pos.csv", "sim_Link_Body3_Pos.csv", "Body3_Pos_Comparison"),
        ("Sphere_Body5_Pos.csv", "sim_Sphere_Body5_Pos.csv", "Body5_Pos_Comparison"),
    ]
    vel_jobs = [
        ("Link_Body2_Vel_Rz.csv", "sim_Link_Body2_Vel_Rz.csv", "Body2_Vel_Rz_Comparison"),
        ("Sphere_Body5_Vel_Rz.csv", "sim_Sphere_Body5_Vel_Rz.csv", "Body5_Vel_Rz_Comparison"),
    ]

    metrics: list[CompareMetric] = []

    for base_name, sim_name, title in pos_jobs:
        metrics.append(
            compare_pos(
                baseline_dir / base_name,
                sim_dir / sim_name,
                fig_dir / f"{title}.png",
                title,
            )
        )

    for base_name, sim_name, title in vel_jobs:
        metrics.append(
            compare_vel(
                baseline_dir / base_name,
                sim_dir / sim_name,
                fig_dir / f"{title}.png",
                title,
            )
        )

    metrics_path = sim_dir / "comparison_metrics.csv"
    with metrics_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Name", "MaxAbsError", "MeanAbsError"])
        for m in metrics:
            writer.writerow([m.name, f"{m.max_abs_err:.12g}", f"{m.mean_abs_err:.12g}"])

    for m in metrics:
        print(f"{m.name}: max={m.max_abs_err:.9g}, mean={m.mean_abs_err:.9g}")
    print(f"wrote metrics: {metrics_path}")


if __name__ == "__main__":
    main()
