#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CSV = ROOT_DIR / "output" / "gear_example_cpp.csv"
DEFAULT_PNG = ROOT_DIR / "figures" / "gear_example_cpp.png"


def configure_times_font() -> None:
    # Prefer English Times fonts.
    rcParams["font.family"] = "serif"
    rcParams["font.serif"] = ["Times New Roman", "Times", "Nimbus Roman"]
    rcParams["mathtext.fontset"] = "stix"


def load_csv(csv_path: Path) -> np.ndarray:
    data = np.genfromtxt(str(csv_path), delimiter=",", names=True)
    if data.shape == ():
        data = np.array([data], dtype=data.dtype)
    return data


def plot_csv(data: np.ndarray, out_png: Path, title: str | None = None) -> None:
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(8.0, 8.0))

    ax[0].plot(data["time"], data["fn_sum"], label="fn_sum")
    ax[0].plot(data["time"], data["fworld_mag"], label="fworld_mag")
    ax[0].set_ylabel("Force (N)")
    ax[0].legend(loc="best")
    ax[0].grid(True, alpha=0.3)

    ax[1].step(data["time"], data["ncon"], where="post", label="ncon")
    ax[1].set_ylabel("Contacts")
    ax[1].legend(loc="best")
    ax[1].grid(True, alpha=0.3)

    ax[2].plot(data["time"], data["freewheel_vel"], label="freewheel_vel")
    ax[2].plot(data["time"], data["drive_vel"], label="drive_vel")
    ax[2].set_ylabel("Angular Vel (rad/s)")
    ax[2].set_xlabel("Time (s)")
    ax[2].legend(loc="best")
    ax[2].grid(True, alpha=0.3)

    if title:
        fig.suptitle(title)
        fig.tight_layout(rect=[0, 0, 1, 0.97])
    else:
        fig.tight_layout()

    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot gear_example_cpp.csv to figures with Times font."
    )
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="Input CSV path.")
    parser.add_argument("--png", type=Path, default=DEFAULT_PNG, help="Output figure path.")
    parser.add_argument("--title", type=str, default="", help="Optional figure title.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_path = args.csv.resolve()
    png_path = args.png.resolve()

    configure_times_font()
    data = load_csv(csv_path)
    plot_csv(data, png_path, args.title if args.title else None)

    print(f"csv: {csv_path}")
    print(f"png: {png_path}")


if __name__ == "__main__":
    main()
