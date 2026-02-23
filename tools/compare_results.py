import pandas as pd
import numpy as np
import os
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from matplotlib import rcParams

def compare(baseline_file, sim_file, name):
    print(f"Comparing {name}...")
    try:
        # Load Baseline
        # Assuming the first row is header
        df_base = pd.read_csv(baseline_file)
        # Rename columns to Time, X, Y, Z based on observation (Time, X, Y, Z)
        if len(df_base.columns) >= 4:
            df_base = df_base.iloc[:, :4]
            df_base.columns = ['Time', 'X', 'Y', 'Z']
        else:
            print(f"  Error: Baseline file has fewer than 4 columns")
            return

        # Load Sim
        df_sim = pd.read_csv(sim_file)
        
        # Filter Sim Time range to match Baseline
        t_min = df_base['Time'].min()
        t_max = df_base['Time'].max()
        df_sim = df_sim[(df_sim['Time'] >= t_min) & (df_sim['Time'] <= t_max)]
        
        if df_sim.empty:
            print("  Error: Simulation time range does not overlap with baseline")
            return

        # Create interpolators for Sim
        f_x = interp1d(df_sim['Time'], df_sim['X'], kind='linear', fill_value="extrapolate")
        f_y = interp1d(df_sim['Time'], df_sim['Y'], kind='linear', fill_value="extrapolate")
        f_z = interp1d(df_sim['Time'], df_sim['Z'], kind='linear', fill_value="extrapolate")
        
        # Eval at Baseline Time
        sim_x = f_x(df_base['Time'])
        sim_y = f_y(df_base['Time'])
        sim_z = f_z(df_base['Time'])
        
        # Compute Error
        err_x = np.abs(df_base['X'] - sim_x)
        err_y = np.abs(df_base['Y'] - sim_y)
        err_z = np.abs(df_base['Z'] - sim_z)
        
        max_err = np.max([np.max(err_x), np.max(err_y), np.max(err_z)])
        mean_err = np.mean([np.mean(err_x), np.mean(err_y), np.mean(err_z)])
        
        print(f"  Max Error: {max_err:.6f}")
        print(f"  Mean Error: {mean_err:.6f}")
        return df_base, df_sim, sim_x, sim_y, sim_z
        
    except Exception as e:
        print(f"  Error comparing: {e}")
        return None

def plot_compare(df_base, sim_x, sim_y, sim_z, out_png, title):
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(8.0, 8.0))
    t = df_base["Time"]

    ax[0].plot(t, df_base["X"], label="baseline")
    ax[0].plot(t, sim_x, label="sim")
    ax[0].set_ylabel("X")
    ax[0].legend(loc="best")
    ax[0].grid(True, alpha=0.3)

    ax[1].plot(t, df_base["Y"], label="baseline")
    ax[1].plot(t, sim_y, label="sim")
    ax[1].set_ylabel("Y")
    ax[1].legend(loc="best")
    ax[1].grid(True, alpha=0.3)

    ax[2].plot(t, df_base["Z"], label="baseline")
    ax[2].plot(t, sim_z, label="sim")
    ax[2].set_ylabel("Z")
    ax[2].set_xlabel("Time (s)")
    ax[2].legend(loc="best")
    ax[2].grid(True, alpha=0.3)

    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_png, dpi=160)
    plt.close(fig)

def configure_times_font():
    rcParams["font.family"] = "serif"
    rcParams["font.serif"] = ["Times New Roman", "Times", "Nimbus Roman"]
    rcParams["mathtext.fontset"] = "stix"

def main():
    base_dir = r"E:\workspace\NexDyn-Diff\temps\models\double_pendulum"
    sim_dir = r"E:\workspace\NexDyn-Diff\output\double_pendulum"
    fig_dir = r"E:\workspace\NexDyn-Diff\figures\double_pendulum"
    os.makedirs(fig_dir, exist_ok=True)

    configure_times_font()
    
    files = [
        ("Link1_Pos.csv", "sim_Link1_Pos.csv"),
        ("Link1_Vel.csv", "sim_Link1_Vel.csv"),
        ("Link1_Acc.csv", "sim_Link1_Acc.csv"),
        ("Link2_Pos.csv", "sim_Link2_Pos.csv"),
        ("Link2_Vel.csv", "sim_Link2_Vel.csv"),
        ("Link2_Acc.csv", "sim_Link2_Acc.csv"),
    ]
    
    for base, sim in files:
        result = compare(os.path.join(base_dir, base), os.path.join(sim_dir, sim), base)
        if not result:
            continue
        df_base, df_sim, sim_x, sim_y, sim_z = result
        out_png = os.path.join(fig_dir, os.path.splitext(base)[0] + ".png")
        plot_compare(df_base, sim_x, sim_y, sim_z, out_png, base)

if __name__ == "__main__":
    main()
