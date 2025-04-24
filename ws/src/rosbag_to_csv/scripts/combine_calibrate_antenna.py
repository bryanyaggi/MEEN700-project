#!/usr/bin/env python3
import os
import sys
import ast  #
import pandas as pd  
import numpy as np  
from scipy.optimize import curve_fit  
import matplotlib.pyplot as plt  

# Compute the project root by moving up four directories from this script
BASE_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..'
))

ANTENNA_COUNT = 4  

DEFAULT_BOUNDS = ([-100, 1], [100, 4]) # Bounds for path-loss fit: P0 ∈ [-100,100] dB, exponent n ∈ [1,4]
BIN_WIDTH = 1.0  # distance bin width for variance estimation

def scenario_folder(scenario):
    """
    Return the path to the CSV folder for a given scenario ('small' or 'large').
    """
    return os.path.join(
        BASE_DIR,
        'ws', 'src', 'rosbag_to_csv', 'csvs', scenario
    )

def load_base_stations(scenario):
    """
    Return hardcoded base-station (x,y) map-frame coordinates for each scenario.
    Raises ValueError if the scenario is unknown.
    """
    sc = scenario.lower()
    if sc == 'small':
        # Small-scenario coordinates
        return {
            41341: (-479.92726658281555, 659.0540357526822),
            41628: (-228.65424791588117, 910.0482297151457),
            41602: (-229.22946276317674, 427.3500316096892)
        }
    elif sc == 'large':
        # Large-scenario coordinates
        return {
            41341: (579.0432653478554, -273.85945461479054),
            41602: (-255.1141308426911, -1175.5095338973592),
            41628: (-518.5625304222979, 175.13634288798463)
        }
    else:
        # Defensive check
        raise ValueError(f"Unknown scenario '{scenario}'")

def path_loss(dist, P0, n):
    """
    Log-distance path-loss model:
      RSSI = P0 - 10 * n * log10(dist)
    """
    return P0 - 10 * n * np.log10(dist)

def combine_single_trial(scenario, trial_id, tol_ms=100):
    """
    Load and merge RSSI, ToF, and pose CSVs for a single trial.
    Returns a DataFrame with columns: time, radioID, rssi_antX, tof, age, map_x, map_y
    """
    folder = scenario_folder(scenario)
    prefix = f"radio-localization_{trial_id}"

    # Filenames
    rssi_p = os.path.join(folder, f"{prefix}-rssi.csv")
    pose_p = os.path.join(folder, f"{prefix}-localization-map_pose.csv")
    tof_p  = os.path.join(folder, f"{prefix}-current_tof.csv")

    # --- RSSI: separate antennas ---
    df_r = pd.read_csv(rssi_p)
    df_r.rename(columns={'Time': 'time'}, inplace=True)
    df_r['time'] = pd.to_datetime(df_r['time'])
    df_r['parsed'] = df_r['.data'].apply(ast.literal_eval)
    df_r['radioID'] = df_r['parsed'].apply(lambda t: t[0])
    for i in range(ANTENNA_COUNT):
        # each antenna's RSSI value
        df_r[f'rssi_ant{i+1}'] = df_r['parsed'].apply(lambda t, j=i: t[j+1])
    df_r = df_r[['time', 'radioID'] + [f'rssi_ant{i+1}' for i in range(ANTENNA_COUNT)]]

    # --- Time-of-Flight: explode triples ---
    df_t = pd.read_csv(tof_p)
    df_t.rename(columns={'Time': 'time'}, inplace=True)
    df_t['time'] = pd.to_datetime(df_t['time'])
    df_t['tuple'] = df_t['.data'].apply(ast.literal_eval)
    recs = []
    for _, row in df_t.iterrows():
        t = row['time']
        tup = row['tuple']
        # iterate in steps of 3: id, tof, age
        for k in range(0, len(tup), 3):
            recs.append({
                'time': t,
                'radioID': tup[k],
                'tof': tup[k+1],
                'age': tup[k+2]
            })
    df_t_long = pd.DataFrame(recs)

    # --- Merge RSSI & ToF on time & radioID ---
    df_rt = pd.merge_asof(
        df_r.sort_values('time'),
        df_t_long.sort_values('time'),
        on='time', by='radioID', direction='nearest',
        tolerance=pd.Timedelta(f'{tol_ms}ms')
    )

    # --- Pose: load map-frame X/Y and merge ---
    df_x = pd.read_csv(pose_p)
    df_x.rename(columns={
        '.pose.pose.position.x': 'map_x',
        '.pose.pose.position.y': 'map_y'
    }, inplace=True)
    df_x['time'] = pd.to_datetime(df_x['time'])
    df_px = df_x[['time', 'map_x', 'map_y']]

    df_m = pd.merge_asof(
        df_rt.sort_values('time'), df_px.sort_values('time'),
        on='time', direction='nearest',
        tolerance=pd.Timedelta(f'{tol_ms}ms')
    )
    # drop missing pose
    return df_m.dropna(subset=['map_x', 'map_y'])

def calibrate_per_antenna(df, bs_map, bounds=DEFAULT_BOUNDS):
    """
    Fit path-loss for each radioID & antenna.
    Returns augmented df and dict of fits {(rid,ant): (P0,n)}.
    """
    # compute distance to each base station
    df['dist'] = np.nan
    for rid, (bx, by) in bs_map.items():
        mask = df['radioID'] == rid
        df.loc[mask, 'dist'] = np.hypot(df.loc[mask, 'map_x'] - bx,
                                        df.loc[mask, 'map_y'] - by)

    fits = {}
    # fit per (rid, antenna)
    for rid in bs_map:
        sub = df[df['radioID'] == rid]
        for i in range(ANTENNA_COUNT):
            col = f'rssi_ant{i+1}'
            valid = sub[col].notna() & (sub['dist'] > 0)
            if not valid.any():
                continue
            popt, _ = curve_fit(path_loss,
                                sub.loc[valid, 'dist'],
                                sub.loc[valid, col],
                                p0=[-30, 2], bounds=bounds)
            fits[(rid, i+1)] = tuple(popt)
            # store calibrated
            df.loc[sub.index, f'{col}_cal'] = path_loss(df.loc[sub.index, 'dist'], *popt)
    return df, fits

def compute_variance_per_antenna(df, bin_width=BIN_WIDTH):
    """
    Compute RSSI variance in distance bins and fit a power-law model var = c*d^k.
    Returns var_df and var_model dict {(rid,ant): (c,k)}.
    """
    # bin distances
    df['dist_bin'] = (df['dist'] // bin_width) * bin_width
    # group and compute sample variance for each antenna
    var_records = []
    for (rid, dist_bin), group in df.groupby(['radioID', 'dist_bin']):
        var_dict = {'radioID': rid, 'dist_bin': dist_bin}
        for i in range(ANTENNA_COUNT):
            col = f'rssi_ant{i+1}'
            var_dict[col] = group[col].var()
        var_records.append(var_dict)
    var_df = pd.DataFrame(var_records)

    # fit power-law for each antenna
    var_model = {}
    for rid in var_df['radioID'].unique():
        for i in range(ANTENNA_COUNT):
            col = f'rssi_ant{i+1}'
            sub = var_df[(var_df['radioID'] == rid) & (var_df[col] > 0)]
            if len(sub) < 2:
                continue
            # log-log fit: log(var) = k*log(d) + log(c)
            coeffs = np.polyfit(np.log(sub['dist_bin']), np.log(sub[col]), 1)
            k, logc = coeffs
            var_model[(rid, i+1)] = (np.exp(logc), k)
    return var_df, var_model

def plot_results(df, fits, bs_map, var_df=None, var_model=None, stddev_scale=1):
    """
    Plot RSSI fits and ±Nσ shading from variance model.
    stddev_scale controls confidence band width (e.g., 1 for ±1σ, 2 for ±2σ).
    """
    plt.ioff()
    fig_list = []

    # per-radio figures
    for rid in sorted(df['radioID'].unique()):
        sub = df[df['radioID'] == rid]
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        axes = axes.flatten()
        for i in range(ANTENNA_COUNT):
            ax = axes[i]
            col = f'rssi_ant{i+1}'
            ax.scatter(sub['dist'], sub[col], s=5, alpha=0.6, label='Raw')
            key = (rid, i+1)
            if key in fits:
                P0, n = fits[key]
                xs = np.linspace(sub['dist'].min(), sub['dist'].max(), 200)
                rssi_fit = path_loss(xs, P0, n)
                ax.plot(xs, rssi_fit, 'r-', lw=2,
                        label=f'Fit P₀={P0:.1f}, n={n:.2f}')
                # ±Nσ shading from variance model
                if var_model and key in var_model:
                    c, k = var_model[key]
                    sigma = stddev_scale * np.sqrt(c * xs**k)
                    ax.fill_between(xs, rssi_fit - sigma, rssi_fit + sigma,
                                    color='gray', alpha=0.3,
                                    label=f'±{stddev_scale}σ')
            ax.set_title(f'Radio {rid}, Antenna {i+1}')
            ax.set_xlabel('Distance [m]')
            ax.set_ylabel('RSSI [dB]')
            ax.legend()
            ax.grid()
        fig.suptitle(f'RSSI vs Distance for Radio {rid}')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig_list.append(fig)

    # map trajectory figure
    fig_map = plt.figure(figsize=(8, 8))
    plt.scatter(df['map_x'], df['map_y'], c='lightgray', s=5, label='Trajectory')

    # get a palette of colors
    colors = plt.cm.tab10.colors  

    for idx, (rid, (bx, by)) in enumerate(bs_map.items()):
        color = colors[idx % len(colors)]
        # plot the base station as a large colored X
        plt.scatter(bx, by, marker='X', s=150, color=color, edgecolors='white')
        # annotate the ID on top, centered, in white
        plt.text(
            bx, by, str(rid),
            fontsize=10, fontweight='bold',
            ha='center', va='center',
            color='black'
        )

    plt.title('Map Trajectory with Base Stations')
    plt.xlabel('Map X')
    plt.ylabel('Map Y')
    plt.legend()
    plt.grid()
    plt.axis('equal')
    fig_list.append(fig_map)

    for f in fig_list:
        f.show()
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) not in [3, 4]:
        print(f"Usage:\n  {sys.argv[0]} OUTPUT.csv [small|large] [TRIAL_ID (optional)]")
        sys.exit(1)

    out_csv = sys.argv[1]
    scenario = sys.argv[2].lower()

    bs_map = load_base_stations(scenario)

    # --- SINGLE TRIAL MODE ---
    if len(sys.argv) == 4:
        trial_id = sys.argv[3]
        df = combine_single_trial(scenario, trial_id)

    # --- MULTI TRIAL MODE (ALL) ---
    else:
        folder = scenario_folder(scenario)
        # find all the raw RSSI files
        filenames = [
            f for f in os.listdir(folder)
            if f.startswith("radio-localization_") and f.endswith("-rssi.csv")
        ]
        # strip off prefix and suffix so trial_id == "2025-04-18-02-25-19"
        all_trials = sorted([
            fname[len("radio-localization_"):-len("-rssi.csv")]
            for fname in filenames
        ])

        print(f"Found {len(all_trials)} trials: {all_trials}")
        dfs = []
        for trial_id in all_trials:
            try:
                print(f"Processing trial {trial_id}...")
                df_trial = combine_single_trial(scenario, trial_id)
                dfs.append(df_trial)
            except Exception as e:
                print(f"Skipping trial {trial_id} due to error: {e}")
        if not dfs:
            print("No valid trials found.")
            sys.exit(1)
        df = pd.concat(dfs, ignore_index=True)

    # Calibration and variance modeling
    df, fits = calibrate_per_antenna(df, bs_map)
    var_df, var_model = compute_variance_per_antenna(df)
    print("Variance model per antenna:", var_model)
    df.to_csv(out_csv, index=False)

    # Plotting
    plot_results(df, fits, bs_map, var_df=var_df, var_model=var_model, stddev_scale=2)
