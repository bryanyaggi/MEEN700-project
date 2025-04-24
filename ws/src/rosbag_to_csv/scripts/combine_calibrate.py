#!/usr/bin/env python3
import os
import sys
import ast
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from yaml import safe_load

# base_dir is the MEEN700-project root (four levels up from this file)
BASE_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    '..',    # scripts/
    '..',    # rosbag_to_csv/
    '..',    # src/
    '..'     # ws/
))

def scenario_folder(scenario):
    return os.path.join(
        BASE_DIR,
        'ws','src','rosbag_to_csv','csvs',
        scenario)

def load_base_stations(scenario):
    """
    Hardcoded base-station positions in map frame for each scenario.
    """
    scenario = scenario.lower()
    if scenario == 'small':
        return {
            41341: (-479.92726658281555, 659.0540357526822),
            41628: (-228.65424791588117, 910.0482297151457),
            41602: (-229.22946276317674, 427.3500316096892)
        }
    elif scenario == 'large':
        return {
            41341: (579.0432653478554, -273.85945461479054),
            41602: (-255.1141308426911, -1175.5095338973592),
            41628: (-518.5625304222979, 175.13634288798463)
        }
    else:
        raise ValueError(f"Unknown scenario '{scenario}'; expected 'small' or 'large'.")

def combine_data(csv_paths, tol_ms=100):
    # unchanged combine_data implementation
    rssi_files = sorted(p for p in csv_paths if p.endswith('-rssi.csv'))
    pose_files = sorted(p for p in csv_paths if 'map_pose.csv' in p)
    tof_files  = sorted(p for p in csv_paths if p.endswith('-current_tof.csv'))
    merged_runs = []
    for rssi_p, pose_p, tof_p in zip(rssi_files, pose_files, tof_files):
        # RSSI
        df_r = pd.read_csv(rssi_p); df_r.rename(columns={'Time':'time'}, inplace=True)
        df_r['time'] = pd.to_datetime(df_r['time'])
        df_r['parsed'] = df_r['.data'].apply(ast.literal_eval)
        df_r['radioID'] = df_r['parsed'].apply(lambda t: t[0])
        df_r['rssi'] = df_r['parsed'].apply(lambda t: np.mean(t[1:]))
        df_r = df_r[['time','radioID','rssi']]
        # TOF
        df_t = pd.read_csv(tof_p); df_t.rename(columns={'Time':'time'}, inplace=True)
        df_t['time'] = pd.to_datetime(df_t['time'])
        df_t['tuple'] = df_t['.data'].apply(ast.literal_eval)
        records = []
        for _, row in df_t.iterrows():
            t = row['time']; tup = row['tuple']
            for i in range(0, len(tup), 3):
                records.append({'time':t,'radioID':tup[i],'tof':tup[i+1],'age':tup[i+2]})
        df_t_long = pd.DataFrame(records)
        df_rt = pd.merge_asof(
            df_r.sort_values('time'),
            df_t_long.sort_values('time'),
            on='time', by='radioID', direction='nearest', tolerance=pd.Timedelta(f'{tol_ms}ms')
        )
        # Pose
        df_x = pd.read_csv(pose_p)
        df_x.rename(columns={'.pose.pose.position.x':'map_x','.pose.pose.position.y':'map_y'}, inplace=True)
        df_x['time'] = pd.to_datetime(df_x['time'])
        df_px = df_x[['time','map_x','map_y']]
        df_m = pd.merge_asof(
            df_rt.sort_values('time'), df_px.sort_values('time'),
            on='time', direction='nearest', tolerance=pd.Timedelta(f'{tol_ms}ms')
        )
        merged_runs.append(df_m[['time','radioID','rssi','tof','age','map_x','map_y']])
    df_comb = pd.concat(merged_runs, ignore_index=True)
    df_comb.dropna(subset=['rssi','tof','map_x','map_y'], inplace=True)
    df_comb.sort_values(['radioID','time'], inplace=True)
    return df_comb

def path_loss(dist, P0, n):
    return P0 - 10*n*np.log10(dist)

def calibrate_rssi(df, bs_map, base_power=-30, path_loss_exponent=2, bounds=([-100,1],[100,4])):
    # compute distances
    df['dist'] = np.nan
    for rid,(bx,by) in bs_map.items():
        mask = df['radioID']==rid
        df.loc[mask,'dist'] = np.hypot(df.loc[mask,'map_x']-bx, df.loc[mask,'map_y']-by)
    # fit P0,n per radio
    fits = {}
    for rid in bs_map:
        sub = df[df['radioID']==rid]
        valid = sub['rssi'].notna() & sub['dist']>0
        if not valid.any(): continue
        popt,_ = curve_fit(path_loss, sub.loc[valid,'dist'], sub.loc[valid,'rssi'], p0=[base_power,path_loss_exponent], bounds=bounds)
        fits[rid] = tuple(popt)
        df.loc[sub.index,'rssi_cal'] = path_loss(sub['dist'],*popt)
    return df, fits

def compute_variance(df, bin_width=1.0):
    df['dist_bin'] = (df['dist']//bin_width)*bin_width
    var_df = df.groupby(['radioID','dist_bin'])['rssi'].var().reset_index().rename(columns={'rssi':'rssi_var'})
    # fit variance model var = c * dist^k
    model = {}
    for rid in var_df['radioID'].unique():
        vsub = var_df[(var_df['radioID']==rid)&(var_df['rssi_var']>0)]
        if len(vsub)>1:
            coeffs = np.polyfit(np.log(vsub['dist_bin']), np.log(vsub['rssi_var']),1)
            c = np.exp(coeffs[1]); k = coeffs[0]
            model[rid] = (c,k)
    return var_df, model

def plot_results(df, fits, var_df, bs_map, title):
    radios = sorted(df['radioID'].unique())
    fig, axes = plt.subplots(1, len(radios)+1, figsize=(5*(len(radios)+1),5))
    ax0 = axes[0]
    ax0.scatter(df['map_x'], df['map_y'], c='lightgray', s=5)
    for rid,(bx,by) in bs_map.items(): ax0.scatter(bx,by,marker='X',c='red',s=100); ax0.text(bx,by,str(rid))
    ax0.set_title('Map Trajectory'); ax0.grid(); ax0.axis('equal')

    for i,rid in enumerate(radios,1):
        ax = axes[i]
        sub = df[df['radioID']==rid]
        ax.scatter(sub['dist'], sub['rssi'], s=5, alpha=0.6, label='Raw')
        if rid in fits:
            P0,n = fits[rid]
            xs = np.linspace(sub['dist'].min(),sub['dist'].max(),200)
            ax.plot(xs, path_loss(xs,P0,n), lw=2, label=f'Fit P0={P0:.1f},n={n:.2f}', color='r')
        vsub = var_df[var_df['radioID']==rid]
        sigma = np.sqrt(vsub['rssi_var'])
        mid = path_loss(vsub['dist_bin'],*fits.get(rid,(0,2)))
        ax.fill_between(vsub['dist_bin'], mid-sigma, mid+sigma, alpha=0.3, label='±1σ')
        ax.set_title(f'Radio {rid}'); ax.set_xlabel('Distance [m]'); ax.set_ylabel('RSSI [dB]'); ax.legend(); ax.grid()
    fig.suptitle(title); plt.tight_layout(); plt.show()

if __name__ == '__main__':
    # allow either 3 or 4 arguments:
    #   combine_avg.py OUTPUT.csv small
    #   combine_avg.py OUTPUT.csv small 2025-04-18-03-08-11
    if len(sys.argv) not in (3, 4):
        print(f"Usage:\n  {sys.argv[0]} OUTPUT.csv [small|large] [TRIAL_ID (optional)]")
        sys.exit(1)

    out_csv = sys.argv[1]
    scenario = sys.argv[2].lower()
    folder = scenario_folder(scenario)

    # --- SINGLE TRIAL MODE ---
    if len(sys.argv) == 4:
        trial_id = sys.argv[3]
        # pick only files for that timestamp
        csv_paths = sorted(
            os.path.join(folder, f)
            for f in os.listdir(folder)
            if trial_id in f and any(tok in f for tok in ['-rssi.csv', 'map_pose.csv', '-current_tof.csv'])
        )
        if not csv_paths:
            print(f"No files found for trial '{trial_id}' in scenario '{scenario}'.")
            sys.exit(1)

    # --- MULTI-TRIAL MODE (ALL) ---
    else:
        csv_paths = sorted(
            os.path.join(folder, f)
            for f in os.listdir(folder)
            if any(tok in f for tok in ['-rssi.csv', 'map_pose.csv', '-current_tof.csv'])
        )
        if not csv_paths:
            print(f"No CSVs found in '{folder}'.")
            sys.exit(1)

    # combine, calibrate, variance, plot
    df = combine_data(csv_paths)
    df.to_csv(out_csv, index=False)

    bs_map = load_base_stations(scenario)
    df, fits = calibrate_rssi(df, bs_map)
    var_df, var_model = compute_variance(df)

    print('Variance model per radio:', var_model)
    plot_results(df, fits, var_df, bs_map, title=f"{scenario.capitalize()} Scenario Analysis")
