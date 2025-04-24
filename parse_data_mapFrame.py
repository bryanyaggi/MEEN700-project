from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import os

os.system('cls')

scenario = 'Large'

if scenario == 'Large':
    bagfile1 = r"C:\Users\hanco\OneDrive\Documents\Texas A&M\MS Mechanical Engineering\2025 Spring\MEEN 700\Project\data\radio-localization_2025-04-18-03-08-11.bag" # large_scenario
    bagfile2 = r"C:\Users\hanco\OneDrive\Documents\Texas A&M\MS Mechanical Engineering\2025 Spring\MEEN 700\Project\data\radio-localization_2025-04-18-03-17-32.bag" # large_scenario
    bagfiles = [bagfile1, bagfile2]
    yamlfile = 'large_scenario_basestations.yaml'
elif scenario == 'Small':
    bagfile1 = r"C:\Users\hanco\OneDrive\Documents\Texas A&M\MS Mechanical Engineering\2025 Spring\MEEN 700\Project\data\radio-localization_2025-04-18-02-25-19.bag" # small_scenario
    bagfile2 = r"C:\Users\hanco\OneDrive\Documents\Texas A&M\MS Mechanical Engineering\2025 Spring\MEEN 700\Project\data\radio-localization_2025-04-18-02-29-18.bag" # small_scenario
    bagfiles = [bagfile1, bagfile2]
    yamlfile = 'small_scenario_basestations.yaml'

import numpy as np
from scipy.optimize import curve_fit
from yaml import safe_load
    
def calculate_distance(x, y, start_point):
    dist = np.sqrt((x-start_point[0])**2 + (y-start_point[1])**2)
    return dist
    
def dist2rssi(dist, close_rssi, rssiN):
    measurements = close_rssi - 10 * rssiN * np.log10(dist)
    return measurements

def getBaseStationPoints(yamlFile):
    with open(yamlFile, 'r') as file:
        config_data = safe_load(file)

    base_stations = np.zeros((2, len(config_data['markers'])))
    for ii, base_station in enumerate(config_data['markers']):
        id = base_station['id']
        x = base_station['pose']['position']['x']
        y = base_station['pose']['position']['y']
        base_stations[0,ii] = x
        base_stations[1,ii] = y
        # print(f'radio: {id}')
        # print(f'   x: {x}')
        # print(f'   y: {y}')
        # print()

    return config_data, base_stations

def MSE(yhat, ydata):
    if len(yhat) != len(ydata):
        raise Warning('yhat and ydata must be of the same length')
    return np.sum((ydata - yhat)**2)/len(yhat)

print("Getting base station positions...")
# config_data, baseStations = getBaseStationPoints('large_scenario_basestations.yaml')
config_data, baseStations = getBaseStationPoints(yamlfile)

# bagfile = 'radio-localization_2025-04-17-00-14-14.bag'
# bagfile = 'radio-localization_2025-04-16-23-35-41.bag'
# bagfile = 'radio-localization_2025-04-17-00-14-14.bag'
# bagfile = r"C:\Users\hanco\OneDrive\Documents\Texas A&M\MS Mechanical Engineering\2025 Spring\MEEN 700\Project\data\radio-localization_2025-04-18-02-25-19.bag" # small_scenario
# bagfile = r"C:\Users\hanco\OneDrive\Documents\Texas A&M\MS Mechanical Engineering\2025 Spring\MEEN 700\Project\data\radio-localization_2025-04-18-03-08-11.bag" # large_scenario
# bagfile = r"C:\Users\hanco\OneDrive\Documents\Texas A&M\MS Mechanical Engineering\2025 Spring\MEEN 700\Project\MEEN700-project\radio-localization_2025-04-17-00-14-14.bag"

print("Reading bagfile...")
# Read data from bagfile and convert to pandas dataframes
xy_df = pd.DataFrame()
rssi_df = pd.DataFrame()
merged_data = pd.DataFrame()

for bagfile in bagfiles:
    b = bagreader(bagfile)
    rssi_data = b.message_by_topic('/rssi')
    rssi_df1 = pd.read_csv(rssi_data)[['Time', 'data_0', 'data_1', 'data_2', 'data_3', 'data_4']]
    xy_data = b.message_by_topic("/localization/map_pose")
    xy_df1 = pd.read_csv(xy_data)[['Time', 'pose.pose.position.x', 'pose.pose.position.y']]
    rssi_df1.to_csv('rssi_df1.csv', index=False)
    xy_df1.to_csv('xy_df1.csv', index=False)

    print("Formatting data...")
    # Merge and time sync the rssi and gps data
    merged_data1 = pd.merge_asof(rssi_df1, xy_df1, on="Time", direction='nearest',tolerance=0.01)
    # print(f'{len(merged_data1)=}')
    for ii, radio in enumerate(config_data['markers']):
        merged_data1[f'distance_from_{radio['id']}'] = merged_data1.apply(lambda row: calculate_distance(row['pose.pose.position.x'], row['pose.pose.position.y'], [baseStations[0,ii], baseStations[1,ii]]), axis = 1)
    merged_data1['rssi'] = merged_data1.apply(lambda row: (row['data_1'] + row['data_2'] + row['data_3'] + row['data_4'])/4, axis = 1)

    # merged_data1 = merged_data1.bfill()
    # merged_data1 = merged_data1.ffill()
    print(f'{len(merged_data1)}')
    merged_data1 = merged_data1.dropna()
    print(f'{len(merged_data1)} after dropping na')

    xy_df = pd.concat([xy_df, xy_df1], ignore_index=True)
    rssi_df = pd.concat([rssi_df, rssi_df1], ignore_index=True)
    merged_data = pd.concat([merged_data, merged_data1], ignore_index=True)

merged_data.to_csv('merged_data.csv', index=False)

distance_data = merged_data[['distance_from_41341', 'distance_from_41628', 'distance_from_41602', 'data_0', 'rssi']]
radio_41341_df = distance_data.loc[distance_data['data_0'] == 41341]
radio_41628_df = distance_data.loc[distance_data['data_0'] == 41628]
radio_41602_df = distance_data.loc[distance_data['data_0'] == 41602]
# print(radio_41341_df.head())
# print(radio_41628_df.head())
# print(radio_41602_df.head())
# print(f'{len(radio_41341_df)=}')
# print(f'{len(radio_41628_df)=}')
# print(f'{len(radio_41602_df)=}')


print("Plotting...")
## Plotting

# Plot UTM
plt.figure()
# plt.plot(merged_data['easting'].iloc[start_ndx[0]:], merged_data['northing'].iloc[start_ndx[0]:])
plt.plot(merged_data['pose.pose.position.x'], merged_data['pose.pose.position.y'])
# plt.scatter(start_point[0], start_point[1], s=100, c='r')
plt.scatter(baseStations[0,:], baseStations[1,:], s=100, c='g')
# plt.text(baseStations[0,:], baseStations[1,:], [f'{config_data['base_stations'][ii]['id']}' for ii in range(len(config_data['base_stations']))])
for ii, radio in enumerate(config_data['markers']):
    plt.text(baseStations[0,ii], baseStations[1,ii], f'{radio['id']}')
plt.grid()
plt.axis('equal')

# Plot rssi over distance
plt.figure(figsize=(12, 7))
radio_list = [41341, 41628, 41602]
line_colors = ['tab:blue', 'tab:orange', 'tab:green']
for ii in range(3):
    distance = distance_data.sort_values(by=f'distance_from_{radio_list[ii]}').loc[distance_data['data_0'] == radio_list[ii]][f'distance_from_{radio_list[ii]}']
    rssi = distance_data.sort_values(by=f'distance_from_{radio_list[ii]}').loc[distance_data['data_0'] == radio_list[ii]]['rssi']
    print(f'{ii=}')
    print(f'{rssi=}')
    popt, pcov = curve_fit(dist2rssi, distance, rssi, p0 = [-30, 3])
    optimized_power, optimized_N = popt
    rssi_hat = dist2rssi(distance, *(popt))
    meanSquaredError = MSE(rssi_hat,rssi)
    plt.plot(distance, rssi, line_colors[ii], linestyle='-', label=f'Radio {radio_list[ii]}')    
    plt.plot(distance, rssi_hat, line_colors[ii], linestyle='--', marker='o', ms=5, mec='r', label=f'Fitted Curve (radio {radio_list[ii]}) A = {optimized_power:.2f}, N = {optimized_N:.2f}, MSE = {meanSquaredError:.2f}')
    print(f'{radio_list[ii]} MSE: {meanSquaredError=}')
    # print(f'{pcov=}')
plt.title(f"High Power Calibration - {scenario} Scenario, {len(bagfiles)} total runs")
plt.xlabel("Distance [m]")
plt.ylabel("RSSI [dB]")
plt.legend()
plt.show()