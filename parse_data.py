# %%
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
from pyproj import Transformer
import numpy as np
from scipy.optimize import curve_fit

def latlon_to_utm(latitude, longitude):
    utm_zone = int((longitude + 180) // 6) + 1
    if latitude >= 0:
        utm_crs = f'EPSG:326{utm_zone}' # Northern hemisphere
    else:
        utm_crs = f'EPSG:327{utm_zone}' # Southern hemisphere
        
    transformer = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)
    easting, northing = transformer.transform(longitude, latitude)
    return easting, northing
    
def calculate_distance(easting, northing, start_point):
    dist = np.sqrt((easting-start_point[0])**2 + (northing-start_point[1])**2)
    return dist
    
def dist2rssi(dist, close_rssi, rssiN):
    measurements = close_rssi - 10 * rssiN * np.log10(dist)
    return measurements

# bagfile = 'radio-localization_2025-04-17-00-14-14.bag'
# bagfile = 'radio-localization_2025-04-16-23-35-41.bag'
bagfile = 'radio-localization_2025-04-17-00-14-14.bag'
# bagfile = r"C:\Users\hanco\OneDrive\Documents\Texas A&M\MS Mechanical Engineering\2025 Spring\MEEN 700\Project\MEEN700-project\radio-localization_2025-04-17-00-14-14.bag"

# Read data from bagfile and convert to pandas dataframes
b = bagreader(bagfile)
rssi_data = b.message_by_topic('/rssi')
rssi_df = pd.read_csv(rssi_data)
gps_data = b.message_by_topic("/vectornav/GPS")
gps_df = pd.read_csv(gps_data)

# Convert lat lon to UTM coordinates and add as columns in the gps dataframe
gps_df[['easting', 'northing']] = gps_df.apply(lambda row: pd.Series(latlon_to_utm(row["latitude"], row["longitude"])), axis=1)

# Merge and time sync the rssi and gps data
merged_data = pd.merge_asof(rssi_df, gps_df, on="Time", direction='backward')
min_northing = merged_data['northing'].iloc[1000:4000].min()
# start_point = [merged_data.loc[merged_data['northing'] == min_northing, ['easting']].iloc[0,0], min_northing]
# merged_data['distance'] = merged_data.apply(lambda row: calculate_distance(row['easting'], row['northing'], start_point) + 2, axis = 1)
merged_data['distance'] = merged_data.apply(lambda row: calculate_distance(row['easting'], row['northing'], [merged_data['easting'].iloc[0], merged_data['northing'].iloc[0]]) + 2, axis = 1)
merged_data['rssi'] = merged_data.apply(lambda row: (row['data_1'] + row['data_2'] + row['data_3'] + row['data_4'])/4, axis = 1)

print(merged_data[['latitude', 'longitude', 'easting', 'northing', 'distance']].head())

merged_data.to_csv('merged_data.csv', index=False)

merged_data = merged_data.bfill()

start_ndx = merged_data[merged_data['distance'] == 2.0].index
print(merged_data['distance'])

# %%

# Plot UTM
plt.figure()
# plt.plot(merged_data['easting'].iloc[start_ndx[0]:], merged_data['northing'].iloc[start_ndx[0]:])
plt.plot(merged_data['easting'], merged_data['northing'])
# plt.scatter(start_point[0], start_point[1], s=100, c='r')
plt.scatter(merged_data['easting'][[0,3900]], merged_data['northing'][[0,3900]], s=100, c='g')
plt.grid()
plt.axis('equal')

# Plot rssi over time
plt.figure()
plt.title("RSSI vs Time")
plt.plot(rssi_df.loc[rssi_df['data_0'] == 41602]['Time'], rssi_df.loc[rssi_df['data_0'] == 41602]['data_1'])
plt.plot(rssi_df.loc[rssi_df['data_0'] == 41628]['Time'], rssi_df.loc[rssi_df['data_0'] == 41628]['data_1'])
plt.plot(rssi_df.loc[rssi_df['data_0'] == 41341]['Time'], rssi_df.loc[rssi_df['data_0'] == 41341]['data_1'])
plt.show()

# Plot distance over time
plt.figure()
plt.title("Distance vs Time")
plt.plot(merged_data['Time'],merged_data['distance'])
plt.show()


#  %% Plot rssi over distance
plt.figure(figsize=(12, 7))
# plt.plot(merged_data.iloc[start_ndx[0]:].loc[merged_data['data_0'] == 41602]['distance'].loc[(merged_data['distance'] > 25) & (merged_data['distance'] < 400)], merged_data.iloc[start_ndx[0]:].loc[merged_data['data_0'] == 41602]['rssi'].loc[(merged_data['distance'] > 25) & (merged_data['distance'] < 400)], label='Radio 41602')
# plt.plot(merged_data.iloc[start_ndx[0]:].loc[merged_data['data_0'] == 41628]['distance'], merged_data.iloc[start_ndx[0]:].loc[merged_data['data_0'] == 41628]['rssi'], label='Radio 41628')
# plt.plot(merged_data.iloc[start_ndx[0]:].loc[merged_data['data_0'] == 41341]['distance'], merged_data.iloc[start_ndx[0]:].loc[merged_data['data_0'] == 41341]['rssi'], label='Radio 41341')
radio_list = [41602, 41628, 41341]
line_colors = ['tab:blue', 'tab:orange', 'tab:green']
for ii in range(3):
    distance = merged_data.iloc[0:3900].loc[merged_data['data_0'] == radio_list[ii]]['distance'].loc[(merged_data['distance'] > 25) & (merged_data['distance'] < 400)]
    rssi = merged_data.iloc[0:3900].loc[merged_data['data_0'] == radio_list[ii]]['rssi'].loc[(merged_data['distance'] > 25) & (merged_data['distance'] < 400)]
    popt, _ = curve_fit(dist2rssi, distance, rssi, p0 = [-30, 3])
    optimized_power, optimized_N = popt
    rssi_hat = dist2rssi(distance, *(popt))
    plt.plot(distance, rssi, line_colors[ii], linestyle='-', label=f'Radio {radio_list[ii]}')    
    plt.plot(distance, rssi_hat, line_colors[ii], linestyle='--', label=f'Fitted Curve (radio {radio_list[ii]}) baseStationTXpwr={optimized_power:.2f}, rssiN={optimized_N:.2f}')
plt.xlabel("Distance [m]")
plt.ylabel("RSSI [dB]")
plt.legend()
plt.show()
# %%
