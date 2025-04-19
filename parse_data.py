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
bagfile = 'radio-localization_2025-04-16-23-35-41.bag'

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
start_point = [merged_data.loc[merged_data['northing'] == min_northing, ['easting']].iloc[0,0], min_northing]
merged_data['distance'] = merged_data.apply(lambda row: calculate_distance(row['easting'], row['northing'], start_point) + 1, axis = 1)
merged_data['rssi'] = merged_data.apply(lambda row: (row['data_1'] + row['data_2'] + row['data_3'] + row['data_4'])/4, axis = 1)

print(merged_data[['latitude', 'longitude', 'easting', 'northing', 'distance']].head())

merged_data.to_csv('merged_data.csv', index=False)

merged_data = merged_data.bfill()
# Use nonlinear least squares to find rssiN and 1-m power
distance = merged_data.loc[merged_data['data_0'] == 41602]['distance'].loc[merged_data['northing'].diff() > 0]
# distance = np.linspace(merged_data['distance'].min(),merged_data['distance'].max(), len(merged_data))
rssi = merged_data.loc[merged_data['data_0'] == 41602]['rssi'].loc[merged_data['northing'].diff() > 0]
popt, _ = curve_fit(dist2rssi, distance, rssi, p0 = [-30, 3])
optimized_power, optimized_N = popt


# Plot UTM
plt.figure()
plt.plot(merged_data['easting'].loc[merged_data['northing'].diff() > 0], merged_data['northing'].loc[merged_data['northing'].diff() > 0])
plt.scatter(start_point[0], start_point[1], s=100, c='r')
plt.axis('equal')

# Plot rssi over time
plt.figure()
plt.plot(rssi_df.loc[rssi_df['data_0'] == 41602]['Time'], rssi_df.loc[rssi_df['data_0'] == 41602]['data_1'])
plt.plot(rssi_df.loc[rssi_df['data_0'] == 41628]['Time'], rssi_df.loc[rssi_df['data_0'] == 41628]['data_1'])
plt.plot(rssi_df.loc[rssi_df['data_0'] == 41341]['Time'], rssi_df.loc[rssi_df['data_0'] == 41341]['data_1'])
# plt.show()

# Plot rssi over distance
rssi_hat = dist2rssi(merged_data.loc[merged_data['data_0'] == 41602]['distance'].loc[merged_data['northing'].diff() > 0], *(popt))
plt.figure()
plt.plot(merged_data.loc[merged_data['data_0'] == 41602]['distance'].loc[merged_data['northing'].diff() > 0], merged_data.loc[merged_data['data_0'] == 41602]['rssi'].loc[merged_data['northing'].diff() > 0], label='Radio 41602')
plt.plot(merged_data.loc[merged_data['data_0'] == 41628]['distance'].loc[merged_data['northing'].diff() > 0], merged_data.loc[merged_data['data_0'] == 41628]['rssi'].loc[merged_data['northing'].diff() > 0], label='Radio 41628')
plt.plot(merged_data.loc[merged_data['data_0'] == 41341]['distance'].loc[merged_data['northing'].diff() > 0], merged_data.loc[merged_data['data_0'] == 41341]['rssi'].loc[merged_data['northing'].diff() > 0], label='Radio 41341')
plt.plot(merged_data.loc[merged_data['data_0'] == 41602]['distance'].loc[merged_data['northing'].diff() > 0], rssi_hat, color='r', label=f'Fitted Curve (radio 41602) baseStationTXpwr={optimized_power:.2f}, rssiN={optimized_N:.2f}')
plt.xlabel("Distance [m]")
plt.ylabel("RSSI [dB]")
plt.legend()
plt.show()