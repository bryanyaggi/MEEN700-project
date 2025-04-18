import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
from pyproj import Transformer
import numpy as np

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

# bagfile = 'radio-localization_2025-04-17-00-14-14.bag'
bagfile = 'radio-localization_2025-04-16-23-35-41.bag'

b = bagreader(bagfile)

rssi_data = b.message_by_topic('/rssi')
rssi_df = pd.read_csv(rssi_data)
gps_data = b.message_by_topic("/vectornav/GPS")
gps_df = pd.read_csv(gps_data)

gps_df[['easting', 'northing']] = gps_df.apply(lambda row: pd.Series(latlon_to_utm(row["latitude"], row["longitude"])), axis=1)

merged_data = pd.merge_asof(gps_df, rssi_df, on="Time", direction='backward')
min_northing = merged_data['northing'].iloc[1000:4000].min()
start_point = [merged_data.loc[merged_data['northing'] == min_northing, ['easting']].iloc[0,0], min_northing]
merged_data['distance'] = merged_data.apply(lambda row: calculate_distance(row['easting'], row['northing'], start_point), axis = 1)
print(merged_data[['latitude', 'longitude', 'easting', 'northing', 'distance']].head())



plt.figure()
plt.plot(merged_data['easting'].iloc[1000:4000], merged_data['northing'].iloc[1000:4000])
plt.scatter(start_point[0], start_point[1], s=100, c='r')
plt.axis('equal')

plt.figure()
plt.plot(rssi_df.loc[rssi_df['data_0'] == 41602]['Time'], rssi_df.loc[rssi_df['data_0'] == 41602]['data_1'])
plt.plot(rssi_df.loc[rssi_df['data_0'] == 41628]['Time'], rssi_df.loc[rssi_df['data_0'] == 41628]['data_1'])
plt.plot(rssi_df.loc[rssi_df['data_0'] == 41341]['Time'], rssi_df.loc[rssi_df['data_0'] == 41341]['data_1'])
plt.show()

plt.figure()
plt.plot(merged_data.loc[merged_data['data_0'] == 41602]['distance'], merged_data.loc[merged_data['data_0'] == 41602]['data_1'], label='Radio 1')
plt.plot(merged_data.loc[merged_data['data_0'] == 41628]['distance'], merged_data.loc[merged_data['data_0'] == 41628]['data_1'], label='Radio 2')
plt.plot(merged_data.loc[merged_data['data_0'] == 41341]['distance'], merged_data.loc[merged_data['data_0'] == 41341]['data_1'], label='Radio 3')
plt.xlabel("Distance [m]")
plt.ylabel("RSSI [dB]")
plt.legend()
plt.show()