import bagpy
from bagpy import bagreader
import pandas as pd

bagfile = 'radio-localization_2025-04-17-00-14-14.bag'

b = bagreader(bagfile)

topic_name = '/rssi'
data = b.message_by_topic(topic_name)
df = pd.read_csv(data)

print(df.head())