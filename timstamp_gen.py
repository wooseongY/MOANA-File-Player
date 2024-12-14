import os
import glob
import csv

base_dir = 'Your_path_to_each_sequence'

sensor_dir = base_dir + '/sensor_data'
all_data = []


for sensor_name in os.listdir(sensor_dir):
    sensor_path = os.path.join(sensor_dir, sensor_name)
    if os.path.isdir(sensor_path):

        for ext in ['*.png', '*.jpg', '*.bin']:
            for file_path in glob.glob(os.path.join(sensor_path, ext)):
                fname = os.path.splitext(os.path.basename(file_path))[0]

                timestamp = int(fname)
                all_data.append((timestamp, sensor_name))


all_data.sort(key=lambda x: x[0])


with open(base_dir+'/data_stamp.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(all_data)

