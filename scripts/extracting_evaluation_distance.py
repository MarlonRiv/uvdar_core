#!/usr/bin/env python
import rosbag
import csv
import os

name_experiment = 'variant1_th60'
# name_file_uav38 = 'uav38_'+name_experiment
# name_file_uav39 = 'uav39_'+name_experiment

output_csv = os.path.expanduser('~/Desktop/MRS_Master_Project/paper/raw_csv/'+name_experiment+'.csv')

bag_files = [
    #os.path.expanduser('~/bag_files/marlon_experiments_temesvar/day3/day3/'+name_file_uav38+'/'+name_file_uav38+'.bag'),
    #RX
    os.path.expanduser('~/bag_files/paper_rosbags/new_launchfile/variant01_v2_th60.bag'),
    #TX1
    os.path.expanduser('~/bag_files/datasets_08_2024/uav39_rosbags/76_2024_08_20_14_02_27variant1/_2024-08-20-14-14-09.bag'),
    #TX2
    os.path.expanduser('~/bag_files/datasets_08_2024/uav40_rosbags_tx/37_2024_08_20_14_02_31variant1/_2024-08-20-14-14-10.bag'),
]

topics = ['/uav36/uvdar/blinkers_seen_left','/uav36/estimation_manager/odom_main','/uav39/estimation_manager/odom_main', '/uav40/estimation_manager/odom_main']


combined_data = []

for bag_path in bag_files:
    with rosbag.Bag(bag_path, 'r') as bag:
        data = []  
        for topic, msg, t in bag.read_messages(topics=topics):
            entry = {'timestamp': t.to_sec(), 'uav36_distance_x': None, 'uav36_distance_y':None, 'uav36_distance_z':None,
                                              'uav39_distance_x': None, 'uav39_distance_y':None, 'uav39_distance_z':None,
                                              'uav40_distance_x': None, 'uav40_distance_y':None, 'uav40_distance_z':None,
                                              'value': None, 'point_x': None, 'point_y': None}
            if topic in ['/uav36/uvdar/blinkers_seen_left']:
                #entry['value'] = msg.points[0].value if msg.points else None
                #Take all the values
                entry['value'] = [point.value for point in msg.points]
                entry['point_x'] = [point.x for point in msg.points]
                entry['point_y'] = [point.y for point in msg.points]
            elif topic == '/uav36/estimation_manager/odom_main':
                entry['uav36_distance_x'] = msg.pose.pose.position.x
                entry['uav36_distance_y'] = msg.pose.pose.position.y
                entry['uav36_distance_z'] = msg.pose.pose.position.z
            elif topic == '/uav39/estimation_manager/odom_main':
                entry['uav39_distance_x'] = msg.pose.pose.position.x
                entry['uav39_distance_y'] = msg.pose.pose.position.y
                entry['uav39_distance_z'] = msg.pose.pose.position.z
            elif topic == '/uav40/estimation_manager/odom_main':
                entry['uav40_distance_x'] = msg.pose.pose.position.x
                entry['uav40_distance_y'] = msg.pose.pose.position.y
                entry['uav40_distance_z'] = msg.pose.pose.position.z

            data.append(entry)
        combined_data.extend(data)  

# Sort combined data by timestamp to align the entries
combined_data.sort(key=lambda x: x['timestamp'])

with open(output_csv, 'w', newline='') as csvfile:
    fieldnames = ['timestamp', 'uav36_distance_x', 'uav36_distance_y', 'uav36_distance_z', 'uav39_distance_x', 'uav39_distance_y', 'uav39_distance_z', 'uav40_distance_x', 'uav40_distance_y', 'uav40_distance_z', 'value', 'point_x', 'point_y']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    for entry in combined_data:
        writer.writerow(entry)

print(f"Data extracted and combined from multiple bag files and saved to {output_csv}")
