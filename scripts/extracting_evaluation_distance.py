#!/usr/bin/env python
import rosbag
import csv
import os


#bag_path = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/test_standard_topics_3.bag')


primary_bag_path = os.path.expanduser('~/bag_files/marlon_experiments_temesvar/day2/day2/otsu_exp2/otsu_exp2.bag')  
secondary_bag_path = os.path.expanduser('~/bag_files/marlon_experiments_temesvar/day2/day2/otsu_exp2/otsu_exp.bag') 


output_csv = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/rw/raw_csv/otsu_exp2_test.csv')

distance_y_list = []

"""
with rosbag.Bag(secondary_bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/uav39/control_manager/control_reference']):
        distance_y_list.append(msg.pose.pose.position.y) """


#Print size of distance_y_list
print(len(distance_y_list))

combined_data = []
index = 0  
with rosbag.Bag(primary_bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/uav38/uvdar/blinkers_seen_left', '/uav38/uvdar/blinkers_seen_right']):
        timestamp = t.to_sec()
        distance_x = distance_y_list[index] if index < len(distance_y_list) else None 
        entry = {'timestamp': timestamp, 'distance_y': distance_x, 'value': None}
        if topic in ['/uav38/uvdar/blinkers_seen_left', '/uav38/uvdar/blinkers_seen_right']:
            entry['value'] = [point.value for point in msg.points]
        combined_data.append(entry)
        index += 1  




""" # List of ROS bag files
bag_files = [
    os.path.expanduser('~/bag_files/marlon_experiments_temesvar/otsu_first/otsu_first.bag'),

    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_8m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_11m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_14m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_17m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_20m.bag'),
]

# Output CSV file
#output_csv = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/standard_test_3.csv')

# Output CSV file
output_csv = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/rw/raw_csv/otsu_exp1.csv')

# Define the topics to extract
topics = ['/uav38/uvdar/blinkers_seen_left','/uav38/uvdar/blinkers_seen_right','/uav38/control_manager/control_reference']


combined_data = []

# Process each bag file
for bag_path in bag_files:
    with rosbag.Bag(bag_path, 'r') as bag:
        data = []  # Temporary list to store data from the current bag
        for topic, msg, t in bag.read_messages(topics=topics):
            entry = {'timestamp': t.to_sec(), 'distance_x': None, 'value': None}
            if topic in ['/uav38/uvdar/blinkers_seen_left', '/uav38/uvdar/blinkers_seen_right']:
                #entry['value'] = msg.points[0].value if msg.points else None
                #Take all the values
                entry['value'] = [point.value for point in msg.points]
            elif topic == '/uav38/control_manager/control_reference':
                # Extract the x position as distance
                entry['distance_x'] = msg.pose.pose.position.x
            data.append(entry)
        combined_data.extend(data)  # Add the data from the current bag to the combined list """

# Sort combined data by timestamp to align the entries
combined_data.sort(key=lambda x: x['timestamp'])

# Write the combined data to a CSV file
with open(output_csv, 'w', newline='') as csvfile:
    fieldnames = ['timestamp', 'distance_y', 'value']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    for entry in combined_data:
        writer.writerow(entry)

print(f"Data extracted and combined from multiple bag files and saved to {output_csv}")