 #!/usr/bin/env python
import rosbag
import csv
import os


#bag_path = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/test_standard_topics_3.bag')
""" 

primary_bag_path = os.path.expanduser('~/bag_files/marlon_experiments_temesvar/day3/day3/uav_38_standard_exp3/uav38_standard_exp3.bag')  
secondary_bag_path = os.path.expanduser('~/bag_files/marlon_experiments_temesvar/day3/day3/uav39_standard_exp3/uav39_standard_exp3.bag')  """



""" distance_y_list = []

with rosbag.Bag(secondary_bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/uav39/control_manager/control_reference']):
        distance_y_list.append(msg.pose.pose.position.y)


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
        index += 1   """


name_experiment = 'kl_exp1'
name_file_uav38 = 'uav38_'+name_experiment
name_file_uav39 = 'uav39_'+name_experiment

output_csv = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/rw/raw_csv_v2/'+name_experiment+'.csv')


bag_files = [
    #os.path.expanduser('~/bag_files/marlon_experiments_temesvar/day3/day3/'+name_file_uav38+'/'+name_file_uav38+'.bag'),
    os.path.expanduser('~/bag_files/marlon_experiments_temesvar/uav38_kl_first/uav38_kl_first.bag'),
    os.path.expanduser('~/bag_files/marlon_experiments_temesvar/uav39_kl_first/uav39_kl_first.bag'),
    #os.path.expanduser('~/bag_files/marlon_experiments_temesvar/day3/day3/'+name_file_uav39+'/'+name_file_uav39+'.bag'),

    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_8m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_11m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_14m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_17m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_20m.bag'),
]

#output_csv = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/standard_test_3.csv')

#output_csv = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/rw/raw_csv/otsu_exp1.csv')

topics = ['/uav38/uvdar/blinkers_seen_left','/uav38/estimation_manager/odom_main','/uav39/estimation_manager/odom_main']


combined_data = []

for bag_path in bag_files:
    with rosbag.Bag(bag_path, 'r') as bag:
        data = []  
        for topic, msg, t in bag.read_messages(topics=topics):
            entry = {'timestamp': t.to_sec(), 'uav38_distance_y': None, 'uav39_distance_y':None, 'value': None, 'point_x': None, 'point_y': None}
            if topic in ['/uav38/uvdar/blinkers_seen_left']:
                #entry['value'] = msg.points[0].value if msg.points else None
                #Take all the values
                entry['value'] = [point.value for point in msg.points]
                entry['point_x'] = [point.x for point in msg.points]
                entry['point_y'] = [point.y for point in msg.points]
            elif topic == '/uav38/estimation_manager/odom_main':
                entry['uav38_distance_x'] = msg.pose.pose.position.x
                entry['uav38_distance_y'] = msg.pose.pose.position.y
                entry['uav38_distance_z'] = msg.pose.pose.position.z
            elif topic == '/uav39/estimation_manager/odom_main':
                entry['uav39_distance_x'] = msg.pose.pose.position.x
                entry['uav39_distance_y'] = msg.pose.pose.position.y
                entry['uav39_distance_z'] = msg.pose.pose.position.z

            data.append(entry)
        combined_data.extend(data)  

# Sort combined data by timestamp to align the entries
combined_data.sort(key=lambda x: x['timestamp'])

with open(output_csv, 'w', newline='') as csvfile:
    fieldnames = ['timestamp', 'uav38_distance_x', 'uav38_distance_y', 'uav38_distance_z', 'uav39_distance_x', 'uav39_distance_y', 'uav39_distance_z', 'value', 'point_x', 'point_y']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    for entry in combined_data:
        writer.writerow(entry)

print(f"Data extracted and combined from multiple bag files and saved to {output_csv}")