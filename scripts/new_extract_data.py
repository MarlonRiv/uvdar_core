#!/usr/bin/env python
import rosbag
import pandas as pd
import os
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Extract data from ROS bags and save to CSV.')
parser.add_argument('--name_experiment', type=str, required=True, help='Name of the experiment')
parser.add_argument('--rx_bag', type=str, required=True, help='Path to RX bag file')
parser.add_argument('--tx1_bag', type=str, required=True, help='Path to TX1 bag file')
parser.add_argument('--tx2_bag', type=str, required=True, help='Path to TX2 bag file')
parser.add_argument('--output_csv', type=str, required=True, help='Path to output CSV file')

args = parser.parse_args()

name_experiment = args.name_experiment
output_csv = args.output_csv

# Read blinkers data from uav36 bag
blinkers_msgs = []

with rosbag.Bag(args.rx_bag, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/uav36/uvdar/blinkers_seen_left']):
        timestamp = t.to_sec()
        entry = {
            'timestamp': timestamp,
            'value': [point.value for point in msg.points],
            'point_x': [point.x for point in msg.points],
            'point_y': [point.y for point in msg.points]
        }
        blinkers_msgs.append(entry)

# Convert to DataFrame
blinkers_df = pd.DataFrame(blinkers_msgs)
blinkers_df.sort_values('timestamp', inplace=True)

# Aggregate duplicate timestamps by summing the lists
blinkers_df = blinkers_df.groupby('timestamp').agg({
    'value': 'sum',
    'point_x': 'sum',
    'point_y': 'sum'
}).reset_index()

# Set timestamp as index
blinkers_df.set_index('timestamp', inplace=True)

# Get the first blinkers timestamp
first_blinkers_timestamp = blinkers_df.index.min()

# Function to read odom data, adjust timestamps, and set timestamp as index
def read_odom_data(bag_path, topic_name, prefix, offset):
    msgs = []
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            timestamp = t.to_sec() + offset
            entry = {
                'timestamp': timestamp,
                f'{prefix}_distance_x': msg.pose.pose.position.x,
                f'{prefix}_distance_y': msg.pose.pose.position.y,
                f'{prefix}_distance_z': msg.pose.pose.position.z
            }
            msgs.append(entry)
    df = pd.DataFrame(msgs)
    df.sort_values('timestamp', inplace=True)
    df.set_index('timestamp', inplace=True)
    # Remove duplicate timestamps
    df = df[~df.index.duplicated(keep='first')]
    return df

# Calculate offsets
def get_first_timestamp(bag_path, topic_name):
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            return t.to_sec()
    return None

first_uav36_odom_timestamp = get_first_timestamp(args.rx_bag, '/uav36/estimation_manager/odom_main')
first_uav39_odom_timestamp = get_first_timestamp(args.tx1_bag, '/uav39/estimation_manager/odom_main')
first_uav40_odom_timestamp = get_first_timestamp(args.tx2_bag, '/uav40/estimation_manager/odom_main')

# Calculate offsets
offset_uav36 = first_blinkers_timestamp - first_uav36_odom_timestamp
offset_uav39 = first_blinkers_timestamp - first_uav39_odom_timestamp
offset_uav40 = first_blinkers_timestamp - first_uav40_odom_timestamp

# Read odom data and adjust timestamps
uav36_odom_df = read_odom_data(args.rx_bag, '/uav36/estimation_manager/odom_main', 'uav36', offset_uav36)
uav39_odom_df = read_odom_data(args.tx1_bag, '/uav39/estimation_manager/odom_main', 'uav39', offset_uav39)
uav40_odom_df = read_odom_data(args.tx2_bag, '/uav40/estimation_manager/odom_main', 'uav40', offset_uav40)

# Align position data to blinkers timestamps using reindex with method='nearest'
def align_to_blinkers(blinkers_index, odom_df, tolerance):
    # Reindex odom data to blinkers timestamps
    aligned_df = odom_df.reindex(index=blinkers_index, method='nearest', tolerance=tolerance)
    return aligned_df

tolerance = 0.1  # 100 milliseconds

# Align position data
uav36_aligned = align_to_blinkers(blinkers_df.index, uav36_odom_df, tolerance)
uav39_aligned = align_to_blinkers(blinkers_df.index, uav39_odom_df, tolerance)
uav40_aligned = align_to_blinkers(blinkers_df.index, uav40_odom_df, tolerance)

# Combine all data into one DataFrame
merged_df = pd.concat([blinkers_df, uav36_aligned, uav39_aligned, uav40_aligned], axis=1)

# Drop any rows with NaN values (if positions couldn't be matched within tolerance)
merged_df.dropna(inplace=True)

# Reset index to have timestamp as a column
merged_df.reset_index(inplace=True)

# Save to CSV
merged_df.to_csv(output_csv, index=False)

print(f"Data extracted and combined from multiple bag files and saved to {output_csv}")
