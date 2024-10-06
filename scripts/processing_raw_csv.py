import pandas as pd
import os
import numpy as np
import ast
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Process CSV data')
parser.add_argument('--name_experiment', type=str, required=True, help='Name of the experiment (without .csv)')
parser.add_argument('--subset_rx', type=str, required=True, help='Subset name (e.g., rx1_s1)')
parser.add_argument('--center_x_min', type=float, required=True, help='Minimum x value for center region')
parser.add_argument('--center_x_max', type=float, required=True, help='Maximum x value for center region')
args = parser.parse_args()

# Experiment name
name_experiment = args.name_experiment  # e.g., 'variant1_th40'
subset_rx = args.subset_rx  # e.g., 'rx1_s1'

# Paths
input_csv_path = os.path.expanduser('~/Desktop/MRS_Master_Project/paper/new_raw_csv/' + name_experiment + '.csv')
cleaned_file_path = os.path.expanduser('~/Desktop/MRS_Master_Project/paper/processed_csv_automation/' + name_experiment + '_'+ subset_rx +'_processed.csv')

# Read data
data = pd.read_csv(input_csv_path)

# Convert 'value', 'point_x', 'point_y' from string representation of lists to actual lists
data['value'] = data['value'].apply(lambda x: ast.literal_eval(x) if isinstance(x, str) else [])
data['point_x'] = data['point_x'].apply(lambda x: ast.literal_eval(x) if isinstance(x, str) else [])
data['point_y'] = data['point_y'].apply(lambda x: ast.literal_eval(x) if isinstance(x, str) else [])

# Ensure the fields are lists
def ensure_list(x):
    return x if isinstance(x, list) else []

data['value'] = data['value'].apply(ensure_list)
data['point_x'] = data['point_x'].apply(ensure_list)
data['point_y'] = data['point_y'].apply(ensure_list)

# Define center area of the image
image_height, image_width = 480, 752
center_x_min, center_x_max = args.center_x_min, args.center_x_max
center_y_min, center_y_max = 0.2 * image_height, 0.8 * image_height

# Function to filter points in the center of the image
def filter_center_points(values, point_x, point_y):
    filtered_values, filtered_point_x, filtered_point_y = [], [], []
    for v, x, y in zip(values, point_x, point_y):
        if center_x_min <= x <= center_x_max and center_y_min <= y <= center_y_max:
            filtered_values.append(v)
            filtered_point_x.append(x)
            filtered_point_y.append(y)
    return filtered_values, filtered_point_x, filtered_point_y

# Apply filtering to each row
filtered_results = data.apply(
    lambda row: filter_center_points(row['value'], row['point_x'], row['point_y']), axis=1
)

# Update the DataFrame with filtered results
data['value'] = filtered_results.apply(lambda x: x[0])
data['point_x'] = filtered_results.apply(lambda x: x[1])
data['point_y'] = filtered_results.apply(lambda x: x[2])

# Remove rows where 'value' is empty after filtering
data = data[data['value'].str.len() > 0]

# Check for missing values in UAV distance fields
uav_fields = [
    'uav36_distance_x', 'uav36_distance_y', 'uav36_distance_z',
    'uav39_distance_x', 'uav39_distance_y', 'uav39_distance_z',
    'uav40_distance_x', 'uav40_distance_y', 'uav40_distance_z'
]

# Drop rows with missing values in UAV fields
data.dropna(subset=uav_fields, inplace=True)

# Calculate relative distance_y
data['uav39_relative_distance_y'] = np.abs(data['uav39_distance_y'] - data['uav36_distance_y'])
data['uav40_relative_distance_y'] = np.abs(data['uav40_distance_y'] - data['uav36_distance_y'])

# Calculate number of points
data['num_points'] = data['value'].apply(len)

# Error rate calculation
def calculate_error_rate(values):
    return sum(1 for value in values if value not in [1.0, 2.0])

data['error_count'] = data['value'].apply(calculate_error_rate)

# Signal occurrence functions
def check_signal_2(values):
    return 1 if 2.0 in values else 0

def check_signal_1(values):
    return 1 if 1.0 in values else 0

# Apply signal checks
data['signal_2'] = data['value'].apply(check_signal_2)
data['signal_1'] = data['value'].apply(check_signal_1)

# Save processed data
data.to_csv(cleaned_file_path, index=False)
