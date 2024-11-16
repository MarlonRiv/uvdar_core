import pandas as pd
import os
import numpy as np
import ast
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Process CSV data')
parser.add_argument('--name_experiment', type=str, required=True, help='Name of the experiment (without .csv)')
parser.add_argument('--variant', type=int, required=True, help='Variant number (e.g., 1, 2, or 3)')
args = parser.parse_args()

# Experiment name and variant
name_experiment = args.name_experiment  # e.g., 'variant1_th40'
variant = args.variant  # e.g., 1

# Paths
input_csv_path = os.path.expanduser('~/Desktop/MRS_Master_Project/paper/adaptive_variant02_raw_csv/' + name_experiment + '.csv')
cleaned_file_path = os.path.expanduser('~/Desktop/MRS_Master_Project/paper/processed_csv_automation_adaptive/' + name_experiment + '_processed.csv')

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

# Define target positions for each variant
variants = {
    1: {
        'uav36': {'x': -55, 'y': 10, 'z': 5},
        'uav39': {'x': -58, 'y': 15, 'z': 5},
        'uav40': {'x': -53, 'y': 20, 'z': 5}
    },
    2: {
        'uav36': {'x': -55, 'y': 10, 'z': 5},
        'uav39': {'x': -58, 'y': 17, 'z': 5},
        'uav40': {'x': -53, 'y': 24, 'z': 5}
    },
    3: {
        'uav36': {'x': -55, 'y': 10, 'z': 5},
        'uav39': {'x': -58, 'y': 24, 'z': 5},
        'uav40': {'x': -53, 'y': 38, 'z': 5}
    }
}

# Get target positions for the specified variant
try:
    target_positions = variants[variant]
except KeyError:
    raise ValueError(f"Variant {variant} is not defined.")

# Define tolerance (in meters)
position_tolerance = 0.5  # Adjust as needed

# Function to check if UAV is at its target position within tolerance
def is_uav_at_position(uav_x, uav_y, uav_z, target_x, target_y, target_z, tolerance):
    return (abs(uav_x - target_x) <= tolerance and
            abs(uav_y - target_y) <= tolerance and
            abs(uav_z - target_z) <= tolerance)

# Apply the position filter to the data
def position_filter(row):
    uav36_at_pos = is_uav_at_position(
        row['uav36_distance_x'], row['uav36_distance_y'], row['uav36_distance_z'],
        target_positions['uav36']['x'], target_positions['uav36']['y'], target_positions['uav36']['z'],
        position_tolerance
    )
    uav39_at_pos = is_uav_at_position(
        row['uav39_distance_x'], row['uav39_distance_y'], row['uav39_distance_z'],
        target_positions['uav39']['x'], target_positions['uav39']['y'], target_positions['uav39']['z'],
        position_tolerance
    )
    uav40_at_pos = is_uav_at_position(
        row['uav40_distance_x'], row['uav40_distance_y'], row['uav40_distance_z'],
        target_positions['uav40']['x'], target_positions['uav40']['y'], target_positions['uav40']['z'],
        position_tolerance
    )
    return uav36_at_pos and uav39_at_pos and uav40_at_pos

# Apply the filter
data = data[data.apply(position_filter, axis=1)]

# Proceed with the rest of the processing
# Define signal regions and expected values
signals = [
    {
        'signal_name': 'signal_1',
        'expected_value': 1.0,
        'center_x_min': 190,
        'center_x_max': 380
    },
    {
        'signal_name': 'signal_2',
        'expected_value': 2.0,
        'center_x_min': 390,
        'center_x_max': 600
    }
]

# Initialize columns for counts and occurrences
for signal in signals:
    signal_name = signal['signal_name']
    data[f'num_points_{signal_name}'] = 0
    data[f'num_errors_{signal_name}'] = 0
    data[f'{signal_name}_present'] = 0

# Function to process each row
def process_row(row):
    values = row['value']
    point_x = row['point_x']
    point_y = row['point_y']
    
    # Initialize counters for each signal
    counts = {}
    for signal in signals:
        signal_name = signal['signal_name']
        counts[signal_name] = {
            'num_points': 0,
            'num_errors': 0,
            'present': 0
        }
    
    # Process each point
    for v, x, y in zip(values, point_x, point_y):
        for signal in signals:
            signal_name = signal['signal_name']
            expected_value = signal['expected_value']
            center_x_min = signal['center_x_min']
            center_x_max = signal['center_x_max']
            center_y_min = 0.2 * 480  # image_height = 480
            center_y_max = 0.8 * 480

            if center_x_min <= x <= center_x_max and center_y_min <= y <= center_y_max:
                counts[signal_name]['num_points'] += 1
                if v != expected_value:
                    counts[signal_name]['num_errors'] += 1
                else:
                    counts[signal_name]['present'] = 1  # Signal is present
    
    # Assign counts to the row
    for signal in signals:
        signal_name = signal['signal_name']
        row[f'num_points_{signal_name}'] = counts[signal_name]['num_points']
        row[f'num_errors_{signal_name}'] = counts[signal_name]['num_errors']
        row[f'{signal_name}_present'] = counts[signal_name]['present']
    
    return row

# Apply the processing to each row
data = data.apply(process_row, axis=1)

# Remove rows where there are no points for any signal
data = data[(data['num_points_signal_1'] > 0) | (data['num_points_signal_2'] > 0)]

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

# Save processed data
data.to_csv(cleaned_file_path, index=False)
