#!/usr/bin/env python

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# # Define the file paths and thresholds

# #{ Variant 01-03 Static

#Variant01
# # Define the file paths and thresholds
# file_paths = [
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant01_v2_th40_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant01_v2_th60_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant01_v2_th80_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant01_v2_th100_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant01_v2_th120_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant01_v2_th160_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant01_v2_th200_processed.csv',
# ]

#Variant02
# # Define the file paths and thresholds
# file_paths = [
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant02_v2_th40_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant02_v2_th60_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant02_v2_th80_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant02_v2_th100_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant02_v2_th120_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant02_v2_th160_processed.csv',
#     '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant02_v2_th200_processed.csv',
# ]

##Variant03
## Define the file paths and thresholds
#file_paths = [
#    '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant03_v2_th40_processed.csv',
#    '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant03_v2_th60_processed.csv',
#    '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant03_v2_th80_processed.csv',
#    '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant03_v2_th100_processed.csv',
#    '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant03_v2_th120_processed.csv',
#    '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant03_v2_th160_processed.csv',
#    '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_v2/variant03_v2_th200_processed.csv',
#]

# #} end of Variant 01-03 Static

filename = 'threshold_analysis_summary_dynamic_v2.csv'
file_paths = [
    '/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_dynamic_v2/motiv_dynamic_th100_processed.csv'
    ,'/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_dynamic_v2/motiv_dynamic_th120_processed.csv'
    ,'/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_dynamic_v2/motiv_dynamic_th160_processed.csv'
    ,'/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_dynamic_v2/motiv_dynamic_th200_processed.csv'
    ,'/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_dynamic_v2/motiv_dynamic_th40_processed.csv'
    ,'/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_dynamic_v2/motiv_dynamic_th60_processed.csv'
    ,'/home/rivermar/Desktop/MRS_Master_Project/paper/processed_csv_automation_dynamic_v2/motiv_dynamic_th80_processed.csv'
]
keys = [
    'motiv_dynamic_th100_processed'
    ,'motiv_dynamic_th120_processed'
    ,'motiv_dynamic_th160_processed'
    ,'motiv_dynamic_th200_processed'
    ,'motiv_dynamic_th40_processed'
    ,'motiv_dynamic_th60_processed'
    ,'motiv_dynamic_th80_processed'
]

thresholds = [40, 60, 80, 100, 120, 160, 200]

# Initialize a dataframe to hold the summarized data
summary_data = {
    'Key': [],
    # 'Threshold': [],
    'Total Errors Signal 1': [],
    'Total Errors Signal 2': [],
    'Total Instances Signal 1': [],
    'Total Instances Signal 2': [],
    'Error Rate Signal 1': [],
    'Error Rate Signal 2': [],
    'Percentage Fully Present Signal 1': [],
    'Percentage Fully Present Signal 2': [],
    'Decoding Success Rate Signal 1': [],
    'Decoding Success Rate Signal 2': [],
    'Average Decoding Quality Signal 1': [],
    'Average Decoding Quality Signal 2': [],
    'Combined Error Rate': [],
    'Max Error Rate': [],
    'Error Rate Difference (S2 - S1)': [],
    'Relative Error Rate Difference (%)': []
}

# Loop through the files and calculate the required metrics
for path, key in zip(file_paths, keys):
# for path, threshold in zip(file_paths, thresholds):
    # Load the data
    df = pd.read_csv(path)
    
    # Calculate total errors for each signal
    total_errors_signal_1 = df['num_errors_signal_1'].sum()
    total_errors_signal_2 = df['num_errors_signal_2'].sum()

    # Calculate total instances signal present
    total_signal_1 = df['num_points_signal_1'].sum()
    total_signal_2 = df['num_points_signal_2'].sum()

    # Calculate error rates
    error_rate_signal_1 = total_errors_signal_1 / total_signal_1 if total_signal_1 != 0 else 0
    error_rate_signal_2 = total_errors_signal_2 / total_signal_2 if total_signal_2 != 0 else 0

    # Calculate percentage fully present based on number of errors
    total_entries = len(df)
    fully_present_signal_1 = (df['num_errors_signal_1'] == 0).sum() / total_entries * 100
    fully_present_signal_2 = (df['num_errors_signal_2'] == 0).sum() / total_entries * 100

    # Calculate decoding success rate
    success_rate_signal_1 = ((total_signal_1 - total_errors_signal_1) / total_signal_1) * 100 if total_signal_1 != 0 else 0
    success_rate_signal_2 = ((total_signal_2 - total_errors_signal_2) / total_signal_2) * 100 if total_signal_2 != 0 else 0

    # Calculate decoding quality for each row
    decoding_quality_signal_1 = ((df['num_points_signal_1'] - df['num_errors_signal_1']).clip(0, 3) / 3).mean() * 100
    decoding_quality_signal_2 = ((df['num_points_signal_2'] - df['num_errors_signal_2']).clip(0, 3) / 3).mean() * 100

    # Append the data to the summary
    summary_data['Key'].append(key)
    # summary_data['Threshold'].append(threshold)
    summary_data['Total Errors Signal 1'].append(total_errors_signal_1)
    summary_data['Total Errors Signal 2'].append(total_errors_signal_2)
    summary_data['Total Instances Signal 1'].append(total_signal_1)
    summary_data['Total Instances Signal 2'].append(total_signal_2)
    summary_data['Error Rate Signal 1'].append(error_rate_signal_1)
    summary_data['Error Rate Signal 2'].append(error_rate_signal_2)
    summary_data['Percentage Fully Present Signal 1'].append(fully_present_signal_1)
    summary_data['Percentage Fully Present Signal 2'].append(fully_present_signal_2)
    summary_data['Decoding Success Rate Signal 1'].append(success_rate_signal_1)
    summary_data['Decoding Success Rate Signal 2'].append(success_rate_signal_2)
    summary_data['Average Decoding Quality Signal 1'].append(decoding_quality_signal_1)
    summary_data['Average Decoding Quality Signal 2'].append(decoding_quality_signal_2)

# After looping, calculate performance differences and optimization metrics
for i in range(len(summary_data['Key'])):
# for i in range(len(summary_data['Threshold'])):
    error_rate_diff = summary_data['Error Rate Signal 2'][i] - summary_data['Error Rate Signal 1'][i]
    relative_error_diff = (error_rate_diff / summary_data['Error Rate Signal 1'][i]) * 100 if summary_data['Error Rate Signal 1'][i] != 0 else 0
    combined_error_rate = summary_data['Error Rate Signal 1'][i] + summary_data['Error Rate Signal 2'][i]
    max_error_rate = max(summary_data['Error Rate Signal 1'][i], summary_data['Error Rate Signal 2'][i])
    
    summary_data['Error Rate Difference (S2 - S1)'].append(error_rate_diff)
    summary_data['Relative Error Rate Difference (%)'].append(relative_error_diff)
    summary_data['Combined Error Rate'].append(combined_error_rate)
    summary_data['Max Error Rate'].append(max_error_rate)

# Convert summary data to DataFrame
df_summary = pd.DataFrame(summary_data)

# Find the optimal threshold based on combined error rate and max error rate
optimal_combined_threshold = df_summary.loc[df_summary['Combined Error Rate'].idxmin(), 'Key']
# optimal_combined_threshold = df_summary.loc[df_summary['Combined Error Rate'].idxmin(), 'Threshold']
optimal_max_threshold = df_summary.loc[df_summary['Max Error Rate'].idxmin(), 'Key']
# optimal_max_threshold = df_summary.loc[df_summary['Max Error Rate'].idxmin(), 'Threshold']

# Save the summary to a CSV file
df_summary.to_csv(filename, index=False)

# Output the optimal thresholds
print(f'Optimal Threshold based on Combined Error Rate: {optimal_combined_threshold}')
print(f'Optimal Threshold based on Max Error Rate: {optimal_max_threshold}')

# Optionally, you can include the plotting code here for visualization if desired
