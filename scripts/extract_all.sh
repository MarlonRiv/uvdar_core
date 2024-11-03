#!/bin/bash

# Mapping file path
mapping_file="file_mapping_adaptive_variant01.csv"

# Output directory for CSV files
output_csv_dir=~/Desktop/MRS_Master_Project/paper/adaptive_raw_csv

# Make sure the output directory exists
mkdir -p "$output_csv_dir"

# Read the mapping file line by line
while IFS=, read -r rx_bag tx1_bag tx2_bag; do
    # Skip header line if present
    if [[ $rx_bag == "rx_bag_path" ]]; then
        continue
    fi

    if [ -z "$rx_bag" ] || [ -z "$tx1_bag" ] || [ -z "$tx2_bag" ]; then
        echo "Incomplete mapping: $rx_bag, $tx1_bag, $tx2_bag"
        continue
    fi

    # Get the name_experiment from rx_bag filename
    rx_bag_filename=$(basename "$rx_bag")
    name_experiment="${rx_bag_filename%.*}"  # Remove .bag extension
    echo "Processing experiment: $name_experiment"

    # Output CSV file path
    output_csv="$output_csv_dir/${name_experiment}.csv"

    # Run the Python script
    python new_extract_data.py --name_experiment "$name_experiment" --rx_bag "$rx_bag" --tx1_bag "$tx1_bag" --tx2_bag "$tx2_bag" --output_csv "$output_csv"

done < "$mapping_file"
