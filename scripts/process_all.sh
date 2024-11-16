#!/bin/bash

# Directories
raw_csv_dir=~/Desktop/MRS_Master_Project/paper/adaptive_variant02_raw_csv
processed_csv_dir=~/Desktop/MRS_Master_Project/paper/processed_csv_automation_adaptive

# Make sure the processed_csv directory exists
mkdir -p "$processed_csv_dir"

# Loop over all CSV files in the raw_csv directory
for filepath in "$raw_csv_dir"/*.csv; do
    # Get the filename without extension
    filename=$(basename -- "$filepath")
    extension="${filename##*.}"
    name_experiment="${filename%.*}"

    echo "Processing $name_experiment"

    # Extract the variant number from the experiment name
    # Assuming the filename starts with 'variant' followed by the variant number
    if [[ "$name_experiment" =~ variant([0-9]+)_ ]]; then
        variant="${BASH_REMATCH[1]}"
        echo "Variant number extracted: $variant"
    else
        echo "Variant number not found in $name_experiment. Skipping..."
        continue
    fi

    # Run the Python script with the variant number
    python new_processing_raw_csv.py --name_experiment "$name_experiment" --variant "$variant"

done
