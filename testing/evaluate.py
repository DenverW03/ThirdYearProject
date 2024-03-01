import sys
import pandas as pd

# Taking the directory to output the summarized data to
directory = sys.argv[1]

# The number of robots multiplied by 2 is the skip length, as each robot outputs duplicate values, not sure why but perfection chasing is the enemy of progress aha
skip_length = int(sys.argv[2]) * 2

# The number of testing runs conducted
runs = int(sys.argv[3])

# The output csv
data_directory = directory + "output.csv"

# Constructing the overall data frame
new_dataframe = pd.DataFrame({"AvgDistance": [], "TimeToStall": []})

for i in range(runs):
    # Getting the first row to read
    row_skip = i * skip_length
    
    # Reading the csv
    temp_dataframe = pd.read_csv(data_directory, dtype=float, skiprows=row_skip, nrows=skip_length, names=["id","AvgDistance","TimeToStall"])

    print("----------")
    print(temp_dataframe)
    print("----------")

    # Dropping duplicate ids in the range
    temp_dataframe_no_duplicates = temp_dataframe.drop_duplicates(subset=["id"])

    # Dropping the first column
    temp_dataframe_no_id = temp_dataframe_no_duplicates.drop(columns=["id"])

    # Concatenating the frame
    new_dataframe = pd.concat([new_dataframe, temp_dataframe_no_id])
    
new_dataframe.to_csv(directory + 'processed_data.csv', index=False)