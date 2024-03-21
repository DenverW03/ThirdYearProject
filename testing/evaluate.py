import pandas as pd
import matplotlib.pyplot as plt

def evaluate_data(directory):
    # Loading the CSV data and removing rows where the value is 0
    data = pd.read_csv(directory + 'processed_data.csv')

    # Plotting the average distance as a histogram
    plt.hist(data['AvgDistance'], bins=20, color='blue', alpha=0.7)
    plt.xlim(0, 20)
    plt.xlabel('Average Distance From Vip')
    plt.ylabel('Frequency')
    plt.title("A histogram depicting average distances of a convoy robot from the VIP")
    plt.savefig(directory + "distance_hist.png")

    # Clearing the plot
    plt.clf()

    # Plotting the time to stall
    data_removed = data[data['TimeToStall'] != 0.0]
    plt.hist(data_removed['TimeToStall'], bins=20, color='red', alpha=0.7)
    plt.xlabel('Time in seconds')
    plt.ylabel('Frequency')
    plt.title("A histogram depicting time taken for a convoy robot to crash")
    plt.savefig(directory + "time_hist.png")

    # Calculating the stall percentage
    num_stall = (data['TimeToStall'] != 0.0).sum()
    stall_percentage = num_stall / len(data)
    print("Stall Percentage: " + str(stall_percentage))

    # Calculating statistical measures
    mean_distance = data['AvgDistance'].mean()
    print("Mean Distance: " + str(mean_distance))

    sd_distance = data['AvgDistance'].std()
    print("Distance Standard Deviation: " + str(sd_distance))

    variance_distance = data['AvgDistance'].var()
    print("Distance Variance: " + str(variance_distance))

    # Calculating statistical measures
    mean_distance = data_removed['TimeToStall'].mean()
    print("Mean Stall Time: " + str(mean_distance))

    sd_distance = data_removed['TimeToStall'].std()
    print("Time Standard Deviation: " + str(sd_distance))

    variance_distance = data_removed['TimeToStall'].var()
    print("Time Variance: " + str(variance_distance))


print("-----------------------------")
print("Circle")
evaluate_data("./boids_follow_circle_results/")
print("-----------------------------")
plt.clf()
print("Classic")
evaluate_data("./boids_follow_classic_results/")
print("-----------------------------")