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
    plt.hist(data_removed['TimeToStall'], bins=20, color='blue', alpha=0.7)
    plt.xlabel('Average Distance From Vip')
    plt.ylabel('Frequency')
    plt.title("A histogram depicting average distances of a convoy robot from the VIP")
    plt.savefig(directory + "distance_hist.png")

    # Calculating statistical measures

evaluate_data("./boids_follow_circle_results/")
plt.clf()
evaluate_data("./boids_follow_classic_results/")