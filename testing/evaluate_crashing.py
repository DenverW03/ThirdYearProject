import pandas as pd
import matplotlib.pyplot as plt

def evaluate(directory, sets):
    # Making an empty dataframe to hold the crash percentage data
    crash_percentages = pd.DataFrame(columns=['num_robots', 'crash_percentage'])

    # For every number set of data add the crash percentage to the crash percentages
    for set in sets:
        data = pd.read_csv(directory + str(set) + "_processed_data.csv")

        # Calculating the stall percentage
        num_stall = (data['TimeToStall'] != 0.0).sum()
        stall_percentage = num_stall / len(data)

        # Adding to the dataframe
        new_frame = pd.DataFrame({
            'num_robots': [set],
            'crash_percentage': [stall_percentage]
        })
        crash_percentages = pd.concat([crash_percentages, new_frame], ignore_index=True)

    # Plotting the figure
    plt.plot(crash_percentages['num_robots'], crash_percentages['crash_percentage'], marker='o', linestyle='-', label=" ".join(directory.split('/')[-2].replace('_', ' ').split(' ')[2:]))

    # Add labels and title
    plt.xlabel('Number Of Convoy Robots')
    plt.ylabel('Crash Percentage')
    plt.title('A graph to show the crash percentage\nagainst the number of robots in the simulation')

sets = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
evaluate("./boids_follow_circle_results/", sets)
evaluate("./boids_follow_classic_results/", sets)

plt.legend()

plt.savefig("./crash_graph.png")