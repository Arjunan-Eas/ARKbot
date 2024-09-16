import matplotlib.pyplot as plt
import numpy as np

# Function to read data from file
def read_data(file_path):
    correction_list = []
    derivative_list = []
    deriv_list = []
    prop_list = []
    gz_list = []
    m1_list = []
    m2_list = []

    with open(file_path, 'r') as file:
        for line in file:
            # Strip any whitespace and split by comma
            values = line.strip().split(',')
            if len(values) == 8:  # Ensure correct format
                correction, derivative, deriv, prop, gz, m1, m2, _ = map(float, values)
                correction_list.append(correction)
                derivative_list.append(derivative)
                deriv_list.append(deriv)
                prop_list.append(prop)
                gz_list.append(gz)
                m1_list.append(m1)
                m2_list.append(m2)

    
    return correction_list, derivative_list, deriv_list, prop_list, gz_list, m1_list, m2_list

# Function to apply a rolling average filter
def rolling_average(data, window_size):
    return np.convolve(data, np.ones(window_size) / window_size, mode='valid')

# Function to plot gz data with and without rolling average filter
def plot_gz(rlist, filtered_list):
    plt.figure(figsize=(10, 6))
    plt.plot(rlist, label='Original', color='blue', alpha=0.5)
    plt.plot(range(len(filtered_list)), filtered_list, label='Filtered', color='red')
    plt.xlabel('Time (arbitrary units)')
    plt.ylabel('value')
    plt.title('over Time with Rolling Average Filter')
    plt.legend()
    plt.grid(True)
    plt.show()

# Main function
def main():
    file_path = 'data.txt'  # Replace with your file path
    correction_list, derivative_list, deriv_list, prop_list, gz_list, m1_list, m2_list = read_data(file_path)
    
    # Apply rolling average filter (e.g., window size of 5)
    window_size = 1
    filtered_list = rolling_average(gz_list, window_size)
    
    # Plot gz data with and without rolling average filter
    plot_gz(derivative_list, filtered_list)

if __name__ == "__main__":
    main()
