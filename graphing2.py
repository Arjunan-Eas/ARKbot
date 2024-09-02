import matplotlib.pyplot as plt

# Initialize lists to store the data
gz_values = []
m1_values = []
m2_values = []
m3_values = []
m4_values = []

# Read the data file
with open('data.txt', 'r') as file:
    for line in file:
        # Split the line by comma to extract the values
        values = line.strip().split(',')
        
        # Ensure that there are exactly 5 values per line
        if len(values) == 5:
            gz, m1, m2, m3, m4 = map(float, values)
            gz_values.append(gz)
            m1_values.append(m1)
            m2_values.append(m2)
            m3_values.append(m3)
            m4_values.append(m4)

# Plotting the data
plt.figure(figsize=(10, 6))

# Plot each motor's data
plt.plot(gz_values, label="GZ (Gyro Z)", color="black", linestyle="--")
plt.plot(m1_values, label="Motor 1", color="blue")
plt.plot(m2_values, label="Motor 2", color="green")
plt.plot(m3_values, label="Motor 3", color="red")
plt.plot(m4_values, label="Motor 4", color="purple")

# Add labels and title
plt.xlabel("Sample Number")
plt.ylabel("Values")
plt.title("GZ and Motor Values Over Time")
plt.legend()

# Display the plot
plt.show()
