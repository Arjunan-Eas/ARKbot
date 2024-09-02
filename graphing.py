import matplotlib.pyplot as plt

# Initialize lists to store the data and deltas
gz_values = []
m1_values = []
m2_values = []
m3_values = []
m4_values = []

m1_deltas = []
m2_deltas = []
m3_deltas = []
m4_deltas = []

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

# Calculate the deltas for each motor
for i in range(1, len(m1_values)):
    m1_deltas.append(m1_values[i] - m1_values[i - 1])
    m2_deltas.append(m2_values[i] - m2_values[i - 1])
    m3_deltas.append(m3_values[i] - m3_values[i - 1])
    m4_deltas.append(m4_values[i] - m4_values[i - 1])

# Plotting the delta data and gz values
plt.figure(figsize=(10, 6))

# Plot the deltas of each motor
plt.plot(m1_deltas, label="Delta Motor 1", color="blue")
plt.plot(m2_deltas, label="Delta Motor 2", color="green")
plt.plot(m3_deltas, label="Delta Motor 3", color="red")
plt.plot(m4_deltas, label="Delta Motor 4", color="purple")

# Plot the gz values
plt.plot(gz_values[1:], label="GZ (Gyro Z)", color="black", linestyle="--")  # Slice to align with delta arrays

# Add labels and title
plt.xlabel("Sample Number")
plt.ylabel("Delta / GZ Values")
plt.title("Delta of Motor Values and GZ Over Time")
plt.legend()

# Display the plot
plt.show()
