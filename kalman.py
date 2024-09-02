import numpy as np

INPUT_FILE = 'data.txt'
OUTPUT_FILE = 'filtered_data.txt'

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimated_error, initial_estimate):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_error = estimated_error
        self.estimate = initial_estimate
        self.kalman_gain = 0
    
    def update(self, measurement):
        # Prediction update
        self.estimated_error += self.process_variance
        
        # Measurement update
        self.kalman_gain = self.estimated_error / (self.estimated_error + self.measurement_variance)
        self.estimate += self.kalman_gain * (measurement - self.estimate)
        self.estimated_error = (1 - self.kalman_gain) * self.estimated_error
        
        return self.estimate

def apply_kalman_filter_to_data(input_file, output_file):
    # Parameters for the Kalman filter (tune these based on your specific sensor)
    process_variance = 1e-4
    measurement_variance = 1e-2
    estimated_error = 1e-1
    initial_estimate = 0
    
    # Create Kalman filters for ax, ay, and gz
    kalman_filter_ax = KalmanFilter(process_variance, measurement_variance, estimated_error, initial_estimate)
    kalman_filter_ay = KalmanFilter(process_variance, measurement_variance, estimated_error, initial_estimate)
    kalman_filter_gz = KalmanFilter(process_variance, measurement_variance, estimated_error, initial_estimate)
    
    with open(input_file, 'r') as f, open(output_file, 'w') as out_f:
        for line in f:
            # Read and parse the line (ax, ay, az, gx, gy, gz, t)
            data = [float(val) for val in line.strip().split(',')]
            ax, ay, gz, t = data[0], data[1], data[5], data[6]
            
            # Apply Kalman filters to the ax, ay, and gz data
            filtered_ax = kalman_filter_ax.update(ax)
            filtered_ay = kalman_filter_ay.update(ay)
            filtered_gz = kalman_filter_gz.update(gz)
            
            # Write the filtered data back to the output file (with the original format)
            out_f.write(f'{filtered_ax},{filtered_ay},{data[2]},{data[3]},{data[4]},{filtered_gz},{t}\n')

# Call the function to apply the Kalman filter to the data and write the output
apply_kalman_filter_to_data(INPUT_FILE, OUTPUT_FILE)
