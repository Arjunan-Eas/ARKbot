import math

INPUT_FILE = 'data.txt'
OUTPUT_FILE = 'trajectory.txt'
N = 50

def update_telemetry(sensor_data, telemetry, previous_data, rolling_avgs):
    # [ax, ay, gz, t]
    new_vals = [sensor_data[0], sensor_data[1], sensor_data[5]]

    # Find time interval
    dTime = sensor_data[6] - telemetry[3]

    for i in range(3):
        # Calculate rolling average before new value
        rolling_avgs[i][0] = average(previous_data[i])

        # Make sure to limit rolling average interval to N
        while(len(previous_data[i]) >= N):
            previous_data.pop(0)
        
        # Add new values to list
        previous_data[i].append(new_vals[i])

        # Compute average with new value
        rolling_avgs[i][1] = average(previous_data[i])

        # Integrate each rate
        telemetry[i] += trapezoid_rule(rolling_avgs[i], dTime)
    
    # Update time and interval
    telemetry[3] = sensor_data[6]
    telemetry[4] = dTime

    # Find rolling averages for thetaG and thetaA
    for i in range(3, 5):

        while(len(previous_data[i]) >= N):
            previous_data.pop(0)

    previous_data[3].append(telemetry[2] / (math.pi / 180))
    rolling_avgs[3] = average(previous_data[3])

    previous_data[4].append(math.atan(telemetry[0] / telemetry[1]))
    rolling_avgs[4] = average(previous_data[4])


    return telemetry, previous_data, rolling_avgs

def trapezoid_rule(values, interval):
    return ((values[0] + values[1]) / 2) * interval

def average(values):
    return sum(values) / len(values)

def update_position(telemetry, rolling_avgs, position):

    # Get two values of theta
    thetaG = telemetry[2] / (math.pi / 180)
    thetaA = math.atan(telemetry[0] / telemetry[1])

    sG = abs((rolling_avgs[3] - thetaG) / thetaG)
    sA = abs((rolling_avgs[4] - thetaA) / thetaA)

    thetaF = thetaG * (sG / (sG + sA)) + thetaA * (sA / (sG + sA))

    speed = ((telemetry[0] ** 2) + (telemetry[1] ** 2)) ** 0.5

    deltaX = speed * math.sin(thetaF) * telemetry[4]
    deltaY = speed * math.cos(thetaF) * telemetry[4]

    position[0] += deltaX
    position[1] += deltaY
    position[2] = telemetry[3]

    return position

def main(INPUT_FILE, OUTPUT_FILE):
    # Clear output file
    with open(OUTPUT_FILE, 'w') as g:
        pass

    # [x, y, time]
    position = [0, 0, 0]

    # [vx, vy, theta, time, interval length]
    telemetry = [0, 0, 0, 0, 0]

    # Stores up to N previous acceleration/gyro points [ax, ay, gz, thetaG, thetaA]
    previous_data = [[], [], [], [], []]

    # [[axi, axj], [ayi, ayj], [gzi, gzj], thetaG, thetaA]
    rolling_avgs = [[0, 0], [0, 0], [0, 0], 0, 0]

    with open(INPUT_FILE, 'r') as f:
        initial_data = f.readline().split(',')

        # Include initial values
        previous_data[0].append(initial_data[0])
        previous_data[1].append(initial_data[1])
        previous_data[2].append(initial_data[5])
        telemetry[3] = initial_data[6]

    with open(INPUT_FILE, 'r') as f:
        for line in f:
            sensor_data = line.split(',')
            telemetry, previous_data, rolling_avgs = update_telemetry(sensor_data, telemetry, previous_data)
            position = update_position(telemetry, rolling_avgs, position)
            with open(OUTPUT_FILE, 'a') as g:
                g.write(f'{position[0]},{position[1]},{position[3]}\n')

    print("Done!")