import numpy as np
from simulator import Simulator, centerline

sim = Simulator()

MAX_ACCEL = 4
MIN_ACCEL = -10

vel_kp = 10
vel_ki = 0
vel_kd = 0.1

theta_kp = 5
theta_ki = 0
theta_kd = 0

meters = np.zeros(105)
for i in range(105): # 105 meter track
    meters[i] = i
path = centerline(meters)

def find_nearest_point(x, y):
    min_dist = np.hypot(x-path[0][0], y-path[0][1])
    index = 0
    for i in range(105):
        if np.hypot(x-path[i][0], y-path[i][1]) < min_dist:
            index = i
            min_dist = np.hypot(x-path[i][0], y-path[i][1])
    return index

class PIDController():
    
    period = 0.1

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.total_error = 0
        self.prev_error = 0

    def calculate(self, error):
        output = self.kp * error + self.ki * self.total_error + self.kd * ((error - self.prev_error) / self.period)
        self.prev_error = error
        self.total_error += error
        return output

vel_controller = PIDController(vel_kp, vel_ki, vel_kd)
theta_controller = PIDController(theta_kp, theta_ki, theta_kd)

# for figuring out the total time
count = 0

def controller(x):
    global count
    """controller for a car

    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]

    Returns:
        ndarray: numpy array of shape (2,) containing [fwd acceleration, steering rate]
    """
    xpos   = x[0]                   # current x position
    ypos   = x[1]                   # current y position
    phi    = np.mod(x[2], 2*np.pi)  # current heading (radians)
    v      = x[3]                   # current velocity
    theta   = x[4]                  # current steering angle

    index = find_nearest_point(xpos, ypos)
    if (index > 101):
        setpoint = path[104] # idk just stop it
    else:
        setpoint = path[index + 3]

    target_heading = np.arctan2(setpoint[1] - ypos, setpoint[0] - xpos)
    target_heading = np.mod(target_heading, 2*np.pi)

    actual_tire_heading = phi + theta
    actual_tire_heading = np.mod(actual_tire_heading, 2*np.pi)

    # rewrap angle
    ang_error = (target_heading - actual_tire_heading + np.pi) % (2 * np.pi) - np.pi
    d_theta = theta_controller.calculate(ang_error)

    a = vel_controller.calculate(7-v)

    # i think the sim clamps this already but i guess for irl
    if a > MAX_ACCEL:
        a = MAX_ACCEL
    elif a < MIN_ACCEL:
        a = MIN_ACCEL

    count += 1
    print("time: ", count * 0.01)
    return np.array([a, d_theta])

sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()