import numpy as np
from simulator import Simulator, centerline

sim = Simulator()

x_kp = 1
x_ki = 0
x_kd = 0

y_kp = 1
y_ki = 0
y_kd = 0

theta_kp = 1
theta_ki = 0
theta_kd = 0

meters = np.zeros(105)
for i in range(105): # 105 meter track
    meters[i] = i
path = centerline(meters)

dist_traveled = 0
x_last = -1
y_last = -1

# def find_next_point(x):
#     for p in path:
#         if p[0] > x:
#             return p
#     return np.zeros(2)

def find_nearest_point(pt):
    min_dist = np.linalg.norm(pt.subtract(path[0]))
    min_pt = path[0]
    index = 0
    for i in range(105):
        if np.linalg.norm(pt.subtract(path[i])) < min:
            min_pt = path[i]
            index = i
    return min_pt, i

speed = 4
class PIDController():
    total_error = 0
    prev_error = 0
    period = 0.02 # don't know what this is but should be how often the control loop runs

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def calculate(self, current, setpoint):
        error = setpoint - current
        output = self.kp * error + self.ki * self.total_error + self.kd * ((error - self.prev_error) / self.period)
        self.prev_error = error
        self.total_error += error
        return output

x_controller = PIDController(x_kp, x_ki, x_kd)
y_controller = PIDController(y_kp, y_ki, y_kd)
theta_controller = PIDController(theta_kp, theta_ki, theta_kd)

def controller(x):
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

    # we are starting over actually

    nearest_pt = find_nearest_point(np.array[xpos, ypos])[0]
    index = find_nearest_point(np.array[xpos, ypos])[1]
    setpoint = path[index + 1]
    x_vel = np.cos(phi) * v
    y_vel = np.sin(phi) * v
    x_vel += x_controller.calculate(xpos, nearest_pt[0])
    y_vel += y_controller.calculate(ypos, nearest_pt[1])

    
    
    return np.array([a, d_theta])

sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()