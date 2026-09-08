import numpy as np
from simulator import Simulator, centerline

sim = Simulator()

prev_error = np.zeros(5)
total_error = np.zeros(5)
period = 0.02 # don't know what this is but should be how often the control loop runs

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

    # TODO tune
    # this is probably different for xy and angle actually 
    kp = 0
    ki = 0
    kd = 0

    setpoint = np.zeros(5) #TODO how do you calculate the setpoint??
    output = np.zeros(5)

    # iterate 3x to find error and output for xpos, ypos, and phi
    for i in range(3):
        error = setpoint[i] - x[i]
        total_error[i] += error
        output[i] = kp * error + ki * total_error[i] + kd * ((error - prev_error[i]) / period)
        prev_error[i] = error

    # find difference between current vel and output vel, divide by period to get accel
    a = (v - np.linalg.norm(output[0], output[1])) / period
    '''
    find difference between current steering wheel angle and desired angle (?)
    f.e. if you want to be at 10 degrees and are currently at 0 (10 deg error) but your wheel is already at 10 degrees, don't do anything
    by contrast if you want to be at 0 and are currently at 10 degrees (-10 deg error) and your wheel is at like idk 5 degrees
    you should go -15 degrees
    this doesn't seem right lol
    '''
    d_theta = (output[3] - theta) / period
    return np.array([a, d_theta])

sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()