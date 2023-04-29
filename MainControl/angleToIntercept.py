import numpy as np

# Uses the data from the camera to determine the angle the robot needs to turn to intercept the other robot.
def angleToTurn(distance, theta0, theta1, runnerSpeed, chaserSpeed, sideOfRobot):
    # Check with lam on the logic
    if (theta0 < 0): # Robot is on the left
        theta = 180 + theta0 - theta1
    else: # Robot is on the right
        theta = 180 + theta0 + theta1
    b = 2*distance*runnerSpeed*np.cos(theta)
    a = np.power(chaserSpeed, 2)-np.power(runnerSpeed, 2)
    c = np.power(distance,2)
    time = (-b+np.sqrt(np.power(b,2)+4*a*c))/(2*a)
    # D = (cos(theta0)*d, sin(theta0)*d)
    Dy = np.sin(theta0)*d
    # VR = (cos(theta1)*runnerSpeed, sin(theta1)*runnerSpeed)
    VRy = np.sin(theta1)*runnerSpeed
    y = (Dy+VRy*time)/time
    return np.arccos(y/chaserSpeed)
