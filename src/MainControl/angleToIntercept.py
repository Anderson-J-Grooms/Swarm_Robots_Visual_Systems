import numpy as np

# Uses the data from the camera to determine the angle the robot needs to turn to intercept the other robot.
def angleToIntercept(distance, theta0, theta1, squareColor, runnerSpeed, chaserSpeed):
    #if squareColor == "purple":
    #    theta1 = theta1 + 90
    #elif squareColor == "green":
    #    theta1 = theta1 + 180
    print(squareColor)
    theta0 = 90 - theta0
    lor = False #left false right true
    # Check with lam on the logic 
    #if (theta1 > 90 or theta1 > 270): # Robot is going left
    if (squareColor == 0):
        theta = theta0 - theta1
        print("purple {} {}".format(theta, squareColor == 0))
        if theta1 < 0:
            VRy = np.sin(np.absolute(np.deg2rad(theta1)))*runnerSpeed
        else:
            VRy = -np.sin(np.absolute(np.deg2rad(theta1)))*runnerSpeed
    elif (squareColor == 1):
        theta = 180 - theta0 + theta1
        print("green {} {}".format(theta, squareColor == 1))
        lor = True
        if theta1 < 0:
            VRy = -np.sin(np.absolute(np.deg2rad(theta1)))*runnerSpeed
        else:
            VRy = np.sin(np.absolute(np.deg2rad(theta1)))*runnerSpeed
    elif (squareColor == 2):
        if theta1 > 0: #purple
            theta1 = theta1 - 90
            theta = theta0 - theta1
            if theta1 < 0:
                VRy = np.sin(np.absolute(np.deg2rad(theta1)))*runnerSpeed
            else:
                VRy = -np.sin(np.absolute(np.deg2rad(theta1)))*runnerSpeed
            print("purple {} {}".format(theta, squareColor == 0))
        else:
            theta1 = theta1 + 90
            theta = 180 - theta0 + theta1
            print("green {} {}".format(theta, squareColor == 1))
            lor = True
            if theta1 < 0:
                VRy = -np.sin(np.absolute(np.deg2rad(theta1)))*runnerSpeed
            else:
                VRy = np.sin(np.absolute(np.deg2rad(theta1)))*runnerSpeed
    else:
        print("NAN")
    #else: # Robot is on the right
    #    #if(theta1 > 90 or theta1 > 270): # Robot is going left
    #    if (squareColor == 'purple' or (squareColor == 'red' and theta1 < 0)):
    #        theta = theta0 - theta1
    #    else:
    #        theta = 180 - theta0 + theta1
    theta = np.deg2rad(theta)
    theta0 = np.deg2rad(theta0)
    theta1 = np.deg2rad(theta1) 
    b = 2*distance*runnerSpeed*np.cos(theta)
    a = np.power(chaserSpeed, 2)-np.power(runnerSpeed, 2)
    c = np.power(distance,2)
    time = (-b+np.sqrt(np.power(b,2)+4*a*c))/(2*a)
    # D = (cos(theta0)*d, sin(theta0)*d)
    Dy = np.sin(theta0)*distance
    # VR = (cos(theta1)*runnerSpeed, sin(theta1)*runnerSpeed)
    #VRy = np.sin(theta1)*runnerSpeed
    y = (Dy+VRy*time)/time
    print("{} {} {}".format(Dy, VRy, time))
    print("{} {}".format(y, chaserSpeed))
    angle = np.arccos(y/chaserSpeed)
    angle = np.rad2deg(angle)
    if lor: 
        return angle, time
    else:
        return -angle, time
