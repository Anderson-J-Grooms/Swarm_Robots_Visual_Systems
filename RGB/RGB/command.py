# Threshold values to be changed
greenThreshold = 4000
redThreshold = 5000
blueThreshold = 5000
# This is a function used for parsing RGB sensor values to determine a command.
# Each color represents one bit making 8 possile commands.
# MSB - Red , Green, LSB - Blue
def command(redValue, greenValue, blueValue) : 
    # Command 7 - Contains all light which makes white light
    if (greenValue > greenThreshold & redValue > redThreshold & blueValue > blueThreshold) :
        return "white"
    # Command 6 - red and green light makes yellow light
    if (redValue > redThreshold  & greenValue > greenThreshold):
        return "yellow"
    # Command 5 - red and blue light makes magenta light
    if (redValue > redThreshold  & blueValue > blueThreshold):
        return "magenta"
    # Command 4 - red light
    if (redValue > redThreshold):
        return "red"
    # Command 3 - green and blue light makes cyan light
    if (greenValue > greenThreshold & blueValue > blueThreshold):
        return "cyan"
    # Command 2 - green light
    if (greenValue > greenThreshold) :
        return "green"
    #Command 1 - blue light
    if (blueValue > blueThreshold):
        return "blue"
    # Command 0 - default, no light or black light
    return "black"
