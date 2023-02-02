greenThreshold = 5000
redThreshold = 5000
blueThreshold = 5000
# This is a function used for parsing RGB sensor values to determine a command.
def command(redValue, greenValue, blueValue) : 
    if (greenValue > greenThreshold):
        return "green"
    if (redValue > redThreshold):
        return "red"
    if (blueValue > blueThreshold):
        return "blue"
    return 0
