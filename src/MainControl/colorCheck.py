# This is a function used for parsing RGB sensor values to determine a command.
# Each color represents one bit making 6 possible commands (red is not detected well).
# MSB - Red , Green, LSB - Blue
def colorCheck(h,s,v) :

    #if (v >= 99) :
    #    return "white"

    #if (s < 14.5 and v < 60) :
	#return "black"

    # Command 7 - seeing red
    if (h >= 0 and h < 30) :
        return "red"

    # Command 6 - seeing yellow
    if (h >= 30 and h < 60):
        return "yellow"

    # Command 5 - seeing green
    if (h >= 60 and h < 100):
        return "green"

    # Command 4 - seeing cyan
    if (h >= 100 and h < 220):
        return "cyan"

    # Command 3 - seeing blue
    if (h >= 220 and h < 275):
        return "blue"

    # Command 2 - seeing magenta
    if (h >= 275) :
        return "magenta"

    # Command 0 - default, no light or black light
    return "black"
