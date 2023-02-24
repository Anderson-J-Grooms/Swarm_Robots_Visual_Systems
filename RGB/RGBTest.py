import smbus
import time
from colorCheck import*

# Get I2C bus
bus = smbus.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
#               0x0D(13)        Operation: RGB, Range: 10000 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x0D)

# Threshold values to be changed
greenThreshold = 500
redThreshold = 400
blueThreshold = 300

greenBlack = 0
greenWhite = 0
redBlack = 0
redWhite = 0
blueBlack = 0
blueWhite = 0
rScale = 1
gScale = 1
bScale = 1
r = 0
g = 0
b = 0

first = True
firstCount = 0
second = True
secondCount = 0

time.sleep(0.5)
count = 0
while True:
        # ISL29125 address, 0x44(68)
        # Read data back from 0x09(9), 6 bytes
        # Green LSB, Green MSB, Red LSB, Red MSB, Blue LSB, Blue MSB
        data = bus.read_i2c_block_data(0x44, 0x09, 6)


        # Convert the data
        g = data[1] * 256 + data[0]
        r = data[3] * 256 + data[2]
        b = data[5] * 256 + data[4]

	if first:
		greenBlack = min(g, greenBlack)
		redBlack = min(r, redBlack)
		blueBlack = min(b, blueBlack)
		firstCount = firstCount + 1
		if (firstCount >= 2000):
			first = False
			print("Switch to white and press enter to continue")
			raw_input()
	elif second:
                greenWhite = max(g, greenWhite)
                redWhite = max(r, redWhite)
                blueWhite = max(b, blueWhite)
		rScale = redWhite-redBlack
  		gScale = greenWhite-greenBlack
  		bScale = blueWhite-blueBlack
		secondCount = secondCount + 1
		if (secondCount >= 2000):
                	second= False
                	print("Press enter to continue")
                	raw_input()
	else:
        	try: 
            		r = 1.0 * (r-redBlack) / rScale
            		g = 1.0 * (g-greenBlack) / gScale
            		b = 1.0 * (b-blueBlack) / bScale 
        	except:
            		print("Division by Zero")

		cmax = max(r, g, b) # maximum of r, g, b
        	
  		cmin = min(r, g, b) # minimum of r, g, b
  		diff = cmax - cmin # diff of cmax and cmin.
  		h = -1 # h is hue value, s is saturation
		s = -1

  		# if cmax and cmax are equal then h = 0
  		if (cmax == cmin):
    			h = 0
  		# if cmax equal r then compute h
  		elif (cmax == r):
    			h = (60.0 * ((g - b) / diff) + 360) % 360
  		# if cmax equal g then compute h
  		elif (cmax == g):
  			h = (60.0 * ((b - r) / diff) + 120) % 360
  		# if cmax equal b then compute h
  		elif (cmax == b):
    			h = (60.0 * ((r - g) / diff) + 240) % 360
  	
		if cmax == 0:
			s = 0
		else:
			s = (diff / cmax) * 100
	
		v = cmax * 100

		out = colorCheck(h,s,v)
        	print out,' hue: ', round(h, 2),' Sat: ', round(s,2),'Value', round(v,2)
		time.sleep(2)