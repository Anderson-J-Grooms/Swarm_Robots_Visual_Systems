#include <stdio.h>
#include </opt/source/librobotcontrol/library/include/rc/i2c.h>
#include <unistd.h>

#define I2C_BUS 1
#define DEVICE_ADDR 0x44

#define DEVICE_ID 0x00
#define CONFIG_1 0x01
#define CONFIG_2 0x02
#define CONFIG_3 0x03
#define THRESHOLD_LL 0x04
#define THRESHOLD_LH 0x05
#define THRESHOLD_HL 0x06
#define THRESHOLD_HH 0x07
#define STATUS 0x08 
#define GREEN_L 0x09 
#define GREEN_H 0x0A
#define RED_L 0x0B
#define RED_H 0x0C
#define BLUE_L 0x0D
#define BLUE_H 0x0E

#define ISL29125_RESET 0x46  
#define ISL29125_WHOAMI 0x7D
#define ISL29125_RNG_MASK 0xF7
#define ISL29125_RGB    0x05    // Red, Green and Blue  

int main()
{
	uint8_t low;
	uint8_t high;
	uint16_t green_out;
	uint16_t red_out;
	uint16_t blue_out;

	printf("\nTest for receiving data. This will update every 2 seconds\n");
	rc_i2c_init(I2C_BUS, DEVICE_ADDR);

	uint8_t data;
        rc_i2c_read_byte(I2C_BUS, DEVICE_ID, &data);

	// Initialize sensor
	uint8_t cmd = ISL29125_RESET;
	rc_i2c_write_bytes(I2C_BUS, DEVICE_ID, 1, &cmd);
	// Init the ISL29125 : Enable RGB operating mode, sensing range = 10000 lux, 16 bit resolution
	// Following registers remain at the default reset values :
	// Register 0x03 - Interrupt source (none), persist control (1) , INT when conversion (0).
	// Register 0x04, 0x05 - Low threshold interrupt : 0x0000
	// Register 0x06, 0x07 - High threshold interrupt : 0xFFFF
	cmd = ISL29125_RGB;
	rc_i2c_write_bytes(I2C_BUS, CONFIG_1, 1, &cmd);
	// End intialization
	
	// uint8_t hold = 0;
	
	printf("\nData: \n");
	for(int i=0;i<5;i++) {
		printf("\nGreen data: ");
		rc_i2c_read_byte(I2C_BUS, GREEN_L, &low);
		rc_i2c_read_byte(I2C_BUS, GREEN_H, &high);
		green_out = (high << 8) | low;
		green_out = (uint8_t) green_out / 2^8;
		printf("High: %x Low: %x Out: %d\n", high, low, green_out);
		sleep(1);
		printf("Red data: ");
		rc_i2c_read_byte(I2C_BUS, RED_L, &low);
		rc_i2c_read_byte(I2C_BUS, RED_H, &high);
		red_out = (high << 8) | low;
		red_out = (uint8_t) red_out / 2^8;
		printf("High: %x Low: %x Out: %d\n", high, low, red_out);
		sleep(1);
		printf("Blue data: ");
		rc_i2c_read_byte(I2C_BUS, BLUE_L, &low);
		rc_i2c_read_byte(I2C_BUS, BLUE_H, &high);
		blue_out = (high << 8) | low;
		blue_out = (uint8_t) blue_out / 2^8;
		printf("High: %x Low: %x Out: %d\n", high, low, blue_out);
		sleep(1);
	}
}
