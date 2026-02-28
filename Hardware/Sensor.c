#include "Sensor.h"
#include "i2c.h"

#define sensor_address (0x12<<1)

uint8_t Sensor_Read(void)
{
uint8_t sensor_data=0;
	if(HAL_I2C_Master_Receive(&hi2c1,sensor_address,&sensor_data,1,100)==HAL_OK)
	{
	  return sensor_data;
	}
	return 0xFF;
}
