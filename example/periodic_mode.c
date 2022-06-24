#include <stdio.h>

#include "STS3X.h"

void STS3X_link_read(uint8_t adr, uint8_t *buff, uint8_t len);      
void STS3X_link_write(uint8_t adr, uint8_t *buff, uint8_t len);
void STS3X_link_delay_ms(int duration);

sts3x_cfg_t sts31_cfg = {
	.adr = STS3x_ADR_L,	                  // Adr pin is tied low
	.clk_str = false,                     // clock streching off
	.mode = STS3x_MODE_PE,                // Single shot mode
	.repeat = STS3x_REPEAT_HIGH,          // Repeatability is high 
	.pda_mps = STS3X_PE_MPS_10,           // *Measurements per second is required for periodic mode
	.read = STS3X_link_read,              // Linking in functions
	.write = STS3X_link_write,
	.delay_ms = STS3X_link_delay_ms,
};

void app_main(void)
{
	uint32_t temp;
	bool heater_state = false;

	STS3X_init(&sts31_cfg); 

	for(uint8_t x = 0;;x++)
	{
		if(x > 10)
		{
			heater_state = !heater_state;
			STS3X_set_heater(heater_state);      // Turn off or on heater to test the sensor 
			x = 0;
		}
		
		if(STS3X_get_temp(&temp) == STS3X_OK)    // Function returns an error code. 0 = success. Returns 1 (STS3X_NOT_READY) if new measurement is not ready,
		{                                     
			int16_t temp_int = temp / 1000;      // Dividing by 1000 gives the integer part
			int16_t temp_dec = temp % 1000;      // Similarly, the remainder gives the fractional part

			printf("Temp: %d.%s%d", temp_int, (temp_dec < 100) ? "0" : "", temp_dec);
		}
		// STS3X_stop_periodic_mode();     // Periodic mode can be stopped by calling this functio. Note, this switches to single shot mode.
		delay_ms(1000);
		// STS3X_restart_periodic_mode();  // Periodic mode can be restated by calling this function.     
	}
}

void STS3X_link_read(uint8_t adr, uint8_t *buff, uint8_t len)
{
	i2c_m_sync_set_slaveaddr(&I2C_0, adr, I2C_M_SEVEN);
	io_read(I2C_SHARED, buff, len);
}

void STS3X_link_write(uint8_t adr, uint8_t *buff, uint8_t len)
{
	i2c_m_sync_set_slaveaddr(&I2C_0, adr, I2C_M_SEVEN);
	io_write(I2C_SHARED, buff, len);
}

void STS3X_link_delay_ms(int duration)
{
	delay_ms(duration);
}

