#include <stdio.h>

#include "STS3X.h"

void STS3X_link_read(uint8_t adr, uint8_t *buff, uint8_t len);      
void STS3X_link_write(uint8_t adr, uint8_t *buff, uint8_t len);
void STS3X_link_delay_ms(int duration);

sts3x_cfg_t sts31_cfg = {
	.adr = STS3x_ADR_L,	              // Adr pin is tied low
	.clk_str = false,                     // clock streching off
	.mode = STS3x_MODE_SS,                // Single shot mode
	.repeat = STS3x_REPEAT_HIGH,          // Repeatability is high 
	                                      // Measurements per second is not needed in single shot mode
	.read = STS3X_link_read,              // Linking in functions
	.write = STS3X_link_write,
	.delay_ms = STS3X_link_delay_ms,
};

 /* Note limits are stored as volatile and are lost on brown out or reset  */

void app_main(void)
{
    STS3X_init(&sts31_cfg); 

    /* 
        The first agument requires an sts3x_alert_t value from the header, this is shown below:

        typedef enum  {
            ALERT_HIGH_SET = STS3X_CMD_ALIM_W_HL_SET,
            ALERT_HIGH_CLR = STS3X_CMD_ALIM_W_HL_CLR,
            ALERT_LOW_SET = STS3X_CMD_ALIM_W_LL_SET,
            ALERT_LOW_CLR = STS3X_CMD_ALIM_W_LL_CLR
        } sts3x_alert_t;

        The second argument is the temperature value as an int32_t. For example, 70.1C is 70100.
    */

    STS3X_set_alert_limit(ALERT_HIGH_SET, 70000); 
    STS3X_set_alert_limit(ALERT_HIGH_CLR, 65000);
    STS3X_set_alert_limit(ALERT_LOW_SET, -9000);
    STS3X_set_alert_limit(ALERT_LOW_CLR, -8000);

    int32_t limits[4] = {0, 0, 0, 0};

	for(uint8_t x = 0;;x++)
	{
		/* This function passes the alert limit into the second argument as the int32_t tempeature value. 
				Note, the tempeature value is stored on the sensor as a 9 bit value, so resolution is ~0.5C  
				
				The first argument uses sts3x_alert_t similarly to the above */
			
		STS3X_get_alert_limit(ALERT_HIGH_SET, &limits[0]);
		STS3X_get_alert_limit(ALERT_HIGH_CLR, &limits[1]);
		STS3X_get_alert_limit(ALERT_LOW_SET, &limits[2]);
		STS3X_get_alert_limit(ALERT_LOW_CLR, &limits[3]);

		prtinf("HS:%d HC:%d LS:%d LC:%d", limits[0], limits[1], limits[2], limits[3]);
		delay_ms(1000);
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

