/*
 * STS3X.c
 *
 * Created: 21/06/2022 16:35:43
 *  Author: J.Dempsey (https://github.com/WifWaf)
 */ 

#include "src/components/drivers/include/STS3X.h"

#define STS3X_SS_CLKSTR_DIS_REP_BIT 0x06   // Bitwise value to convert from clock stretch enable to disable for repeatability
#define STS3X_SS_CLKSTR_DIS_BIT     0x08   // Bitwise value to convert from clock stretch enable to disable
#define STS3X_ALIM_WRITE_BIT        0x02   // Bitwise value to convert from alert set to clear

#define STS3X_MEAS_DUR_LOW   4.5  // typical 2.5
#define STS3X_MEAS_DUR_MED   6.5  // typical 4.5
#define STS3X_MEAS_DUR_HIGH  15.5 // typical 12.5

#define CRC8_POLY 0x131   // Polynomial value for CRC8 calculation
#define CRC8_INIT 0xFF    // Initial value for CR8 calculation

#define STS3X_MSB 0
#define STS3X_LSB 1
#define STS3X_CRC 2

void STS3X_make_measure_cmnd(sts3x_cfg_t *cfg);
int32_t STS3X_temp_from_s_t(uint16_t raw_temp);
uint16_t STS3X_s_t_from_temp(int32_t val); 

#if STS3X_CRC8	
bool STS3X_crc8_check(uint8_t data[], uint8_t checksum);
#endif
uint8_t STS3X_get_crc8(uint8_t data[]);  // required for writing

sts3x_cfg_t *_sts3x_cfg;
uint8_t sts3x_com[2] = {0, 0};
uint8_t measure_elapse = 0;

void STS3X_init(sts3x_cfg_t *cfg)
{
	_sts3x_cfg = cfg;
	STS3X_make_measure_cmnd(_sts3x_cfg);
	
	if(_sts3x_cfg->mode == STS3X_MODE_PE)
		STS3X_restart_periodic_mode();
}

sts3x_err_t STS3X_set_alert_limit(sts3x_alert_t alert_def, int32_t limit)
{
	uint8_t data[5] = {STS3X_CMD_ALIM_WRITE, alert_def, 0, 0, 0};	
	
	uint16_t temp_raw = STS3X_s_t_from_temp(limit);
	
	data[2] = ((temp_raw >> 15) & 0x01);          // mask the 9th bit for the MSB
	data[3] = ((temp_raw >> 7) & 0xFF);             // mask the remaining 8 bits for the LSB		
	uint8_t buff[2] = {data[2], data[3]};         // copy bytes 3 and 4 to buffer
	data[4] = STS3X_get_crc8(buff);	              // calculate crc and set as byte 5
	
	_sts3x_cfg->write(_sts3x_cfg->adr, data, 5);

	uint16_t reg = 0;
	if(!STS3X_read_status_reg(&reg) || reg & STS3X_STATUS_CRC_FAIL)   // read reg, check for CRC failure flag
		return STS3X_CRC8_FAIL;
		
	return STS3X_OK;
}

sts3x_err_t STS3X_get_alert_limit(sts3x_alert_t alert_def, int32_t *limit)
{
	alert_def ^= STS3X_ALIM_WRITE_BIT;
	
	uint8_t data[3] = {STS3X_CMD_ALIM_READ, alert_def, 0};

	_sts3x_cfg->write(_sts3x_cfg->adr, data, 2);
	data[STS3X_MSB] = 0; 
	data[STS3X_LSB] = 0;
	_sts3x_cfg->read(_sts3x_cfg->adr, data, 3);
	
#if STS3X_CRC8
	if(!STS3X_crc8_check(data, data[2]))
		return STS3X_CRC8_FAIL;
#endif
	 uint16_t temp_raw = (((((uint16_t)data[STS3X_MSB] << 8) & 0x100) | data[STS3X_LSB]) << 7) & 0xFF80;

	*limit = STS3X_temp_from_s_t(temp_raw);
	
	return STS3X_OK;	
}

sts3x_err_t STS3X_restart_periodic_mode()
{
	_sts3x_cfg->write(_sts3x_cfg->adr, sts3x_com, 2);
		
	sts3x_com[STS3X_MSB] = STS3X_CMD_PDA_READ_B1;
	sts3x_com[STS3X_LSB] = STS3X_CMD_PDA_READ_B2;
	
	uint16_t reg = 0;
	if(!STS3X_read_status_reg(&reg))	
		return STS3X_CRC8_FAIL;
		
	return (reg & STS3X_STATUS_PE_MODE) ? STS3X_OK : STS3X_FAIL;
}

sts3x_err_t STS3X_stop_periodic_mode()
{
	uint8_t data[2] = {STS3X_CMD_PDA_STOP_B1, STS3X_CMD_PDA_STOP_B2};
	
	_sts3x_cfg->write(_sts3x_cfg->adr, data, 2);	
	
	uint16_t reg = 0;
	if(!STS3X_read_status_reg(&reg))
		return STS3X_CRC8_FAIL;
	
	return (reg & STS3X_STATUS_PE_MODE) ? STS3X_FAIL : STS3X_OK;
}

sts3x_err_t STS3X_set_heater(bool state)  
{
	uint8_t data[2] = {STS3X_CMD_HEATER, 0};
	uint16_t status_reg = 0;

	data[STS3X_LSB] = (state) ? STS3X_HEATER_ON : STS3X_HEATER_OFF;		
	_sts3x_cfg->write(_sts3x_cfg->adr, data, 2);

	if(!STS3X_read_status_reg(&status_reg))
		return STS3X_CRC8_FAIL;
		
	 return ((status_reg & STS3X_STATUS_HEAT_ON) == state) ? STS3X_OK : STS3X_FAIL;
}

sts3x_err_t STS3X_soft_reset() 
{
	uint8_t data[2] = {STS3X_CMD_SOFT_RST_B1, STS3X_CMD_SOFT_RST_B2};
	uint16_t status_reg = 0;
	
	_sts3x_cfg->write(_sts3x_cfg->adr, data, 2);

	if(!STS3X_read_status_reg(&status_reg))
		return STS3X_CRC8_FAIL;
	
    return (status_reg & STS3X_STATUS_RST_OK) ? STS3X_OK : STS3X_FAIL;	
}

sts3x_err_t STS3X_read_status_reg(uint16_t *status_reg)
{
	uint8_t data[3] = {STS3X_CMD_STATUS_B1, STS3X_CMD_STATUS_B2, 0};		
		
	_sts3x_cfg->write(_sts3x_cfg->adr, data, 2);	
	data[STS3X_MSB] = 0; 
	data[STS3X_LSB] = 0;			
	_sts3x_cfg->read(_sts3x_cfg->adr, data, 3);	
			
#if STS3X_CRC8
	if(!STS3X_crc8_check(data, data[2]))
		return STS3X_CRC8_FAIL;
#endif

    *status_reg = ((data[STS3X_MSB] << 8) & 0xFF) | data[STS3X_LSB];	
	return STS3X_OK;
}

void STS3X_clear_status_reg()
{
	uint8_t data[2] = {STS3X_CMD_CLR_REG_B1, STS3X_CMD_CLR_REG_B2};
	
	_sts3x_cfg->write(_sts3x_cfg->adr, data, 2);
}

sts3x_err_t STS3X_get_temp(int32_t *temp)
{
	uint8_t data[3] = {0, 0, 0};		

	_sts3x_cfg->write(_sts3x_cfg->adr, sts3x_com, 2);
	
	if(!_sts3x_cfg->clk_str)
		_sts3x_cfg->delay_ms(measure_elapse);
	
	_sts3x_cfg->read(_sts3x_cfg->adr, data, 3);	
	
	if(!data[STS3X_MSB] && !data[STS3X_LSB] && _sts3x_cfg->mode == STS3X_MODE_PE)       // Note, measurements of exactly 0.000 C will be seen as not ready.
		return STS3X_NOT_READY;
				
#if STS3X_CRC8	
	if(!STS3X_crc8_check(data, data[STS3X_CRC]))
		return STS3X_CRC8_FAIL;
#endif	
	 
	*temp = STS3X_temp_from_s_t((data[STS3X_MSB] << 8) | data[STS3X_LSB]);

	return STS3X_OK;
}

int32_t STS3X_temp_from_s_t(uint16_t raw_temp)
{
	/* ---------------------------------------------------------------------------------------
               From the data sheet, where S_T is the raw temperature value.

                    T_celsius = -45 + 175(S_T / (2^16 - 1))

                   T_fahrenheit = -49 + 315(S_T / (2^16 - 1))		
	---------------------------------------------------------------------------------------- */
	
#if STS3X_TEMP_AS_F
	return (int32_t)((((int64_t)raw_temp * 315000) / 65535) - 49000);   // values are multiplied by 1e3 for int64_t resolution
#else                                                                   // and the 16 bit data for raw temperature is shifted.
	return (int32_t)((((int64_t)raw_temp * 175000) / 65535) - 45000);
#endif		
}

uint16_t STS3X_s_t_from_temp(int32_t val)
{
	/* ---------------------------------------------------------------------------------------
             Rearranged from the data sheet, where S_T is the raw temperature value.

                        S_T = (2^16 - 1)((T_celsius + 45) / 175)

                        S_T = (2^16 - 1	)((T_fahrenheit + 49) / 315)
---------------------------------------------------------------------------------------- */
#if STS3X_TEMP_AS_F
	return (uint16_t)((((int64_t)val + 49000) * 65535) / 315000);
#else   
	 return (uint16_t)((((int64_t)val + 45000) * 65535) / 175000);
#endif	
}

void STS3X_make_measure_cmnd(sts3x_cfg_t *cfg)
{
	if(_sts3x_cfg->mode == STS3X_MODE_PE)
	{
		sts3x_com[STS3X_MSB] = _sts3x_cfg->pda_mps;
		switch(_sts3x_cfg->pda_mps)
		{
			case STS3X_PE_MPS_05:
			switch(_sts3x_cfg->repeat)
			{
				case STS3X_REPEAT_HIGH:
				sts3x_com[STS3X_LSB] = STS3X_PE_05_REP_HIGH;
				break;
				case STS3X_REPEAT_MED:
				sts3x_com[STS3X_LSB] = STS3X_PE_05_REP_MED;
				break;
				case STS3X_REPEAT_LOW:
				sts3x_com[STS3X_LSB] = STS3X_PE_05_REP_LOW;
				break;
			}
			break;
			case STS3X_PE_MPS_1:
			switch(_sts3x_cfg->repeat)
			{
				case STS3X_REPEAT_HIGH:
				sts3x_com[STS3X_LSB] = STS3X_PE_1_REP_HIGH;
				break;
				case STS3X_REPEAT_MED:
				sts3x_com[STS3X_LSB] = STS3X_PE_1_REP_MED;
				break;
				case STS3X_REPEAT_LOW:
				sts3x_com[STS3X_LSB] = STS3X_PE_1_REP_LOW;
				break;
			}
			break;
			case STS3X_PE_MPS_2:
			switch(_sts3x_cfg->repeat)
			{
				case STS3X_REPEAT_HIGH:
				sts3x_com[STS3X_LSB] = STS3X_PE_2_REP_HIGH;
				break;
				case STS3X_REPEAT_MED:
				sts3x_com[STS3X_LSB] = STS3X_PE_2_REP_MED;
				break;
				case STS3X_REPEAT_LOW:
				sts3x_com[STS3X_LSB] = STS3X_PE_2_REP_LOW;
				break;
			}
			break;
			case STS3X_PE_MPS_4:
			switch(_sts3x_cfg->repeat)
			{
				case STS3X_REPEAT_HIGH:
				sts3x_com[STS3X_LSB] = STS3X_PE_4_REP_HIGH;
				break;
				case STS3X_REPEAT_MED:
				sts3x_com[STS3X_LSB] = STS3X_PE_4_REP_MED;
				break;
				case STS3X_REPEAT_LOW:
				sts3x_com[STS3X_LSB] = STS3X_PE_4_REP_LOW;
				break;
			}
			break;
			case STS3X_PE_MPS_10:
			switch(_sts3x_cfg->repeat)
			{
				case STS3X_REPEAT_HIGH:
				sts3x_com[STS3X_LSB] = STS3X_PE_10_REP_HIGH;
				break;
				case STS3X_REPEAT_MED:
				sts3x_com[STS3X_LSB] = STS3X_PE_10_REP_MED;
				break;
				case STS3X_REPEAT_LOW:
				sts3x_com[STS3X_LSB] = STS3X_PE_10_REP_LOW;
				break;
			}
			break;
		}
	}
	else
	{
		sts3x_com[STS3X_MSB] |= STS3X_SS_CLKSTR_EN;
		sts3x_com[STS3X_LSB] = _sts3x_cfg->repeat;
		
		if(!_sts3x_cfg->clk_str)
		{
			sts3x_com[STS3X_MSB] ^= STS3X_SS_CLKSTR_DIS_BIT;
			sts3x_com[STS3X_LSB] ^= STS3X_SS_CLKSTR_DIS_REP_BIT;
		}
		
		switch(measure_elapse)
		{
			case STS3X_SS_CLKSTR_REP_HIGH:
			measure_elapse = STS3X_MEAS_DUR_LOW;
			break;
			case STS3X_SS_CLKSTR_REP_MED:
			measure_elapse = STS3X_MEAS_DUR_MED;
			break;
			case STS3X_SS_CLKSTR_REP_LOW:
			measure_elapse = STS3X_MEAS_DUR_LOW;
			break;
		}
	}
}

uint8_t STS3X_get_crc8(uint8_t data[])
{
	uint8_t crc = CRC8_INIT;
	uint8_t byte_i;
	
	for (byte_i = 0; byte_i < 2; ++byte_i)
	{
		crc ^= (data[byte_i]);
		for (uint8_t bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80)
			crc = (crc << 1) ^ CRC8_POLY;
			else crc = (crc << 1);
		}
	}
	return crc;
}

#if STS3X_CRC8
bool STS3X_crc8_check(uint8_t data[], uint8_t checksum)
{
	uint8_t crc = STS3X_get_crc8(data);
	return (crc == checksum) ? true : false;
}
#endif
