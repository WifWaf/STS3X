/*
 * STS3X.c
 *
 * Created: 21/06/2022 16:35:43
 *  Author: J.Dempsey (https://github.com/WifWaf)
 */ 

#include "STS3X.h"

#define STS3X_SS_CLKSTR_DIS_REP_BIT 0x06
#define STS3X_SS_CLKSTR_DIS_BIT     0x08

#define STS3X_MEAS_DUR_LOW   4.5  // typical 2.5
#define STS3X_MEAS_DUR_MED   6.5  // typical 4.5
#define STS3X_MEAS_DUR_HIGH  15.5 // typical 12.5

#define CRC8_POLY 0x131
#define CRC8_INIT 0xFF

#define STS3X_ARRAY_MSB 0
#define STS3X_ARRAY_LSB 1

void STS3X_make_measure_cmnd(sts3x_cfg_t *cfg);

#if STS3X_CRC8	
bool STS3X_crc8_check(uint8_t data[], uint8_t checksum);
#endif

sts3x_cfg_t *_sts3x_cfg;
uint8_t sts3x_com[2] = {0, 0};
uint8_t measure_elapse = 0;

void STS3X_init(sts3x_cfg_t *cfg)
{
	_sts3x_cfg = cfg;
	STS3X_make_measure_cmnd(_sts3x_cfg);

	if(_sts3x_cfg->mode == STS3X_MODE_PE)
		STS3X_restart_pe_mode();
}

void STS3X_restart_pe_mode()
{
	_sts3x_cfg->write(_sts3x_cfg->adr, sts3x_com, 2);

	sts3x_com[0] = STS3X_CMD_PDA_READ_B1;
	sts3x_com[1] = STS3X_CMD_PDA_READ_B2;
}

void STS3X_stop_pe_mode()
{
	uint8_t data[2] = {STS3X_CMD_PDA_STOP_B1, STS3X_CMD_PDA_STOP_B2};

	_sts3x_cfg->write(_sts3x_cfg->adr, data, 2);
}

sts3x_err_t STS3X_set_heater(bool state)
{
	uint8_t data[2] = {STS3X_CMD_HEATER, 0};
	uint16_t status_reg = 0;

	data[1] = (state) ? STS3X_HEATER_ON : STS3X_HEATER_OFF;	
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
	data[0] = 0; data[1] = 0;
	_sts3x_cfg->read(_sts3x_cfg->adr, data, 3);

#if STS3X_CRC8
	if(!STS3X_crc8_check(data, data[2]))
		return STS3X_CRC8_FAIL;
#endif

	*status_reg = ((data[0] << 8) & 0xFF) | data[1];
	return STS3X_OK;
}


void STS3x_clear_status_reg()
{
	uint8_t data[2] = {STS3X_CMD_CLR_REG_B1, STS3X_CMD_CLR_REG_B2};

	_sts3x_cfg->write(_sts3x_cfg->adr, data, 2);
}

sts3x_err_t STS3X_get_temp(uint32_t *temp)
{
	uint8_t data[3] = {0, 0, 0};

	_sts3x_cfg->write(_sts3x_cfg->adr, sts3x_com, 2);

	if(!_sts3x_cfg->clk_str)
		_sts3x_cfg->delay_ms(measure_elapse);

	_sts3x_cfg->read(_sts3x_cfg->adr, data, 3);

	if(!(data[0] & data[1]) && _sts3x_cfg->mode == STS3X_MODE_PE)            // *Note - assumes no readings will be take at 0C
		return STS3X_NOT_READY;

#if STS3X_CRC8
	if(!STS3X_crc8_check(data, data[2]))
		return STS3X_CRC8_FAIL;
#endif

     /* ---------------------------------------------------------------------------------------
        From Data sheet, where S_T is the raw temperature value.

                                     T_celsius = -45 + 175*(S_T / (2^16 - 1))

                                    T_fahrenheit = -49 + 315*(S_T / (2^16 - 1))
     ---------------------------------------------------------------------------------------- */

#if STS3X_TEMP_AS_F
	*temp = (int32_t)((((int64_t)((uint16_t)(data[0] << 8) | data[1]) * 315000) / 65535) -49000 );        // values are multiplied by 1e3 for int64_t resolution
#else                                                                                                     // and the 16 bit data for raw temperature is shifted.
	*temp = (int32_t)((((int64_t)((uint16_t)(data[0] << 8) | data[1]) * 175000) / 65535) -45000 );   
#endif

	return STS3X_OK;
}

void STS3X_make_measure_cmnd(sts3x_cfg_t *cfg)
{
	if(_sts3x_cfg->mode == STS3X_MODE_PE)
	{
		sts3x_com[STS3X_ARRAY_MSB] = _sts3x_cfg->pda_mps;
		switch(_sts3x_cfg->pda_mps)
		{
			case STS3X_PE_MPS_05:
			switch(_sts3x_cfg->repeat)
			{
				case STS3X_REPEAT_HIGH:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_05_REP_HIGH;
				break;
				case STS3X_REPEAT_MED:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_05_REP_MED;
				break;
				case STS3X_REPEAT_LOW:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_05_REP_LOW;
				break;
			}
			break;
			case STS3X_PE_MPS_1:
			switch(_sts3x_cfg->repeat)
			{
				case STS3X_REPEAT_HIGH:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_1_REP_HIGH;
				break;
				case STS3X_REPEAT_MED:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_1_REP_MED;
				break;
				case STS3X_REPEAT_LOW:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_1_REP_LOW;
				break;
			}
			break;
			case STS3X_PE_MPS_2:
			switch(_sts3x_cfg->repeat)
			{
				case STS3X_REPEAT_HIGH:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_2_REP_HIGH;
				break;
				case STS3X_REPEAT_MED:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_2_REP_MED;
				break;
				case STS3X_REPEAT_LOW:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_2_REP_LOW;
				break;
			}
			break;
			case STS3X_PE_MPS_4:
			switch(_sts3x_cfg->repeat)
			{
				case STS3X_REPEAT_HIGH:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_4_REP_HIGH;
				break;
				case STS3X_REPEAT_MED:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_4_REP_MED;
				break;
				case STS3X_REPEAT_LOW:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_4_REP_LOW;
				break;
			}
			break;
			case STS3X_PE_MPS_10:
			switch(_sts3x_cfg->repeat)
			{
				case STS3X_REPEAT_HIGH:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_10_REP_HIGH;
				break;
				case STS3X_REPEAT_MED:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_10_REP_MED;
				break;
				case STS3X_REPEAT_LOW:
				sts3x_com[STS3X_ARRAY_LSB] = STS3X_PE_10_REP_LOW;
				break;
			}
			break;
		}
	}
	else
	{
		sts3x_com[STS3X_ARRAY_MSB] |= STS3X_SS_CLKSTR_EN;
		sts3x_com[STS3X_ARRAY_LSB] = _sts3x_cfg->repeat;
		
		if(!_sts3x_cfg->clk_str)
		{
			sts3x_com[STS3X_ARRAY_MSB] ^= STS3X_SS_CLKSTR_DIS_BIT;
			sts3x_com[STS3X_ARRAY_LSB] ^= STS3X_SS_CLKSTR_DIS_REP_BIT;
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

#if STS3X_CRC8
bool STS3X_crc8_check(uint8_t data[], uint8_t checksum)
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
	return (crc == checksum) ? true : false;
}
#endif
