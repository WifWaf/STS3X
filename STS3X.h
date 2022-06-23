/*
 * STS3X.h
 *
 * Created: 21/06/2022 16:35:07
 *  Author: J.Dempsey (https://github.com/WifWaf)
 */ 

#include <stdbool.h>
#include <stdint.h>

#ifndef STS3X_H_
#define STS3X_H_ 

#define STS3X_CRC8               1     // set to 1 to enable CRC8 checks, else set to 0.
#define STS3X_TEMP_AS_F          0     // set to 0 to enable temperature as Fahrenheit, else set to 0 for Celsius.

#define STS3X_ADR_L              0x4A
#define STS3X_ADR_H              0x4B

#define STS3X_SS_CLKSTR_EN       0x2C
#define STS3X_SS_CLKSTR_DIS      0x24

#define STS3X_SS_CLKSTR_REP_HIGH 0x06
#define STS3X_SS_CLKSTR_REP_MED  0x0D
#define STS3X_SS_CLKSTR_REP_LOW  0x10

#define STS3X_PE_MPS_05          0x20
#define STS3X_PE_MPS_1           0x21
#define STS3X_PE_MPS_2           0x22
#define STS3X_PE_MPS_4           0x23
#define STS3X_PE_MPS_10          0x27

#define STS3X_PE_05_REP_HIGH     0x32
#define STS3X_PE_05_REP_MED      0x24
#define STS3X_PE_05_REP_LOW      0x2F
#define STS3X_PE_1_REP_HIGH      0x30
#define STS3X_PE_1_REP_MED       0x26
#define STS3X_PE_1_REP_LOW       0x2D
#define STS3X_PE_2_REP_HIGH      0x36
#define STS3X_PE_2_REP_MED       0x20
#define STS3X_PE_2_REP_LOW       0x2B
#define STS3X_PE_4_REP_HIGH      0x34
#define STS3X_PE_4_REP_MED       0x22
#define STS3X_PE_4_REP_LOW       0x29
#define STS3X_PE_10_REP_HIGH     0x37
#define STS3X_PE_10_REP_MED      0x21
#define STS3X_PE_10_REP_LOW      0x2A

#define STS3X_CMD_HEATER         0x30
#define STS3X_CMD_SOFT_RST_B1    0x30
#define STS3X_CMD_SOFT_RST_B2    0xA2
#define STS3X_CMD_STATUS_B1      0xF3
#define STS3X_CMD_STATUS_B2      0x2D
#define STS3X_CMD_CLR_REG_B1     0x30
#define STS3X_CMD_CLR_REG_B2     0x41
#define STS3X_CMD_PDA_READ_B1    0xE0
#define STS3X_CMD_PDA_READ_B2    0x00
#define STS3X_CMD_PDA_STOP_B1    0x30
#define STS3X_CMD_PDA_STOP_B2    0x93
#define STS3X_STATUS_CMD_OK      0x00

#define STS3X_HEATER_ON          0x6D
#define STS3X_HEATER_OFF         0X66
#define STS3X_STATUS_RST_OK      0x04
#define STS3X_STATUS_HEAT_ON     0x2000

typedef enum  {
	STS3X_MODE_SS,
	STS3X_MODE_PE
} sts3x_mode_t;

typedef enum  {
	STS3X_REPEAT_HIGH = STS3X_SS_CLKSTR_REP_HIGH,
	STS3X_REPEAT_MED = STS3X_SS_CLKSTR_REP_MED,
	STS3X_REPEAT_LOW = STS3X_SS_CLKSTR_REP_LOW
} sts3x_repeat_t;

typedef enum  {
	STS3X_OK,
	STS3X_NOT_READY,
	STS3X_CRC8_FAIL,
	STS3X_FAIL
} sts3x_err_t;

typedef struct  {
	uint8_t adr;
	bool clk_str;
	uint8_t pda_mps;
	
	sts3x_mode_t mode;
	sts3x_repeat_t repeat;
	
	void (*read)(uint8_t adr, uint8_t *buff, uint8_t len);
	void (*write)(uint8_t adr, uint8_t *buff, uint8_t len);
	void (*delay_ms)(uint16_t dur);
} sts3x_cfg_t;

void STS3X_init(sts3x_cfg_t *cfg);
void STS3X_restart_pe_mode();
void STS3X_stop_pe_mode();
void STS3X_clear_status_reg();

sts3x_err_t STS3X_set_heater(bool state); 
sts3x_err_t STS3X_soft_reset();
sts3x_err_t STS3X_read_status_reg(uint16_t *status_reg);
sts3x_err_t STS3X_get_temp(uint32_t *temp);

#endif /* STS3X_H_ */