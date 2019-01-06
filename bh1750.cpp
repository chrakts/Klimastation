/*
 * CPPFile1.cpp
 *
 * Created: 01.01.2016 13:31:08
 *  Author: Christof
 */

//#include "TLog.h"
#include <avr/io.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
//#include "RomId/RomId.h"
#include "twi_master_driver.h"
#include "bh1750.h"
#include "SFE_BMP180.h"

BH1750::BH1750(void)
{
}

BH1750::BH1750(TWI_Master_t *mytwi,int address)
{
	twi = mytwi;
	twi_address = address;
	Set_Mode(BH1750_Cont_H_Res);
}

uint16_t BH1750::Get_Data()
{
	TWI_MasterRead(twi,twi_address,2);

	while (twi->status != TWIM_STATUS_READY) {
		/* Wait until transaction is complete. */
	}
	return( (uint16_t)(((twi->readData[0])<<8)|(twi->readData[1])) );
}

void BH1750::Set_Mode(uint8_t mode)
{
uint8_t md;
	md = mode;
	TWI_MasterWrite(twi,twi_address,&md,1);

	while (twi->status != TWIM_STATUS_READY) {
		/* Wait until transaction is complete. */
	}
}

void BH1750::PowerOn()
{
uint8_t md;
	md = BH1750_Power_On;
	TWI_MasterWrite(twi,twi_address,&md,1);

	while (twi->status != TWIM_STATUS_READY) {
		/* Wait until transaction is complete. */
	}
}

void BH1750::PowerOff()
{
	uint8_t md;
	md = BH1750_Power_Down;
	TWI_MasterWrite(twi,twi_address,&md,1);

	while (twi->status != TWIM_STATUS_READY) {
		/* Wait until transaction is complete. */
	}
}

void BH1750::ResetData()
{
	uint8_t md;
	md = BH1750_Reset_Data;
	TWI_MasterWrite(twi,twi_address,&md,1);

	while (twi->status != TWIM_STATUS_READY) {
		/* Wait until transaction is complete. */
	}
}
