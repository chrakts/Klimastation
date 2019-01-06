/*
 * bh1750.h
 *
 * Created: 01.01.2016 13:35:13
 *  Author: Christof
 */ 


#ifndef BH1750_H_
#define BH1750_H_

#define BH1750_Power_Down	0x00
#define BH1750_Power_On		0x01
#define BH1750_Reset_Data	0x07
#define BH1750_Cont_H_Res	0x10	
#define BH1750_Cont_H_Res2	0x11
#define BH1750_Cont_L_Res	0x13	
#define BH1750_OneT_H_Res	0x20
#define BH1750_OneT_H_Res2	0x21
#define BH1750_One_L_Res	0x23
#define BH1750_Chg_Ms_T_h_bm 0x40
#define BH1750_Chg_Ms_T_l_bm 0x60

#define BH1750_SLAVE_ADDR_H 0x5c
#define BH1750_SLAVE_ADDR_L 0x23



class BH1750
{
	private:

	TWI_Master_t *twi;
	uint8_t	twi_address;
	uint16_t raw_data;
	double illuminance;
	protected:
	
	public:
	BH1750(void);
	BH1750(TWI_Master_t *mytwi,int address);
	uint16_t Get_Data();
	void Set_Mode(uint8_t mode);
	void PowerOn();
	void PowerOff();
	void ResetData();
};


#endif /* BH1750_H_ */