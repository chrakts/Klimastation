/*
* TLog.cpp
*
* Created: 16.03.2017 13:03:01
* Author : a16007
*/

#include "TLog.h"

uint8_t doLastSensor();
uint8_t doLicht(BH1750 *lichtsensor);
uint8_t doClima();
uint8_t doDruck(SFE_BMP180 *drucksensor);
void init_clock(int sysclk, int pll);


void setup()
{
	PORTA_DIRSET = PIN2_bm | PIN3_bm | PIN4_bm;
	PORTA_OUTSET = 0xff;

	PORTB_DIRSET = 0xff;

	PORTC_DIRSET = PIN1_bm;

	PORTD_DIRSET = PIN0_bm | PIN4_bm | PIN5_bm | PIN7_bm;
	PORTD_DIRCLR = PIN6_bm;
	PORTD_OUTCLR = PIN4_bm | PIN5_bm;

	PORTE_DIRSET = 0xff;

	uint8_t i;
	init_clock(SYSCLK,PLL);

	for(i=0;i<20;i++)
	{
		LED_ROT_TOGGLE;
		_delay_ms(50);
	}

	for(i=0;i<20;i++)
	{
		LED_GRUEN_TOGGLE;
		_delay_ms(50);
	}
	initReadMonitor();
	initBusyCounter();
	//init_mytimer();
	sendFree = true;

	PMIC_CTRL = PMIC_LOLVLEX_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm;
	sei();

	cnet.open(Serial::BAUD_57600,F_CPU);
	debug.open(Serial::BAUD_115200,F_CPU);


}
/*
void setup_twi()
{
	OneWireMaster::CmdResult result = owm.begin(&twiC_Master,0x18);
	cnet.println("Master Ready");
	if(result != OneWireMaster::Success)
	{
		cnet.println("Failed to initialize OneWire Master");
		cnet.println("Ending Program");
		while(1);
	}
}
*/
int main(void)
{
uint8_t reportStarted = false;
	setup();

	init_mytimer();
	TWI_MasterInit(&twiC_Master, &TWIC, TWI_MASTER_INTLVL_LO_gc, TWI_BAUDSETTING);
	#ifdef USE_PRESSURE_SENSOR
        SFE_BMP180 drucksensor(&twiC_Master,BMP180_SLAVE_ADDR);
        drucksensor.measure_pressure(TRAUNSTEIN);
    #endif // USE_PRESSURE_SENSOR
	#ifdef USE_LIGHT_SENSOR
        BH1750 lichtsensor(&twiC_Master,BH1750_SLAVE_ADDR_L);
        lichtsensor.PowerOn();
        lichtsensor.Set_Mode(BH1750_Cont_H_Res);
    #endif // USE_LIGHT_SENSOR

  char buffer[50];
  sprintf(buffer,"--%x--%x--",DFLLRC32M.CALA,DFLLRC32M.CALB);
  cnet.sendStandard(buffer,BROADCAST,'C','1','d','F');
	uint8_t sensorReady=SENSOR_READY;
	while (1)
	{
		comStateMachine(&cnet);
		doJob(&cnet);
		switch(statusSensoren)
		{
			case KLIMASENSOR:
				LED_ROT_ON;
				LED_GELB_OFF;
				LED_GRUEN_OFF;
				sensorReady = doClima();
			break;
			case DRUCKSENSOR:
				LED_ROT_OFF;
				LED_GELB_ON;
				LED_GRUEN_OFF;
				#ifdef USE_PRESSURE_SENSOR
                    sensorReady = doDruck(&drucksensor);
                #else
                    sensorReady = SENSOR_READY;
                #endif // USE_PRESSURE_SENSOR
			break;
			case LICHTSENSOR:
				LED_ROT_OFF;
				LED_GELB_OFF;
				LED_GRUEN_ON;
				LED_GRUEN_OFF;
				#ifdef USE_LIGHT_SENSOR
                    sensorReady = doLicht(&lichtsensor);
                #else
                    sensorReady = SENSOR_READY;
                #endif // USE_PRESSURE_SENSOR
			break;
			case LASTSENSOR:
				LED_ROT_OFF;
				LED_GELB_ON;
				LED_GRUEN_ON;
				sensorReady = doLastSensor();
			break;
		}
		if (sensorReady==SENSOR_READY)
		{
			statusSensoren++;
			if (statusSensoren>LASTSENSOR)
			{
				statusSensoren = KLIMASENSOR;
				if(reportStarted==false)
                {
                    MyTimers[TIMER_REPORT].state = TM_START;
                    reportStarted = true;
                }
			}
		}
		if( sendStatusReport )
        {
            char buffer[16];
            sendStatusReport = false;
            MyTimers[TIMER_REPORT].value = REPORT_BETWEEN_SENSORS;
            MyTimers[TIMER_REPORT].state = TM_START;
            switch(statusReport)
            {
                case TEMPREPORT:
                    sprintf(buffer,"%f",(double)fTemperatur);
                    cnet.sendStandard(buffer,BROADCAST,'C','1','t','F');
                break;
                case HUMIREPORT:
                    sprintf(buffer,"%f",(double)fHumidity);
                    cnet.sendStandard(buffer,BROADCAST,'C','1','h','F');
                break;
                case ABSHUMIREPORT:
                    sprintf(buffer,"%f",(double)fAbsHumitdity);
                    cnet.sendStandard(buffer,BROADCAST,'C','1','a','F');
                break;
                case DEWPOINTREPORT:
                    sprintf(buffer,"%f",(double)fDewPoint);
                    cnet.sendStandard(buffer,BROADCAST,'C','1','d','F');
                break;
                #ifdef USE_PRESSURE_SENSOR
                case DRUCKREPORT:
                    sprintf(buffer,"%f",(double)dPressure);
                    cnet.sendStandard(buffer,BROADCAST,'C','1','p','F');
                break;
                #endif // DRUCKREPORT
				#ifdef USE_LIGHT_SENSOR
                case LICHTREPORT:
                    sprintf(buffer,"%f",(double)uLicht);
                    cnet.sendStandard(buffer,BROADCAST,'C','1','l','F');
                break;
                #endif // USE_LIGHT_SENSOR
                case LASTREPORT:
                    MyTimers[TIMER_REPORT].value = REPORT_BETWEEN_BLOCKS;
                    MyTimers[TIMER_REPORT].state = TM_START;
                break;
            }
        }
	}
}

uint8_t doLastSensor()
{
	switch( statusLastSensor )
	{
		case NOTHING_LAST_TODO:
			MyTimers[TIMER_TEMPERATURE].value = 100;
			MyTimers[TIMER_TEMPERATURE].state = TM_START;
			statusLastSensor = WAIT_LAST;
		break;
		case READY_LAST:
			statusLastSensor = NOTHING_LAST_TODO;
		break;
	}
	return statusLastSensor;
}

uint8_t doDruck(SFE_BMP180 *drucksensor)
{
	static double Temperature,Pressure;
	switch(statusDruck)
	{
		case NOTHING_BMP_TODO:
			statusDruck = START_BMP_TCONVERSION;
		break;
		case START_BMP_TCONVERSION:
			if (drucksensor->startTemperature()>0)
			{
				statusDruck = WAIT_BMP_TCONVERSION;
				MyTimers[TIMER_TEMPERATURE].value = 2; // 1
				MyTimers[TIMER_TEMPERATURE].state = TM_START;
			}
			else
				statusDruck = NOTHING_BMP_TODO;
		break;
		case READ_BMP_TCONVERSION:
			if (drucksensor->getTemperature(Temperature))
			{
				statusDruck = START_BMP_PCONVERSION;
			}
			else
				statusDruck = NOTHING_BMP_TODO;
		break;
		case START_BMP_PCONVERSION:
			if (drucksensor->startPressure(3)>0)
			{
				statusDruck = WAIT_BMP_PCONVERSION;
				MyTimers[TIMER_TEMPERATURE].value = 6; //3
				MyTimers[TIMER_TEMPERATURE].state = TM_START;
			}
			else
				statusDruck = NOTHING_BMP_TODO;
		break;
		case READ_BMP_PCONVERSION:
			if (drucksensor->getPressure(Pressure,Temperature))
			{
				statusDruck = CALC_BMP_SEALEVEL;
			}
			else
				statusDruck = NOTHING_BMP_TODO;
		break;
		case CALC_BMP_SEALEVEL:
			dPressure = drucksensor->sealevel(Pressure,dSealevel);
			statusDruck = NOTHING_BMP_TODO;
		break;
	}
	return statusDruck;
}

uint8_t doLicht(BH1750 *lichtsensor)
{
	uLicht = lichtsensor->Get_Data();
	return SENSOR_READY;
}

uint8_t doClima()
{
static unsigned char crcSH11;
uint8_t error;
static unsigned int iTemperature,iHumidity;
	switch(statusKlima)
	{
		case NOTHING_CLIMA_TODO:
			statusKlima = START_TCONVERSION;
		break;
		case START_TCONVERSION: // Durchlaufzeit ca. 55 µs
			LED_ROT_ON;
			crcSH11 = 0;
			error=startConversion(0,&crcSH11);
			if (error==0)
			{
				statusKlima = WAIT_TCONVERSION;
				MyTimers[TIMER_TEMPERATURE].value = 44; //22
				MyTimers[TIMER_TEMPERATURE].state = TM_START;
			}
			else
				statusKlima = NOTHING_CLIMA_TODO;
		break;
		case READ_TCONVERSION:  // Durchlaufzeit ca. 82 µs
			error = readConversion(&iTemperature,&crcSH11);
			if (error==0)
			{
				statusKlima = START_HCONVERSION;
			}
			else
				statusKlima = NOTHING_CLIMA_TODO;
		break;
		case START_HCONVERSION:
			error=startConversion(1,&crcSH11);
			if (error==0)
			{
				statusKlima = WAIT_HCONVERSION;
				MyTimers[TIMER_TEMPERATURE].value = 14; // 7
				MyTimers[TIMER_TEMPERATURE].state = TM_START;
			}
			else
				statusKlima = NOTHING_CLIMA_TODO;
		break;
		case READ_HCONVERSION:
			error = readConversion(&iHumidity,&crcSH11);
			if (error==0)
			{
				statusKlima = CALC_CONVERSION1;
			}
			else
				statusKlima = NOTHING_CLIMA_TODO;
		break;
		case CALC_CONVERSION1:  // Durchlaufzeit ca. 58 µs
			fHumidity = (float)iHumidity;
			fTemperatur = (float)iTemperature;
			calc_sth11(&fHumidity ,&fTemperatur);
			statusKlima = CALC_CONVERSION2;
		break;
		case CALC_CONVERSION2:  // Durchlaufzeit ca. 141
			fDewPoint = calc_dewpoint_float(fHumidity,fTemperatur); // calculate dew point temperature
			statusKlima = CALC_CONVERSION3;
		break;
		case CALC_CONVERSION3:  // Durchlaufzeit ca. 230 µs
			fAbsHumitdity = abs_feuchte(fHumidity,fTemperatur); // calculate dew point temperature
			statusKlima = NOTHING_CLIMA_TODO;
/*			MyTimers[TIMER_TEMPERATURE].value = 100;
			MyTimers[TIMER_TEMPERATURE].state = TM_START;*/
			LED_ROT_OFF;
		break;
	}
	return(statusKlima);
}

//*********************************************************************
/*
void print_rom_id(OneWire::RomId & romId)
{
	//print the rom number
	cnet.print("0x");
	for(uint8_t idx = 0; idx < RomId::byteLen; idx++)
	{
		cnet.pformat("%x",romId[idx]);
	}
}
*/
void init_clock(int sysclk, int pll)
{
	CLK_t *mein_clock;
	OSC_t *mein_osc;
	mein_clock = &CLK;
	mein_osc = &OSC;
	switch(sysclk)
	{
		case QUARZ:
			mein_osc->XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;
//			mein_osc->XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCPWR_bm | OSC_XOSCSEL_XTAL_16KCLK_gc;
			mein_osc->CTRL = OSC_XOSCEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 32 MHz-Clock ein

			while((mein_osc->STATUS & OSC_XOSCRDY_bm) == 0)			// wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)		// wartet bis diese stabil
			;

			if ( (pll>0) & (pll<16) )
			{
				mein_osc->PLLCTRL = OSC_PLLSRC_XOSC_gc | pll;
				mein_osc->CTRL = OSC_PLLEN_bm | OSC_XOSCEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet zusätzlich die PLL ein

				while((mein_osc->STATUS & OSC_PLLRDY_bm) == 0)		// wartet bis diese stabil
				;
				CCP = CCP_IOREG_gc;										// geschuetztes Register freigeben
				mein_clock->CTRL = CLK_SCLKSEL_PLL_gc;					// umschalten auf PLL-Clock
				mein_osc->CTRL = OSC_PLLEN_bm | OSC_XOSCEN_bm | OSC_RC32KEN_bm;
			}
			else
			{
				CCP = CCP_IOREG_gc;										// geschuetztes Register freigeben
				mein_clock->CTRL = CLK_SCLKSEL_XOSC_gc;					// umschalten auf XOSC-Clock
				mein_osc->CTRL = OSC_XOSCEN_bm | OSC_RC32KEN_bm;
			}
		break; // QUARZ
		case CLK2M:
			mein_osc->CTRL = OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 2 MHz-Clock ein
			while((mein_osc->STATUS & OSC_RC2MRDY_bm) == 0)  // wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)  // wartet bis diese stabil
			;
			CCP = CCP_IOREG_gc;								// geschuetztes Register freigeben
			mein_clock->CTRL = CLK_SCLKSEL_RC2M_gc;		// umschalten auf 2 MHz-Clock
//			CLKSYS_AutoCalibration_Enable(OSC_RC2MCREF_RC32K_gc,false); // OSC_RC32MCREF_bm
		break;
		case CLK32M:
      DFLLRC32M.CALA = 0x29;  // original 0x40
      DFLLRC32M.CALB = 0x0d;  // original 0x0d
			mein_osc->CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 32 MHz-Clock ein
			while((mein_osc->STATUS & OSC_RC32MRDY_bm) == 0)  // wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)  // wartet bis diese stabil
			;
			CCP = CCP_IOREG_gc;								// geschuetztes Register freigeben
			mein_clock->CTRL = CLK_SCLKSEL_RC32M_gc;		// umschalten auf 32 MHz-Clock
			mein_osc->CTRL = OSC_RC32MEN_bm | OSC_RC32KEN_bm;		// abschalten der 2 MHz-Clock
//			CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_RC32K_gc,false); // OSC_RC32MCREF_bm
		break;
	}
}

/*! \brief This function enables automatic calibration of the selected internal
 *         oscillator.
 *
 *  Either the internal 32kHz RC oscillator or an external 32kHz
 *  crystal can be used as a calibration reference. The user must make sure
 *  that the selected reference is ready and running.
 *
 *  \param  clkSource    Clock source to calibrate, either OSC_RC2MCREF_bm or
 *                       OSC_RC32MCREF_bm.
 *  \param  extReference True if external crystal should be used as reference.
 */
/*
void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, bool extReference )
{
	OSC.DFLLCTRL = ( OSC.DFLLCTRL & ~clkSource ) |
	               ( extReference ? clkSource : 0 );
	if (clkSource == OSC_RC2MCREF_bm) {
		DFLLRC2M.CTRL |= DFLL_ENABLE_bm;
	} else if (clkSource == OSC_RC32MCREF_RC32K_gc) {   // OSC_RC32MCREF_bm
		DFLLRC32M.CTRL |= DFLL_ENABLE_bm;
	}
}
*/
