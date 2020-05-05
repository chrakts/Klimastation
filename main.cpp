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

void setup()
{
  init_clock(SYSCLK,PLL,true,CLOCK_CALIBRATION);
	PORTA_DIRSET = PIN2_bm | PIN3_bm | PIN4_bm;
	PORTA_OUTSET = 0xff;

	PORTB_DIRSET = 0xff;

	PORTC_DIRSET = PIN1_bm;

	PORTD_DIRSET = PIN0_bm | PIN4_bm | PIN5_bm | PIN7_bm;
	PORTD_DIRCLR = PIN6_bm;
	PORTD_OUTCLR = PIN4_bm | PIN5_bm;

	PORTE_DIRSET = 0xff;

	uint8_t i;
	//init_clock(SYSCLK,PLL);

	for(i=0;i<20;i++)
	{
		LED_GRUEN_TOGGLE;
		_delay_ms(50);
	}
	//init_mytimer();

	PMIC_CTRL = PMIC_LOLVLEX_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm;
	sei();

	cnet.open(Serial::BAUD_57600,F_CPU);
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

  #if BOARD_VERSION==0
    cnet.sendInfo("Altes Klimaboard kaputt?","BR");
  #else
    cnet.sendInfo("Neues Klimaboard","BR");
  #endif // BOARD_VERSION

	uint8_t sensorReady=SENSOR_READY;
	while (1)
	{
		cnetRec.comStateMachine();
		cnetRec.doJob();
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
            MyTimers[TIMER_REPORT].value = actReportBetweenSensors;
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
                    MyTimers[TIMER_REPORT].value = actReportBetweenBlocks;
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
			MyTimers[TIMER_TEMPERATURE].value = actWaitAfterLastSensor;
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
		case START_TCONVERSION: // Durchlaufzeit ca. 55 탎
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
		case READ_TCONVERSION:  // Durchlaufzeit ca. 82 탎
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
		case CALC_CONVERSION1:  // Durchlaufzeit ca. 58 탎
			fHumidity = (float)iHumidity;
			fTemperatur = (float)iTemperature;
			calc_sth11(&fHumidity ,&fTemperatur);
			statusKlima = CALC_CONVERSION2;
		break;
		case CALC_CONVERSION2:  // Durchlaufzeit ca. 141
			fDewPoint = calc_dewpoint_float(fHumidity,fTemperatur); // calculate dew point temperature
			statusKlima = CALC_CONVERSION3;
		break;
		case CALC_CONVERSION3:  // Durchlaufzeit ca. 230 탎
			fAbsHumitdity = abs_feuchte(fHumidity,fTemperatur); // calculate dew point temperature
			statusKlima = NOTHING_CLIMA_TODO;
/*			MyTimers[TIMER_TEMPERATURE].value = 100;
			MyTimers[TIMER_TEMPERATURE].state = TM_START;*/
			LED_ROT_OFF;
		break;
	}
	return(statusKlima);
}

