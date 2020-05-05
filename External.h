/*
 * External.h
 *
 * Created: 03.04.2017 21:04:41
 *  Author: Christof
 */



#ifndef EXTERNAL_H_
#define EXTERNAL_H_

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
#include "myconstants.h"
//#include "TempSensor.h"
#include "MyTimer.h"
#include "CRC_Calc.h"
#include "SFE_BMP180.h"
#include "bh1750.h"
#include "Communication.h"

extern volatile uint8_t UART0_ring_received;
extern volatile  char UART0_ring_buffer[UART0_RING_BUFFER_SIZE];
extern volatile uint8_t UART1_ring_received;
extern volatile  char UART1_ring_buffer[UART1_RING_BUFFER_SIZE];

extern char const *Node;
extern char IDNumber[12] EEMEM ;
extern char SerialNumber[12] EEMEM;
extern char IndexNumber[2] EEMEM;

//extern OneWire::DS2484 owm;
extern SFE_BMP180 drucksensor;
extern BH1750 lichtsensor;

extern uint16_t actReportBetweenBlocks;
extern uint16_t actReportBetweenSensors;
extern uint16_t actWaitAfterLastSensor;

extern float fTemperatur,fHumidity,fDewPoint,fAbsHumitdity;
extern double dPressure , dSealevel ;
extern uint16_t uLicht;

//extern TempSensor *tempSensors[NUMBER_OF_TEMPSENSORS];
//extern SENSINFOS storedSensors[NUMBER_STORED_SENSORS] EEMEM;
extern const char *fehler_text[];
extern uint8_t actNumberSensors;
extern volatile TIMER MyTimers[MYTIMER_NUM];
extern volatile uint8_t statusReport;
extern volatile bool sendStatusReport;

extern volatile uint8_t statusKlima;
extern volatile uint8_t statusDruck;
extern volatile uint8_t statusSensoren;
extern volatile uint8_t statusLicht;
extern volatile uint8_t statusLastSensor;

extern char SecurityLevel;
//extern TempSensor *activeSensor;

/* Global variables for TWI */
extern TWI_Master_t twiC_Master;    /*!< TWI master module. */
extern TWI_Master_t twiE_Master;    /*!< TWI master module. */
//extern Serial myout;
//extern Serial cnet;
class Communication;   // Forward declaration
class ComReceiver;
extern Serial debug;
extern Communication cnet;
extern ComReceiver cnetRec;
extern CRC_Calc crcGlobal;

extern volatile uint8_t sendFree;
extern volatile bool nextSendReady;

#endif /* EXTERNAL_H_ */
