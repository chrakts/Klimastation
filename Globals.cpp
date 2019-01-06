/*
 * Globals.cpp
 *
 * Created: 19.03.2017 09:24:37
 *  Author: Christof
 */
#define EXTERNALS_H_

#include "TLog.h"

//using namespace OneWire;
//using namespace RomCommands;


#define CQ "CQ"
#define C1 "C1"

const char *Node = NODE_STRING;
char IDNumber[12] EEMEM = "1784324-01";
char SerialNumber[12] EEMEM = "1958632254";
char IndexNumber[2] EEMEM = "A";


const char *fehler_text[]={"memory errors","parameter error","unknown job","no transmission","command not allowed","CRC error"};

char Compilation_Date[] = __DATE__;
char Compilation_Time[] = __TIME__;

//volatile uint8_t UART0_ring_received;
//volatile char UART0_ring_buffer[UART0_RING_BUFFER_SIZE];
//volatile uint8_t UART1_ring_received;
//volatile char UART1_ring_buffer[UART1_RING_BUFFER_SIZE];

volatile TIMER MyTimers[MYTIMER_NUM]= {	{TM_START,RESTART_YES,100,0,nextTemperatureStatus},
                                        {TM_STOP,RESTART_YES,REPORT_BETWEEN_SENSORS,0,nextReportStatus},
										{TM_STOP,RESTART_NO,100,0,NULL}		// Timeout-Timer
};

//TempSensor *activeSensor=NULL;

float fTemperatur=-999,fHumidity=-999,fDewPoint=-999,fAbsHumitdity=-999;
double dPressure = -999, dSealevel = TRAUNSTEIN;
uint16_t uLicht = 65535;

volatile uint8_t statusSensoren = KLIMASENSOR;
volatile uint8_t statusReport = TEMPREPORT;
volatile bool    sendStatusReport = false;
volatile uint8_t statusKlima = NOTHING_CLIMA_TODO;
volatile uint8_t statusDruck = NOTHING_BMP_TODO;
volatile uint8_t statusLicht = NOTHING_BH_TODO;
volatile uint8_t statusLastSensor = NOTHING_LAST_TODO;

int errno;      // Globale Fehlerbehandlung

char SecurityLevel = 0;

//DS2484 owm;
//TempSensor *tempSensors[NUMBER_OF_TEMPSENSORS];
uint8_t actNumberSensors = 0;
/* Global variables for TWI */
TWI_Master_t twiC_Master;    /*!< TWI master module. */
TWI_Master_t twiE_Master;    /*!< TWI master module. */


/*
// SENSINFOS storedSensors[NUMBER_STORED_SENSORS] EEMEM ={
// 	{{0x28,0x9C,0xDF,0x36,0x09,0x00,0x00,0x2C},0,12,"FirstSensor",{0.0,1.0,0.0,0.0}} ,
// 	{{0x28,0x62,0xD2,0x4B,0x01,0x00,0x00,0xB1},1,12,"Secondo",{0.0,1.0,0.0,0.0}},
// 	{{0x28,0x19,0xAC,0xBF,0x03,0x00,0x00,0x2E},2,12,"LastSensor",{0.0,1.0,0.0,0.0}}
// };
*/

//SENSINFOS storedSensors[NUMBER_STORED_SENSORS] EEMEM;

//volatile uint8_t sendFree;
volatile bool nextSendReady=false;

Serial debug(0);
 Communication cnet(1,Node,5);

//Serial cnet(1);

//CRC_Calc crcGlobal;

