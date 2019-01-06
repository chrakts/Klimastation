/*
 * constants.h
 *
 * Created: 24.05.2018 19:24:50
 *  Author: chrak_2
 */


#ifndef CONSTANTS_H_
#define CONSTANTS_H_

//#include "RomId/RomId.h"
//using namespace OneWire;


enum{QUARZ,CLK2M,CLK32M};




//#define UART0_RING_BUFFER_SIZE 36
//#define UART1_RING_BUFFER_SIZE 36

#define NUMBER_OF_TEMPSENSORS 10

#define NUMBER_OF_SENSORS_NAME_LENGTH 12

#define LED_ROT_ON		PORTA_OUTCLR = PIN2_bm
#define LED_ROT_OFF		PORTA_OUTSET = PIN2_bm
#define LED_ROT_TOGGLE	PORTA_OUTTGL = PIN2_bm
#define LED_GELB_ON			PORTA_OUTCLR = PIN3_bm
#define LED_GELB_OFF		PORTA_OUTSET = PIN3_bm
#define LED_GELB_TOGGLE		PORTA_OUTTGL = PIN3_bm
#define LED_GRUEN_ON		PORTA_OUTCLR = PIN4_bm
#define LED_GRUEN_OFF		PORTA_OUTSET = PIN4_bm
#define LED_GRUEN_TOGGLE	PORTA_OUTTGL = PIN4_bm

#define TRAUNSTEIN			591.0  // sealevel of Traunstein
/*
struct SensInfos
{
	RomId romID;
	uint8_t number;
	uint16_t temperature_bits;
	char name[NUMBER_OF_SENSORS_NAME_LENGTH];
	double caliCoefficients[4];
};
*/
#define NUMBER_STORED_SENSORS NUMBER_OF_TEMPSENSORS

#define BROADCAST "BR"

typedef struct SensInfos SENSINFOS;

#define SENSOR_READY 0

enum{NOTHING_CLIMA_TODO=SENSOR_READY,
	START_TCONVERSION,WAIT_TCONVERSION,READ_TCONVERSION,
	START_HCONVERSION,WAIT_HCONVERSION,READ_HCONVERSION,
CALC_CONVERSION1,CALC_CONVERSION2,CALC_CONVERSION3};

enum{NOTHING_BMP_TODO=SENSOR_READY,
	START_BMP_TCONVERSION,WAIT_BMP_TCONVERSION,READ_BMP_TCONVERSION,
	START_BMP_PCONVERSION,WAIT_BMP_PCONVERSION,READ_BMP_PCONVERSION,
CALC_BMP_SEALEVEL};

enum{NOTHING_BH_TODO=SENSOR_READY,START_BH_GETLIGHT};

enum{NOTHING_LAST_TODO=SENSOR_READY,WAIT_LAST,READY_LAST};

enum{KLIMASENSOR=0,DRUCKSENSOR,LICHTSENSOR,LASTSENSOR};

#define REPORT_BETWEEN_SENSORS 20
#define REPORT_BETWEEN_BLOCKS  3000

enum{TEMPREPORT=0,HUMIREPORT,ABSHUMIREPORT,DEWPOINTREPORT,DRUCKREPORT,LICHTREPORT,LASTREPORT};

enum{MEMORY_ERROR,PARAMETER_ERROR,UNKNOWN_ERROR,TRANSMISSION_ERROR,SECURITY_ERROR,CRC_ERROR};

enum{CRC_NIO,CRC_IO,CRC_NO,CRC_YES};



#endif /* CONSTANTS_H_ */