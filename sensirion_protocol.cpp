/*********************************************
 * vim:sw=8:ts=8:si:et
 * This is the driver code for the sensirion temperature and
 * humidity sensor.
 *
 * Based on ideas from the sensirion application note and modified
 * for atmega88/168.
 * A major part of the code was optimized and as a result the compiled
 * code size was reduced to 50%.
 *
 * Modifications by: Guido Socher
 *
 * Note: the sensirion SHTxx sensor does _not_ use the industry standard
 * I2C. The sensirion protocol looks similar but the start/stop
 * sequence is different. You can not use the avr TWI module.
 *
 *********************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include "sensirion_protocol.h"

//static int		humival=550,tempval=253,dewval=0,abs_hval=0;
static int		tempval=253;

uint8_t make_measure_float(float * fTemp,float * fHumi,float * fDew,float * fabsHumi)
{
	unsigned int humival_raw,tempval_raw;
//	float humifloat,tempfloat,dew,abs_h;
	uint8_t error;
	//int temp;
	//unsigned char rh;

	//measure temperature
	error=s_measure( &tempval_raw,0);
	if (error==0)
	{
		error=s_measure( &humival_raw,1); //measure humidity
	}
	*fHumi = (float)humival_raw;
	*fTemp = (float)tempval_raw;
	if(error!=0)
		tempval = 999;

	// measurement was OK (you could use error to detect if there
	// is no sensor attached of if there is a CRC data integrity problem.
	// We don't do this here to keep the example short).
	if (error==0)
	{
		calc_sth11(fHumi ,fTemp);
		*fDew = calc_dewpoint_float(*fHumi,*fTemp); // calculate dew point temperature
		*fabsHumi = abs_feuchte(*fHumi,*fTemp); // calculate dew point temperature
/*		tempval = (int)(10*tempfloat);
		humival = (int)(10*humifloat);
		dewval  = (int)(10*dew);
		abs_hval  = (int)(10*abs_h);*/
	}
	//	output_print(UART0,"T=%d °C - rH=%d - Tc=%d - aF=%d - Error=%d\n\r",(int)(tempval*10),(int)(humival*10),(int)(dew*10),(int)(abs_h*100),error);

	//
	// That's it. Now you have temperature in Celsius * 10 in temp (e.g 251 for
	// 25.1'C), dew-point in dew and relative humidity (values 0-100 %) in rh.
	return(error);
}


// Compute the CRC8 value of a data set.
//
//  This function will compute the CRC8 of inData using seed
//  as inital value for the CRC.
//
//  This function was copied from Atmel avr318 example files.
//  It is more suitable for microcontroller than the code example
//  in the sensirion CRC application note.
//
//  inData  One byte of data to compute CRC from.
//
//  seed    The starting value of the CRC.
//
//  return The CRC8 of inData with seed as initial value.
//
//  note   Setting seed to 0 computes the crc8 of the inData.
//
//  note   Constantly passing the return value of this function
//         As the seed argument computes the CRC8 value of a
//         longer string of data.
//
unsigned char computeCRC8(unsigned char inData, unsigned char seed)
{
    unsigned char bitsLeft;
    unsigned char tmp;

    for (bitsLeft = 8; bitsLeft > 0; bitsLeft--)
    {
        tmp = ((seed ^ inData) & 0x01);
        if (tmp == 0)
        {
            seed >>= 1;
        }
        else
        {
            seed ^= 0x18;
            seed >>= 1;
            seed |= 0x80;
        }
        inData >>= 1;
    }
    return seed;
}

// sensirion has implemented the CRC the wrong way round. We
// need to swap everything.
// bit-swap a byte (bit7->bit0, bit6->bit1 ...)
unsigned char bitswapbyte(unsigned char byte)
{
        unsigned char i=8;
        unsigned char result=0;
        while(i){
		result=(result<<1);
                if (1 & byte) {
			result=result | 1;
                }
		i--;
		byte=(byte>>1);
        }
	return(result);
}

// writes a byte on the Sensibus and checks the acknowledge
char s_write_byte(unsigned char value)
{
        unsigned char i=0x80;
        unsigned char error=0;
        DMODEOU;
        while(i){ //shift bit for masking
                if (i & value) {
                        SETDAT1; //masking value with i , write to SENSI-BUS
                }else{
                        SETDAT0;
                }
                SETSCK1; //clk for SENSI-BUS
                S_PULSLONG;
                SETSCK0;
                S_PULSSHORT;
                i=(i>>1);
        }
        DMODEIN; //release DATA-line
        PULLUP1;
        SETSCK1; //clk #9 for ack
        S_PULSLONG;
        if (GETDATA){ //check ack (DATA will be pulled down by SHT11)
                error=1;
        }
        S_PULSSHORT;
        SETSCK0;
        return(error); //error=1 in case of no acknowledge
}

// reads a byte form the Sensibus and gives an acknowledge in case of "ack=1"
// reversebits=1 caused the bits to be reversed (bit0=bit7, bit1=bit6,...)
unsigned char s_read_byte(unsigned char ack)
{
        unsigned char i=0x80;
        unsigned char val=0;
        DMODEIN; //release DATA-line
        PULLUP1;
        while(i){ //shift bit for masking
                SETSCK1; //clk for SENSI-BUS
                S_PULSSHORT;
                if (GETDATA){
                        val=(val | i); //read bit
                }
                SETSCK0;
                S_PULSSHORT;
                i=(i>>1);
        }
        DMODEOU;
        if (ack){
                //in case of "ack==1" pull down DATA-Line
                SETDAT0;
        }else{
                SETDAT1;
        }
        SETSCK1; //clk #9 for ack
        S_PULSLONG;
        SETSCK0;
        S_PULSSHORT;
        DMODEIN; //release DATA-line
        PULLUP1;
        return (val);
}

// generates a sensirion specific transmission start
// This is the point where sensirion is not I2C standard conform and the
// main reason why the AVR TWI hardware support can not be used.
//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
void s_transstart(void)
{
        //Initial state
        SCKOUTP;
        SETSCK0;
        DMODEOU;
        SETDAT1;
        //
        S_PULSSHORT;
        SETSCK1;
        S_PULSSHORT;
        SETDAT0;
        S_PULSSHORT;
        SETSCK0;
        S_PULSLONG;
        SETSCK1;
        S_PULSSHORT;
        SETDAT1;
        S_PULSSHORT;
        SETSCK0;
        S_PULSSHORT;
        //
        DMODEIN; //release DATA-line
        PULLUP1;
}

// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//      _____________________________________________________         ________
// DATA:                                                     |_______|
//          _    _    _    _    _    _    _    _    _        ___    ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|  |___|   |______
void s_connectionreset(void)
{
        unsigned char i;
        //Initial state
        SCKOUTP;
        SETSCK0;
        DMODEOU;
        SETDAT1;
        for(i=0;i<9;i++){ //9 SCK cycles
                SETSCK1;
                S_PULSLONG;
                SETSCK0;
                S_PULSLONG;
        }
        s_transstart(); //transmission start
}

// resets the sensor by a softreset
char s_softreset(void)
{
        s_connectionreset(); //reset communication
        //send RESET-command to sensor:
        return (s_write_byte(RESET)); //return=1 in case of no response form the sensor
}


// makes a measurement (humidity/temperature) with checksum
// p_value returns 2 bytes
// mode: 1=humidity  0=temperature
// return value: 1=write error, 2=crc error, 3=timeout
//
char s_measure(unsigned int *p_value, unsigned char mode)
{
        unsigned char i=0;
        unsigned char msb,lsb;
        unsigned char checksum;
        // the crc8 is computed over the entire communication from command to response data
        unsigned char crc_state=0;
        *p_value=0;
        s_transstart(); //transmission start
        if(mode){
                mode=MEASURE_HUMI;
        }else{
                mode=MEASURE_TEMP;
        }
        if (s_write_byte(mode)){
                return(1);
        }
        crc_state=computeCRC8(bitswapbyte(mode),crc_state);
        // normal delays: temp i=70, humi i=20
        while(i<240){
                _delay_ms(3.0);
                if (GETDATA==0){
                        i=0;
                        break;
                }
                i++;
        }
        if(i){
                // or timeout
                return(3);
        }
        msb=s_read_byte(1); //read the first byte (MSB)
        crc_state=computeCRC8(bitswapbyte(msb),crc_state);
        lsb=s_read_byte(1); //read the second byte (LSB)
        *p_value=(msb<<8)|(lsb);
        crc_state=computeCRC8(bitswapbyte(lsb),crc_state);
        checksum =s_read_byte(0); //read checksum
        if (crc_state != checksum ) {
                return(2);
        }
        return(0);
}

uint8_t startConversion(unsigned char mode,unsigned char *crc_state)
{
// the crc8 is computed over the entire communication from command to response data
	*crc_state=0;
    s_transstart(); //transmission start
    if(mode)
	    mode=MEASURE_HUMI;
	else
	    mode=MEASURE_TEMP;
    if (s_write_byte(mode))
	{
	    return(1);
    }
	else
	{
		*crc_state=computeCRC8(bitswapbyte(mode),*crc_state);
		return(0);
	}
}

uint8_t readConversion( unsigned int *p_value, unsigned char *crc_state )
{
unsigned char msb,lsb;
unsigned char checksum;
	msb=s_read_byte(1); //read the first byte (MSB)
	*crc_state=computeCRC8(bitswapbyte(msb),*crc_state);
	lsb=s_read_byte(1); //read the second byte (LSB)
	*p_value=(msb<<8)|(lsb);
	*crc_state=computeCRC8(bitswapbyte(lsb),*crc_state);
	checksum =s_read_byte(0); //read checksum
	if (*crc_state != checksum )
		return(2);
	else
		return(0);
}

//----------------------------------------------------------------------------------------
void calc_sth11(float *p_humidity ,float *p_temperature)
//----------------------------------------------------------------------------------------
// calculates temperature [C] and humidity [%RH]
// input : humi [Ticks] (12 bit)
// temp [Ticks] (14 bit)
// output: humi [%RH]
// temp [C]
{
/*const float C1=-4.0 ; // for 12 Bit
const float C2= 0.0405; // for 12 Bit
const float C3=-0.0000028; // for 12 Bit*/
const float C1=-2.0468 ; // for 12 Bit
const float C2= 0.0367; // for 12 Bit
const float C3=-1.5955e-6; // for 12 Bit
const float T1=0.01; // for 14 Bit  5V
const float T2=0.00008; // for 14 Bit  5V
float rh=*p_humidity; // rh: Humidity [Ticks] 12 Bit
float t=*p_temperature; // t: Temperature [Ticks] 14 Bit
float rh_lin; // rh_lin: Humidity linear
float rh_true; // rh_true: Temperature compensated humidity
float t_C; // t_C : Temperature [C]

	t_C=(-39.65+t*0.01); //calc. Temperature from ticks to [C]

	rh_lin=C3*rh*rh + C2*rh + C1; //calc. Humidity from ticks to [%RH]
	rh_true=(t_C-25)*(T1+T2*rh)+rh_lin; //calc. Temperature compensated humidity [%RH]
	if(rh_true>100)rh_true=100; //cut if the value is outside of
	if(rh_true<0.1)rh_true=0.1; //the physical possible range
	*p_temperature=t_C; //return temperature [C]
	*p_humidity=rh_true; //return humidity[%RH]
}


// calculates temperature [C]
// input : temp [Ticks] (14 bit)
// output: temp [C] times 10 (e.g 253 = 25.3'C)
// Sensor supply voltage: about 3.3V
//
int calc_sth11_temp(unsigned int t)
{
        //t = 10*(t *0.01 - 39.7);
        //or
        //t = t *0.1 - 397;
        t/=10;
        t-=397;
        return(t);
}

// calculates humidity [%RH]
// The relative humitidy is: -0.0000028*s*s + 0.0405*s - 4.0
// but this is very difficult to compute with integer math.
// We use a simpler approach. See sht10_Non-Linearity_Compensation_Humidity_Sensors_E.pdf
//
// output: humi [%RH] (=integer value from 0 to 100)
unsigned char rhcalc_int(unsigned int s)
{
	// s is in the range from 100 to 3340
	unsigned int rh;
	//for s less than 1712: (143*s - 8192)/4096
	//for s greater than 1712: (111*s + 46288)/4096
	//s range: 100<s<3350
        //
	//rh = rel humi * 10
	if (s<1712){
                // div by 4:
		rh=(36*s - 2048)/102;
	}else{
                // div by 8:
		rh=(14*s + 5790)/51;
	}
	// round up as we will cut the last digit to full percent
	rh+=5;
	rh/=10;
        //
	if (rh>98){
		rh=100;
	}
	return((unsigned char)(rh));
}

// calculates humidity [%RH] with temperature compensation
// input : humi [Ticks] (12 bit), temperature in 'C * 100 (e.g 253 for 25.3'C)
// output: humi [%RH] (=integer value from 0 to 100)
unsigned char calc_sth11_humi(unsigned int h, int t)
{
        int rh;
        rh=rhcalc_int(h);
        // now calc. Temperature compensated humidity [%RH]
        // the correct formula is:
        // rh_true=(t/10-25)*(0.01+0.00008*(sensor_val))+rh;
        // sensor_val ~= rh*30
        // we use:
        // rh_true=(t/10-25) * 1/8;
        rh=rh + (t/80 - 3);
        return((unsigned char)rh);
}

// this is an approximation of 100*log10(x) and does not need the math
// library. The error is less than 5% in most cases.
// compared to the real log10 function for 2<x<100.
// input: x=2<x<100
// output: 100*log10(x)
// Idea by Guido Socher
int log10_approx(unsigned char x)
{
	int l,log;
	if (x==1){
		return(0);
	}
	if (x<8){
		return(11*x+11);
	}
	//
	log=980-980/x;
	log/=10;
	if (x<31){
		l=19*x;
		l=l/10;
		log+=l-4;
	}else{
		l=67*x;
		l=l/100;
		if (x>51 && x<81){
			log+=l +42;
		}else{
			log+=l +39;
		}
	}
	if (log>200) log=200;
	return(log);
}

//--------------------------------------------------------------------
float calc_dewpoint_float(float h,float t)
//--------------------------------------------------------------------
// calculates dew point
// input: humidity [%RH], temperature [°C]
// output: dew point [°C]
{
float k,dew_point ;
	k = (log10(h)-2)/0.4343 + (17.62*t)/(243.12+t);
	dew_point = 243.12*k/(17.62-k);
	return dew_point;
}


// calculates dew point
// input: humidity [in %RH], temperature [in C times 10]
// output: dew point [in C times 10]
int calc_dewpoint(unsigned char rh,int t)
{
        // we use integer math and everything times 100
        int k,tmp;
        k = (100*log10_approx(rh)-20000)/43;
	// we do the calculations in many steps otherwise the compiler will try
	// to optimize and that creates nonsence as the numbers
	// can get too big or too small.
        tmp=t/10;
        tmp=881*tmp;
        tmp=tmp/(243+t/10);
        k+=tmp*2;
        tmp=1762-k;
        tmp=24310/tmp;
        tmp*=k;
        // dew point temp rounded:
	if (tmp<0){
		tmp-=51;
	}else{
		tmp+=51;
	}
        return (tmp/10);
}

/*
r = relative Luftfeuchte
T = Temperatur in °C
TK = Temperatur in Kelvin (TK = T + 273.15)
TD = Taupunkttemperatur in °C
DD = Dampfdruck in hPa
SDD = Sättigungsdampfdruck in hPa

Parameter:
a = 7.5, b = 237.3 für T >= 0
a = 7.6, b = 240.7 für T < 0 über Wasser (Taupunkt)
a = 9.5, b = 265.5 für T < 0 über Eis (Frostpunkt)

R* = 8314.3 J/(kmol*K) (universelle Gaskonstante)
mw = 18.016 kg (Molekulargewicht des Wasserdampfes)
AF = absolute Feuchte in g Wasserdampf pro m3 Luft

Formeln:

SDD(T) = 6.1078 * 10^((a*T)/(b+T))

DD(r,T) = r/100 * SDD(T)

r(T,TD) = 100 * SDD(TD) / SDD(T)

TD(r,T) = b*v/(a-v) mit v(r,T) = log10(DD(r,T)/6.1078)

AF(r,TK) = 10^5 * mw/R* * DD(r,T)/TK;
AF(TD,TK) = 10^5 * mw/R* * SDD(TD)/TK
*/

float abs_feuchte( float h, float t)
{
const float a = 7.5, b = 237.3 ;
const float R = 8314.3, mw = 18.016;

float DD;
	DD = h/100 * 6.1078 * pow(10,(a*t)/(b+t));
	return(10e5 * mw/R * DD/(t+273.15));
}
