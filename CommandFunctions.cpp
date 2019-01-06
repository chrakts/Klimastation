/*
 * CommandFunctions.cpp
 *
 * Created: 26.04.2017 14:54:45
 *  Author: a16007
 */

#include "CommandFunctions.h"
#include "External.h"
#include "CRC_Calc.h"

void jobGotCRCError(Communication *output, char function,char address,char job, void * pMem)
{
	sendAnswer(output,function,address,job,fehler_text[CRC_ERROR],false);
}

void jobSetIDNumber(Communication *output, char function,char address,char job, void * pMem)
{
	if (strlen((char*)pMem)<=11)
	{
		eeprom_write_block((char*)pMem,IDNumber,strlen((char*)pMem)+1);
		sendPureAnswer(output,function,address,job,true);
	}
	else
		sendPureAnswer(output,function,address,job,false);
}

void jobSetSerialNumber(Communication *output, char function,char address,char job, void * pMem)
{
	if (strlen((char*)pMem)<=11)
	{
		eeprom_write_block((char*)pMem,SerialNumber,strlen((char*)pMem)+1);
		sendPureAnswer(output,function,address,job,true);
	}
	else
	sendPureAnswer(output,function,address,job,false);
}

void jobSetIndexNumber(Communication *output, char function,char address,char job, void * pMem)
{
	if (strlen((char*)pMem)<=2)
	{
		eeprom_write_block((char*)pMem,IndexNumber,strlen((char*)pMem)+1);
		sendPureAnswer(output,function,address,job,true);
	}
	else
		sendPureAnswer(output,function,address,job,false);
}

void jobGetIDNumber(Communication *output, char function,char address,char job, void * pMem)
{
	char temp[12];
	eeprom_read_block(temp,IDNumber,12);
	sendAnswer(output,function,address,job,temp,true);
}

void jobGetSerialNumber(Communication *output, char function,char address,char job, void * pMem)
{
	char temp[12];
	eeprom_read_block(temp,SerialNumber,12);
	sendAnswer(output,function,address,job,temp,true);
}

void jobGetIndex(Communication *output, char function,char address,char job, void * pMem)
{
	char temp[2];
	eeprom_read_block(temp,IndexNumber,2);
	sendAnswer(output,function,address,job,temp,true);
}

void jobGetCTemperatureSensor(Communication *output, char function,char address,char job, void * pMem)
{
char answer[20]="";

	sprintf(answer,"%f",(double)fTemperatur);
	sendAnswer(output,function,address,job,answer,true);
}

void jobGetCHumiditySensor(Communication *output, char function,char address,char job, void * pMem)
{
	char answer[20]="";

	sprintf(answer,"%f",(double)fHumidity);
	sendAnswer(output,function,address,job,answer,true);
}

void jobGetCAbsHumiditySensor(Communication *output, char function,char address,char job, void * pMem)
{
	char answer[20]="";

	sprintf(answer,"%f",(double)fAbsHumitdity);
	sendAnswer(output,function,address,job,answer,true);
}

void jobGetCDewPointSensor(Communication *output, char function,char address,char job, void * pMem)
{
	char answer[20]="";

	sprintf(answer,"%f",(double)fDewPoint);
	sendAnswer(output,function,address,job,answer,true);
}
void jobGetPressure(Communication *output, char function,char address,char job, void * pMem)
{
	char answer[20]="";

	sprintf(answer,"%f",dPressure);
	sendAnswer(output,function,address,job,answer,true);
}

void jobSetSealevel(Communication *output, char function,char address,char job, void * pMem)
{
	double *pointer;
	double temp;
	pointer = (double*) pMem;
	temp = pointer[0];
	if ( (temp>-100.0) && (temp < 9000.0) )
	{
		dSealevel = temp;
		sendPureAnswer(output,function,address,job,true);
	}
	else
		sendPureAnswer(output,function,address,job,false);
}

void jobGetSealevel(Communication *output, char function,char address,char job, void * pMem)
{
	char answer[20]="";
	sprintf(answer,"%f",dSealevel);
	sendAnswer(output,function,address,job,answer,true);
}

void jobGetLight(Communication *output, char function,char address,char job, void * pMem)
{
	char answer[20]="";
	sprintf(answer,"%d",uLicht);
	sendAnswer(output,function,address,job,answer,true);
}

void jobSetSecurityKey(Communication *output, char function,char address,char job, void * pMem)
{
uint8_t ret = true;
	if (strcmp((char *)pMem,"Phe6%!kdf?+2aQ")==0)
	{
		SecurityLevel = PRODUCTION;
	}
	else if(strcmp((char *)pMem,"D=&27ane%24dez")==0)
	{
		SecurityLevel = DEVELOPMENT;
	}
	else
	{
		SecurityLevel = CUSTOMER;
		ret = false;
	}
	sendAnswerInt(output,function,address,job,SecurityLevel,ret);
}

void jobGetSecurityKey(Communication *output, char function,char address,char job, void * pMem)
{
	sendAnswerInt(output,function,address,job,SecurityLevel,true);
}

void jobTestTripleIntParameter(Communication *output, char function,char address,char job, void * pMem)
{
	uint16_t *pointer;
	sendAnswer(output,function,address,job,"Parameter: ",true);
	pointer = (uint16_t*) pMem;
	sendAnswerInt(output,function,address,job,pointer[0],true);
	sendAnswerInt(output,function,address,job,pointer[1],true);
	sendAnswerInt(output,function,address,job,pointer[2],true);
}

void jobTestStringParameter(Communication *output, char function,char address,char job, void * pMem)
{
CRC_Calc mycrc;

	mycrc.String((const char*) pMem);
	sendAnswerInt(output,function,address,job,mycrc.Get_CRC(),true);
}

void jobTestFloatParameter(Communication *output, char function,char address,char job, void * pMem)
{
	double *pointer;
	sendAnswer(output,function,address,job,"Parameter: ",true);
	pointer = (double*) pMem;
	sendAnswerDouble(output,function,address,job,pointer[0],true);
}

void jobTestParameter(Communication *output, char function,char address,char job, void * pMem)
{
	uint16_t *pointer;
	sendAnswer(output,function,address,job,"Parameter: ",true);
	pointer = (uint16_t*) pMem;
	sendAnswerInt(output,function,address,job,pointer[0],true);
}

void jobGetCompilationDate(Communication *output, char function,char address,char job, void * pMem)
{
char temp[20];
	strcpy(temp,Compilation_Date);
	sendAnswer(output,function,address,job,temp,true);
}

void jobGetCompilationTime(Communication *output, char function,char address,char job, void * pMem)
{
char temp[20];
	strcpy(temp,Compilation_Time);
	sendAnswer(output,function,address,job,temp,true);
}

void jobGetFreeMemory(Communication *output, char function,char address,char job, void * pMem)
{
extern int __heap_start, *__brkval;
int v;
	char answer[15];
	sprintf(answer,"%d",(int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
	sendAnswer(output,function,address,job,answer,true);
}

