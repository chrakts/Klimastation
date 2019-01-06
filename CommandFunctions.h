/*
 * CommandFunctions.h
 *
 * Created: 26.04.2017 14:55:18
 *  Author: a16007
 */


#ifndef COMMANDFUNCTIONS_H_
#define COMMANDFUNCTIONS_H_

#include "TLog.h"

void jobGotCRCError(Communication *output, char function,char address,char job, void * pMem);
void jobGetCTemperatureSensor(Communication *output, char function,char address,char job, void * pMem);
void jobGetCHumiditySensor(Communication *output, char function,char address,char job, void * pMem);
void jobGetCAbsHumiditySensor(Communication *output, char function,char address,char job, void * pMem);
void jobGetCDewPointSensor(Communication *output, char function,char address,char job, void * pMem);
void jobGetPressure(Communication *output, char function,char address,char job, void * pMem);
void jobSetSealevel(Communication *output, char function,char address,char job, void * pMem);
void jobGetSealevel(Communication *output, char function,char address,char job, void * pMem);
void jobGetLight(Communication *output, char function,char address,char job, void * pMem);
void jobTestTripleIntParameter(Communication *output, char function,char address,char job, void * pMem);
void jobTestStringParameter(Communication *output, char function,char address,char job, void * pMem);
void jobTestFloatParameter(Communication *output, char function,char address,char job, void * pMem);
void jobTestParameter(Communication *output, char function,char address,char job, void * pMem);
void jobGetCompilationDate(Communication *output, char function,char address,char job, void * pMem);
void jobGetCompilationTime(Communication *output, char function,char address,char job, void * pMem);
void jobSetSecurityKey(Communication *output, char function,char address,char job, void * pMem);
void jobGetSecurityKey(Communication *output, char function,char address,char job, void * pMem);
void jobGetFreeMemory(Communication *output, char function,char address,char job, void * pMem);
void jobGetIDNumber(Communication *output, char function,char address,char job, void * pMem);
void jobGetSerialNumber(Communication *output, char function,char address,char job, void * pMem);
void jobGetIndex(Communication *output, char function,char address,char job, void * pMem);
void jobSetIDNumber(Communication *output, char function,char address,char job, void * pMem);
void jobSetSerialNumber(Communication *output, char function,char address,char job, void * pMem);
void jobSetIndexNumber(Communication *output, char function,char address,char job, void * pMem);

#endif /* COMMANDFUNCTIONS_H_ */
