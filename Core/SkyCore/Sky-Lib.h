/*
 * Sky-Lib.h
 *
 *  Created on: Sep 29, 2020
 *      Author: RadaR
 */

#ifndef SKYCORE_SKY_LIB_H_
#define SKYCORE_SKY_LIB_H_

/*******************************************************************/
//Include Section
/*******************************************************************/

#include "Sky-Inc.h"

/*******************************************************************/
//Global Section
/*******************************************************************/

extern volatile bool FLT_Flag;

extern volatile struct TPulls A;
extern volatile struct TPulls B;

extern volatile struct TDuty Duty;

extern volatile uint8_t RepData[MAX_50HZ_INDEX];

extern uint16_t SlopeStep[SLOPE_RESOLUTION];
extern uint16_t SlopeV[SLOPE_RESOLUTION];
extern uint16_t SlopeI[SLOPE_RESOLUTION];

extern uint16_t RefV;
extern uint16_t RefI;

/*******************************************************************/
//Api Section
/*******************************************************************/

void Start_Sky(void);
void Reset_Sky(void);
void Start_Sky_IT(void);

void SyncToA(void);
void SyncToB(void);


/*******************************************************************/
//Slope Api Section
/*******************************************************************/

__STATIC_INLINE void PerformDACs(void)
{
		for (uint8_t i=0;i<SLOPE_RESOLUTION;i++) {
			SlopeV[i]=SlopeStep[i]+RefV;
			SlopeI[i]=SlopeStep[i]+RefI;
		}
}


void ResetSlope(void);

#endif /* SKYCORE_SKY_LIB_H_ */
