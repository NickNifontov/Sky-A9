/*
 * Sky-Defines.h
 *
 *  Created on: Sep 29, 2020
 *      Author: RadaR
 */

#ifndef SKYCORE_SKY_DEFINES_H_
#define SKYCORE_SKY_DEFINES_H_

/*******************************************************************/
//Include Section
/*******************************************************************/

#include "Sky-Inc.h"

/*******************************************************************/
//Define Section
/*******************************************************************/

#define MIN_DUTY					(uint16_t)(0)
#define MAX_DUTY					(uint16_t)(18440)

#define DT_ZONE						(uint16_t)(4608)

#define PULLS_CENTER				(uint16_t)(23040)

#define softstart_step				(uint16_t)(100)

#define MAX_INDEX					(uint16_t)(505)
#define MAX_50HZ_INDEX				(uint16_t)(6)

#define SLOPE_RESOLUTION			(uint8_t)(20) // every 500ns

#define SLOPE_STEP			        (uint8_t)(100) // more for pulls #2
#define SLOPE_STEP_2			    (uint8_t)(100)

struct TPulls {
	bool fComp;
	uint16_t Realized;
	uint16_t Expected;
	bool Complete;
};

struct TDuty {
	bool Softstart;
	uint16_t MaxDuty;
	uint16_t Index;
	uint8_t Index50Hz;
};

#endif /* SKYCORE_SKY_DEFINES_H_ */
