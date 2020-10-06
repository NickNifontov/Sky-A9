/*
 * Sky-Lib.c
 *
 *  Created on: Sep 29, 2020
 *      Author: RadaR
 */


/*******************************************************************/
//Include Section
/*******************************************************************/

#include "Sky-Inc.h"

/*******************************************************************/
//Global Section
/*******************************************************************/

volatile bool FLT_Flag=true;

volatile struct TPulls A={.fComp=false, .Realized=MIN_DUTY, .Expected=MIN_DUTY, .Complete=false};
volatile struct TPulls B={.fComp=false, .Realized=MIN_DUTY, .Expected=MIN_DUTY, .Complete=false};

volatile struct TDuty Duty={.Softstart=false, .MaxDuty=MAX_DUTY, .Index=0, .Index50Hz=0};

volatile uint8_t RepData[MAX_50HZ_INDEX]={250,5,250,250,5,250};

uint16_t RefV=0;
uint16_t RefI=0;

uint16_t SlopeStep[SLOPE_RESOLUTION]={
		(uint16_t)(SLOPE_STEP*7), (uint16_t)(SLOPE_STEP*6), (uint16_t)(SLOPE_STEP*5), (uint16_t)(SLOPE_STEP*4), (uint16_t)(SLOPE_STEP*3), (uint16_t)(SLOPE_STEP*2), (uint16_t)(SLOPE_STEP),
		(uint16_t)(SLOPE_STEP_2*10), (uint16_t)(SLOPE_STEP_2*9), (uint16_t)(SLOPE_STEP_2*8), (uint16_t)(SLOPE_STEP_2*7), (uint16_t)(SLOPE_STEP_2*6), (uint16_t)(SLOPE_STEP_2*5), (uint16_t)(SLOPE_STEP_2*4), (uint16_t)(SLOPE_STEP_2*3), (uint16_t)(SLOPE_STEP_2*2), (uint16_t)(SLOPE_STEP_2),
		(uint16_t)(SLOPE_STEP*10), (uint16_t)(SLOPE_STEP*9), (uint16_t)(SLOPE_STEP*8)
};

uint16_t SlopeV[SLOPE_RESOLUTION]={
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

uint16_t SlopeI[SLOPE_RESOLUTION]={
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/*******************************************************************/
//Slope API Section
/*******************************************************************/
void ResetSlope(void)
{
	RefV=0;
	RefI=0;
}

/*******************************************************************/
//Api Section
/*******************************************************************/

void SyncToA(void)
{
	Duty.Index50Hz=MAX_50HZ_INDEX;
}

void SyncToB(void)
{
	Duty.Index50Hz=2;
}

void Reset_Sky(void)
{
	  //softstart
	  Duty.Softstart=true;
	  Duty.MaxDuty=4608;
	  Duty.Index=0;
	  SyncToB();
	  //
	  A.Complete=false;
	  A.Realized=MIN_DUTY;
	  A.Expected=MIN_DUTY;
	  A.fComp=false;
	  //
	  B.Complete=false;
	  B.Realized=MIN_DUTY;
	  B.Expected=MIN_DUTY;
	  B.fComp=false;

	  LL_GPIO_ResetOutputPin(GPIOA, pwm50HZ_1_Pin|pwm50HZ_2_Pin);

	  LL_HRTIM_DisableOutput(HRTIM1,LL_HRTIM_OUTPUT_TB1 | LL_HRTIM_OUTPUT_TA2);
}

void Start_Sky_IT(void)
{
	  LL_HRTIM_EnableIT_FLT1(HRTIM1);

	  //LL_HRTIM_EnableIT_REP(HRTIM1, LL_HRTIM_TIMER_MASTER);

	  //LL_HRTIM_EnableIT_REP(HRTIM1, LL_HRTIM_TIMER_D);

	  LL_HRTIM_EnableIT_CMP4(HRTIM1, LL_HRTIM_TIMER_B);
}

void Start_Sky(void)
{
	  Reset_Sky();

	  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_MASTER, RepData[0]);

	  //LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_C, 10);

	  LL_HRTIM_EnableOutput(HRTIM1,LL_HRTIM_OUTPUT_TB1 | LL_HRTIM_OUTPUT_TA2);
	  //LL_HRTIM_EnableOutput(HRTIM1,LL_HRTIM_OUTPUT_TB2);

	  LL_HRTIM_TIM_CounterEnable(HRTIM1,
	  	  			    LL_HRTIM_TIMER_MASTER
						| LL_HRTIM_TIMER_A
						| LL_HRTIM_TIMER_B
						| LL_HRTIM_TIMER_C
						| LL_HRTIM_TIMER_D
						| LL_HRTIM_TIMER_E);
}
