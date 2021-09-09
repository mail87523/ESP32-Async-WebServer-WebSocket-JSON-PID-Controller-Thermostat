#include "PID.h"

void PIDController_Init(PIDController *Pid) {

	/* Clear controller variables */
	Pid->Integrator = 0.0f;
	Pid->PrevError1  = 0.0f;

	Pid->Differentiator  = 0.0f;
	Pid->PrevMeasurement = 0.0f;

	Pid->Out = 0.0f;
	
	Pid->LimUp = 1.0f;/*1% Above Setpoint*/
	Pid->LimDown = 1.0f;/*1% Under Setpoint*/
}

float PIDController_Update(PIDController *Pid) 
{

	/*
	* Error signal
	*/
	if (Pid->Mode == HEATING_AUTO_STD || Pid->Mode == HEATING_AUTO_PAR || Pid->Mode == HEATING_MANUAL)
	{
		Pid->Error = Pid->Setpoint - Pid->Measurement;
	}
	else if (Pid->Mode == COOLING_AUTO_STD || Pid->Mode == COOLING_AUTO_PAR || Pid->Mode == COOLING_MANUAL)
	{
		Pid->Error = Pid->Measurement - Pid->Setpoint;
	}

	if (Pid->Mode == HEATING_MANUAL || Pid->Mode == COOLING_MANUAL)
	{
		Pid->Proportional = 0.0f;
		Pid->Integrator = 0.0f;
		Pid->Differentiator = 0.0f;
		Pid->PrevError1 = 0.0f;
		Pid->PrevMeasurement = 0.0f;
		return Pid->Out;
	}


	/***********************************************************************************************/
	/*** STANDARD PID ALGORITHM BEGIN **************************************************************/
	/***********************************************************************************************/
	if (Pid->Mode == HEATING_AUTO_STD || Pid->Mode == COOLING_AUTO_STD)
	{
		if (Pid->Measurement < (Pid->Setpoint + Pid->LimUp * (Pid->Setpoint / 100.0f)) && Pid->Measurement > (Pid->Setpoint - Pid->LimDown * (Pid->Setpoint / 100.0f)))
		{
		Pid->A = 1.0f + Pid->LoopTime / (2.0f * Pid->Ki) + Pid->Kd / Pid->LoopTime;
		Pid->B = 1.0f - Pid->LoopTime / (2.0f * Pid->Ki) + 2.0f * Pid->Kd / Pid->LoopTime;
		Pid->Out = Pid->Out + Pid->Kp * (Pid->A * Pid->Error - (Pid->B * Pid->PrevError1) + (Pid->Kd / Pid->LoopTime) * Pid->PrevError2);
		}
		else
		{
			Pid->Proportional = Pid->Kp * Pid->Error;
			Pid->Out = Pid->Proportional;
		}
		
		
	}
	/***********************************************************************************************/
	/*** STANDARD PID ALGORITHM END ****************************************************************/
	/***********************************************************************************************/

	/***********************************************************************************************/
	/*** PARALELL PID ALGORITHM BEGIN **************************************************************/
	/***********************************************************************************************/	
	else if (Pid->Mode == HEATING_AUTO_PAR || Pid->Mode == COOLING_AUTO_PAR)
	{
		Pid->Proportional = Pid->Kp * Pid->Error;
		/*
		* Integral and Derivative Clamp
		* When is Measurment inside 1% window around Setpoint then calculate Integrator and Differentiator	
		* Up   Clamp from SP limit 1 % = (Pid->Setpoint / 100.0f)
		* Down Clamp from SP limit 1 % = (Pid->Setpoint / 100.0f)
		*/

		if (Pid->Measurement < (Pid->Setpoint + Pid->LimUp * (Pid->Setpoint / 100.0f)) && Pid->Measurement > (Pid->Setpoint - Pid->LimDown * (Pid->Setpoint / 100.0f)))
		{
			Pid->Integrator = Pid->Integrator + 0.5f * Pid->Ki * Pid->LoopTime * (Pid->Error + Pid->PrevError1);
			Pid->Differentiator = Pid->Kd * (Pid->Error - Pid->PrevError1);
		}
		else
		{
			Pid->Integrator = 0.0f;
			Pid->Differentiator = 0.0f;
		}

		/* Anti-wind-up via Integrator clamping */
		if (Pid->Integrator > Pid->LimMaxInt)
		{
			Pid->Integrator = Pid->LimMaxInt;
		}
		else if (Pid->Integrator < Pid->LimMinInt)
		{
			Pid->Integrator = Pid->LimMinInt;
		}

		Pid->Out = Pid->Proportional + Pid->Integrator + Pid->Differentiator;
	}
	/***********************************************************************************************/
	/*** PARALELL PID ALGORITHM END ****************************************************************/
	/***********************************************************************************************/



	// /* Anti-wind-up via dynamic Integrator clamping set limit LimMaxInt  */
	// if (Pid->LimMaxOut > Pid->Proportional)
	// {
	// 	Pid->LimMaxInt = Pid->LimMaxOut - Pid->Proportional;
	// 	//Pid->LimMaxInt = 0.1f * (Pid->LimMaxOut - Pid->LimMinOut) / Pid->Ki;//Tim Wiscott
	// }
	// else
	// {
	// 	Pid->LimMaxInt=0.0f;
	// }

	// /* Anti-wind-up via dynamic Integrator clamping set limit LimMinInt  */
	// if (Pid->LimMinOut < Pid->Proportional)
	// {
	// 	Pid->LimMinInt = Pid->LimMinOut - Pid->Proportional;
	// 	//Pid->LimMinInt = -0.1f * (Pid->LimMaxOut - Pid->LimMinOut) / Pid->Ki; //Tim Wiscott
	// }
	// else
	// {
	// 	Pid->LimMinInt = 0.0f;
	// }



	/*
	* Derivative (band-limited Differentiator)
	*/

	/* Note: derivative on Pid->Measurement, therefore minus sign in front of equation! */
	// Pid->Differentiator = -Pid->Kd * (Pid->Measurement - Pid->PrevMeasurement);
	//Pid->Differentiator = Pid->Kd * (Pid->Error - Pid->PrevError1);
	// 	+ (2.0f * Pid->Tau - Pid->LoopTime) * Pid->Differentiator) /
	//   (2.0f * Pid->Tau + Pid->LoopTime);

	/*
	* Compute output and apply limits
	*/
	

	if (Pid->Out > Pid->LimMaxOut) 
	{
        Pid->Out = Pid->LimMaxOut;
    } 
	else if (Pid->Out < Pid->LimMinOut) 
	{
        Pid->Out = Pid->LimMinOut;
    }

	/* Store Error and Pid->Measurement for later use */

	Pid->PrevError2 = Pid->PrevError1;
	Pid->PrevError1 = Pid->Error;
	Pid->PrevMeasurement = Pid->Measurement;

	/* Return controller output */
    return Pid->Out;

}
