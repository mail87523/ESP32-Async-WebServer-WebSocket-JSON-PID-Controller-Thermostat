#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

enum PIDControllerMode
{
	HEATING_AUTO_STD,
	HEATING_AUTO_PAR,
	HEATING_MANUAL,
	COOLING_AUTO_STD,
	COOLING_AUTO_PAR,
	COOLING_MANUAL
};

typedef struct {

	/* Controller Mode Heating/Cooling Auto/Manual*/
	volatile int Mode;
    volatile float Setpoint;
	/* Controller gains */
	volatile float Kp;
	volatile float Ki;
	volatile float Kd;
    

	/* Derivative low-pass filter time constant */
	volatile float Tau;

	/* Output limits */
	volatile float LimMinOut;
	volatile float LimMaxOut;

	/* Integrator limits */
	volatile float LimMinInt;
	volatile float LimMaxInt;

	/* Sample time (in seconds) */
	volatile float LoopTime;
   	

	/* Controller "memory" */
	volatile float Proportional;
	volatile float Integrator;
	volatile float Differentiator;
	
	
	volatile float Measurement;
	volatile float PrevMeasurement;
	volatile float Error;
	volatile float PrevError1;	
	volatile float PrevError2;	
	volatile float A;	
	volatile float B;	
	volatile float LimUp;   /* = 0-100% Limiter for PID calculation ENABLED ZONE for UP  from Setpoint in % of Setpoint */
	volatile float LimDown; /* = 0-100% Limiter for PID calculation ENABLED ZONE for DOWN from Setpoint in % of Setpoint */
	/* Controller output */
	volatile float Out;

} PIDController;



void  PIDController_Init(PIDController *Pid);
float PIDController_Update(PIDController *Pid);

#endif
