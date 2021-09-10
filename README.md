# ESP32 Async WebServer WebSocket JSON PID Controller Thermostat

Code is a mix of collected and modified projects from github.

As a result we got example of one solution for PID Controller with visualization in a Web browser.

The Controller have 8 channel of digital input + 8 channel of digital output + 1 input for zerocross detection + 1 PWM output which is PID controlled output.

Digital inputs and outputs are independent of the PID calculator and work as independent.

Digital inputs wait HIGH PULSE on each channel to control each channel of outputs in parallel with the Web Interface as a physical and virtual button.

Physical Button connected to input I_01 toggle Q_01, also Web Interface button Q_01 also toggle output Q_01 on ESP32 through WebSocket protocol.

I_01 -> Q_01

I_02 -> Q_02

I_03 -> Q_03

I_04 -> Q_04

I_05 -> Q_05

I_06 -> Q_06

I_07 -> Q_07

I_08 -> Q_08

PID control PWM Output by setting  Setpoint, Kp,Ki,Kd, Offset parameters in Web Interface.

Parameter Offset is offset for sensor reading to calibrate all sensors in system to same refernce point.

PID Controller by default is Automatic Heating Mode with Standard PID algorithm but this can be changed through code to different mode like

	HEATING_AUTO_STD
	HEATING_AUTO_PAR
	HEATING_MANUAL
	COOLING_AUTO_STD
	COOLING_AUTO_PAR
	COOLING_MANUAL

Simple and useful for Home Automation Projects.

Feel free to be inspired and make improve of this project, and if You like to support this project feel free to contact me through mail

mail87523@gmail.com

![ESP32_Async_PID_Thermostat](https://user-images.githubusercontent.com/3797201/132812487-72a7ae12-ad25-4685-bc45-ecb661ab632a.JPG)








