// Do not remove the include below
#include "PlutoPilot.h"
#include "Xshield.h"
#include "Control.h"
#include "Led.h"
#include "Print.h"

//GLOBAL VARIABLES
int r,l;
int e;
long double le = 0;//last error
int ri =50, li =50 ;
int ei;
int maxRange = 2000; // range of sensor
double erf = 0.8; // error reduction factor
int rg = 10; // range of stable band

long double kp = 0.9 , kd = 30, ki = 0; // proportional const
//long double reset = 0;
int pd; // proportional derivative
bool init = true ;
bool tkFlt = false;
double passFilter = 0.1;
double lastErr = 0;


//float q = 0.01;
//float last_estimate = 0 ;
//float err_measure = 50;
//float err_estimate = 50;
//float kalman_gain;
//float current_estimate;





//The setup function is called once at Pluto's hardware startup
void plutoInit()
{
// Add your hardware initialization code here
	Xshield.init();
}


//The function is called once before plutoPilot when you activate Developer Mode
void onPilotStart()
{
  // do your one time stuffs here
	Xshield.startRanging();
	Control.setUserLoopFrequency(50);

}


// The loop function is called in an endless loop
void plutoPilot()
{
	if(tkFlt == true) {
		Control.setRC(RC_PITCH, 1522);
	}

	l = Xshield.getRange(LEFT);
	r = Xshield.getRange(RIGHT);
	if(init == true && (l > 0 || r > 0)){
		li = l;
		ri = r;
		ei = ri - li;
		init = false;
		lastErr = ei;
	}
	Print.monitor("\nl  : ", l);
	Print.monitor("  r  : ", r);
	Print.monitor("  li  : ", li);
	Print.monitor("  ri  : ", ri);

	e = erf*(r - l);
	//Filter
	e = passFilter * e + (1-passFilter) * le ;

	Print.monitor("  Err  : ", e);
	Print.redGraph(e);
//	kalman_filter 
//	kalman_gain = err_estimate / ( err_estimate +  err_measure);
//	current_estimate = last_estimate + kalman_gain * ( e - last_estimate);
//	err_estimate =  (1.0 - kalman_gain) * err_estimate + (( last_estimate - current_estimate) > 0 ? ( last_estimate - current_estimate): -1*( last_estimate - current_estimate)) * q;
//	last_estimate = current_estimate;
//	e = current_estimate;
//	Print.greenGraph(e);

	if (l < maxRange && r < maxRange){

		// move to  left or move to right (PD control)
		if (e < -1*rg || e > rg){
			pd = 1500 + (kp*e + kd*(e - le));
			if (pd > 1800) pd = 1800;
			if (pd < 1200) pd = 1200;
			Control.setRC(RC_ROLL, pd);
			tkFlt = true;
		}

		Print.monitor("   KP   : ", kp*e);
		Print.monitor("   KD   : ", kd*(e - le));
		Print.monitor("   PD   : ", pd);
		if (e <= rg && e >= -1*rg){

		}
	}
	else{
		// Wall discontinuous at Left
		if(l > maxRange && r < maxRange){
			e = erf*(ri - r);
			// move to  left
			if (e < -1*rg && e > rg){
				pd = 1500 + (kp*e + kd*(e - le));
				Control.setRC(RC_ROLL, pd);
			}
			// move to right

		}
		// Wall discontinuous at Right
		if(l < maxRange && r > maxRange){
			e = erf*(li - l);
			// move to  left
			if (e < -1*rg && e > rg){
				pd = 1500 + (kp*e + kd*(e - le));
				Control.setRC(RC_ROLL, pd);
			}
			// move to right
			if (e > rg){
				pd = 1500 + (kp*e + kd*(e - le));
				Control.setRC(RC_ROLL, pd);
			}
		}
	}
	le = e;
}



//The function is called once after plutoPilot when you deactivate Developer Mode
void onPilotFinish()
{
	Xshield.stopRanging();
}




