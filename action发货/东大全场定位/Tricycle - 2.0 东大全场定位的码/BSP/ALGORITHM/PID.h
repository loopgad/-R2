#ifndef __PID_H
#define __PID_H
typedef struct PID {
  float  Proportion;         //  ???? Proportional Const  
  float  Integral;           //  ???? Integral Const  
  float  Derivative;         //  ???? Derivative Const  
	float  PrevError;          //  Error[-2]
  float  LastError;          //  Error[-1]  
	float  Error;
	float  DError;
  float  SumError;           //  Sums of Errors  
	float  output;
	float  outputmax;
} PID;

void PID_Position_Calc( PID *pp,  float  CurrentPoint,  float NextPoint )  
{   
        pp->Error =  NextPoint -  CurrentPoint;          
        pp->SumError += pp->Error;                      
        pp->DError = pp->LastError - pp->PrevError;       
        pp->output =  pp->Proportion * pp->Error +   pp->Integral * pp->SumError +   pp->Derivative * pp->DError ;  
	      if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	      if(pp->output < - pp->outputmax )  pp->output = -pp->outputmax;
//	      pp->PrevError = pp->LastError;  
        pp->LastError = pp->Error;
}

void PID_Incremental_Calc( PID *pp,  float  CurrentPoint,  float NextPoint )  
{  
        pp->Error =  NextPoint -  CurrentPoint;          
        pp->SumError += pp->Error;                      
        pp->DError = pp->LastError - pp->PrevError;       
        pp->output +=  pp->Proportion * ( pp->Error - pp->LastError )+   pp->Integral * pp->Error +   pp->Derivative * ( pp->Error +  pp->PrevError - 2*pp->LastError);  
	      if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	      if(pp->output < - pp->outputmax )  pp->output = -pp->outputmax;
	      pp->PrevError = pp->LastError;  
        pp->LastError = pp->Error;
}

void PIDInit(PID *pp, float Kp , float Ki , float Kd , float outputmax)  
{  
	  pp->outputmax  = outputmax;
	  pp->Proportion = Kp;
	  pp->Integral   = Ki;
	  pp->Derivative = Kd;
    pp->DError = pp->Error = pp->output = pp->LastError = pp->PrevError = pp->Derivative = 0; 
}  




#endif