#include "mode3.h"
//定义模式3的方向判别函数
int8_t Dir_PWM(uint16_t  X_Y)
{
	int8_t Dir_PWM;
	//设立多级开环调节速度
	if(X_Y>2300)
	{
	  Dir_PWM = 10;
	  if(X_Y>2400)
	  {
		Dir_PWM =20;
		if(X_Y>2600)
	   {
		 Dir_PWM =30; 
	     if(X_Y>2800)
	    {
		  Dir_PWM =40;
		  if(X_Y>3000)
	     {
		   Dir_PWM =50; 
		   if(X_Y>3200)
		  {
			Dir_PWM =60; 
			if(X_Y>3400)
	       {
		     Dir_PWM =70; 
			 if(X_Y>3600)
	        {  
		      Dir_PWM =80; 
			  if(X_Y>3800)
	         {
		       Dir_PWM =100; 
	         }
	        }
	       }
		  }
	     }
	    }
	   }	
	  }
     }
	else if(X_Y<1600)
	{
	  Dir_PWM = -10;
	  if(X_Y<1500)
	  {
		Dir_PWM =-20;
		if(X_Y<1400)
	   {
		 Dir_PWM =-30; 
	     if(X_Y<1200)
	    {
		  Dir_PWM =-40;
		  if(X_Y<1000)
	     {
		   Dir_PWM =-50; 
		   if(X_Y<800)
		  {
			Dir_PWM =-60; 
			if(X_Y<600)
	       {
		     Dir_PWM =-70; 
			 if(X_Y<400)
	        {  
		      Dir_PWM =-80; 
			  if(X_Y<200)
	         {
		       Dir_PWM =-100; 
	         }
	        }
	       }
		  }
	     }
	    }
	   }
	  }
	}
	else Dir_PWM = 0;
	return Dir_PWM;
}
