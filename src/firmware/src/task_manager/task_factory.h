/********************************************************************************
 * 
 *      ____                     ____          __           __       _          
 *	   / __ \__  __________     /  _/___  ____/ /_  _______/ /______(_)__  _____
 *	  / / / / / / / ___/ _ \    / // __ \/ __  / / / / ___/ __/ ___/ / _ \/ ___/
 *	 / /_/ / /_/ (__  )  __/  _/ // / / / /_/ / /_/ (__  ) /_/ /  / /  __(__  ) 
 *	/_____/\__, /____/\___/  /___/_/ /_/\__,_/\__,_/____/\__/_/  /_/\___/____/  
 *	      /____/                                                                
 * 
 * 
 * 
 ********************************************************************************/

#ifndef TASK_FACTORY
#define TASK_FACTORY

#include "sensors/lsm6dsox.h"
#include "motor_drivers/pwm.h"
#include "algorithms/constant_task.h"
#include "algorithms/complimentary_filter.h"

#define LSM6DSOX_DRIVER_KEY "LSM"
#define PWM_DRIVER_KEY      "PWM"
#define COMPFLTR_DRIVER_KEY "CMF"
#define CONSTANT_DRIVER_KEY "VAL"

Task* new_task(String);

#endif