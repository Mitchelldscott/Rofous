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

#ifndef SYS_GRAPH_OBJ
#define SYS_GRAPH_OBJ

#include "hid_comms/hid_comms.h"
#include "task_manager/task_factory.h"

#define MAXIMUM_NODES 10

Vector<FWTaskPacket*>* init_task_manager();

int node_index(int);
void add_task(FWTaskPacket*);
void update_task(FWTaskPacket*);
Vector<float> collect_outputs(int);

void spin();
void dump_all_tasks();

#endif