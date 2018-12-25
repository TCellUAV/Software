#ifndef _MSP_NVIC_H_
#define _MSP_NVIC_H_

#include "sys_Platform.h"

typedef enum
{
	NVIC_PRIO_GROUP_0 = NVIC_PriorityGroup_0,	                    
	NVIC_PRIO_GROUP_1 = NVIC_PriorityGroup_1,	                    
	NVIC_PRIO_GROUP_2 = NVIC_PriorityGroup_2,	                    
	NVIC_PRIO_GROUP_3 = NVIC_PriorityGroup_3,	              
	NVIC_PRIO_GROUP_4 = NVIC_PriorityGroup_4,
}MSP_NVIC_GROUP;

void msp_NVIC_Init(MSP_NVIC_GROUP mspNvicGroup);

#endif
