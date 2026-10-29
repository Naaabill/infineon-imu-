/******************************************************************************
* File Name:   sensor_hub_daq_task.h
*
* Description: This file is the public interface of sensor_hub_daq_task.c. This
*              file also contains the BMI270 motion sensor configuration
*              parameters.
*
* Related Document: See README.md
*
*
********************************************************************************
 * (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

#ifndef _SENSOR_HUB_DAQ_TASK_H_
#define _SENSOR_HUB_DAQ_TASK_H_

#if defined(__cplusplus)
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define TASK_SENSOR_HUB_DAQ_PRIORITY    (configMAX_PRIORITIES - 1)
#define TASK_SENSOR_HUB_DAQ_STACK_SIZE  (2048u)
#define TASK_SENSOR_HUB_DAQ_DELAY_MS    (10u)

/*******************************************************************************
* Function prototypes
*******************************************************************************/
cy_rslt_t create_sensor_hub_daq_task(void);

#if defined(__cplusplus)
}
#endif

#endif /* _SENSOR_HUB_DAQ_TASK_H_ */


/* [] END OF FILE */