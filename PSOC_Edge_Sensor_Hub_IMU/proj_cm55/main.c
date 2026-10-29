/*******************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for CM55 CPU
*
* Related Document : See README.md
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

/*******************************************************************************
* Header File
*******************************************************************************/

#include "mtb_hal.h"
#include "cybsp.h"
#include "cy_time.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"
#include "../proj_cm33_ns/retarget_io_init.h"
#include "../proj_cm33_ns/sensor_hub_daq_task.h"

/*******************************************************************************
* Macros and globals for the CM55 FreeRTOS idle timer
*******************************************************************************/
#define CM55_LPTIMER_WAIT_TIME_USEC       (62U)
#define CM55_LPTIMER_INTERRUPT_PRIORITY   (1U)

#if defined(APP_SENSOR_CORE_CM55)
static mtb_hal_lptimer_t cm55_lptimer_obj;
static mtb_hal_rtc_t cm55_rtc_obj;

static void cm55_lptimer_interrupt_handler(void)
{
    mtb_hal_lptimer_process_interrupt(&cm55_lptimer_obj);
}

static void cm55_setup_clib_support(void)
{
    Cy_RTC_Init(&CYBSP_RTC_config);
    Cy_RTC_SetDateAndTime(&CYBSP_RTC_config);
    mtb_clib_support_init(&cm55_rtc_obj);
}

static void cm55_setup_tickless_idle(void)
{
    cy_stc_sysint_t lptimer_intr_cfg =
    {
        .intrSrc = CYBSP_CM55_LPTIMER_1_IRQ,
        .intrPriority = CM55_LPTIMER_INTERRUPT_PRIORITY
    };

    if (CY_SYSINT_SUCCESS != Cy_SysInt_Init(&lptimer_intr_cfg,
                                            cm55_lptimer_interrupt_handler))
    {
        handle_app_error();
    }
    NVIC_EnableIRQ(lptimer_intr_cfg.intrSrc);

    if (CY_MCWDT_SUCCESS != Cy_MCWDT_Init(CYBSP_CM55_LPTIMER_1_HW,
                                          &CYBSP_CM55_LPTIMER_1_config))
    {
        handle_app_error();
    }
    Cy_MCWDT_Enable(CYBSP_CM55_LPTIMER_1_HW, CY_MCWDT_CTR_Msk,
                    CM55_LPTIMER_WAIT_TIME_USEC);

    if (CY_RSLT_SUCCESS != mtb_hal_lptimer_setup(&cm55_lptimer_obj,
                                                 &CYBSP_CM55_LPTIMER_1_hal_config))
    {
        handle_app_error();
    }
    cyabs_rtos_set_lptimer(&cm55_lptimer_obj);
}
#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM55 application. 
* 
* Depending on SENSOR_CORE, this core either acquires BMI270 data or sleeps
* while CM33 owns acquisition.
* 
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        /* Disable all interrupts. */
        __disable_irq();

        CY_ASSERT(0);

        /* Infinite loop */
        while(true);

    }

#if defined(APP_SENSOR_CORE_CM55)
    /* In this build CM55 owns the RTC, debug UART and BMI270 acquisition. */
    cm55_setup_clib_support();
    cm55_setup_tickless_idle();
    init_retarget_io();
    __enable_irq();

    if (CY_RSLT_SUCCESS != create_sensor_hub_daq_task())
    {
        handle_app_error();
    }
    vTaskStartScheduler();
    handle_app_error();
#else
    /* The CM33 owns the acquisition in the default build. */
    __enable_irq();
    for (;;)
    {
        Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
#endif
}

/* [] END OF FILE */
