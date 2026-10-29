/*******************************************************************************
 * File Name:   sensor_hub_daq_task.c
 *
 * Description: This file contains the task that initializes and configures the
 *              bmi270 Motion Sensor and displays the sensor orientation.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
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
* Header Files
*******************************************************************************/
#include "mtb_hal.h"
#include "cybsp.h"
#include "semphr.h"
#include "mtb_bmi270.h"
#include "sensor_hub_daq_task.h"
#include "retarget_io_init.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define GRAVITY_EARTH                       (9.80665f)
#define DEG_TO_RAD                          (0.01745f)
#define GYR_RANGE_DPS                       (2000.0f)
#define ACC_RANGE_2G                        (2.0f)
#define RESET_VAL_ZERO                      (0)

/* Custom app module ID to avoid collisions */
#define APP_RSLT_MODULE_ID                  (1U)

/* App error for task creation failure */
#define APP_RSLT_ACQ_TASK_CREATE_FAILED     CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,\
                                                        APP_RSLT_MODULE_ID,\
                                                        1U)

/*******************************************************************************
* Global Variables
*******************************************************************************/
static mtb_hal_i2c_t CYBSP_I2C_CONTROLLER_hal_obj;
static cy_stc_scb_i2c_context_t CYBSP_I2C_CONTROLLER_context;

static mtb_bmi270_data_t bmi270_data;
static mtb_bmi270_t bmi270;

static cy_en_scb_i2c_status_t initStatus;
static cy_rslt_t result;

/*******************************************************************************
 * Function Name: lsb_to_mps2
 *******************************************************************************
* Summary:
* This function converts the raw sensor value to meter/sec^2.
*
* Parameters:
*  val       raw value
*  g_range   The accelerometer range is expressed in multiples of g.
*  bit_width data width in bits
*
* Return:
*  float
*
*******************************************************************************/
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    float half_scale = (float) (1u << (bit_width - 1u));

    return ((GRAVITY_EARTH)*val * g_range) / half_scale;
}

/*******************************************************************************
 * Function Name: lsb_to_rps
 *******************************************************************************
* Summary:
* This function converts the raw gyroscope sensor value to rad/sec.
*
* Parameters:
*  val       raw value
*  dps       The gyroscope sensor range in degrees per second
*  bit_width data width in bits
*
* Return:
*  float
*
*******************************************************************************/
static float lsb_to_rps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = (float) (1u << (bit_width - 1u));

    return ((DEG_TO_RAD) * ((dps) / (half_scale)) * (val));
}

/*******************************************************************************
 * Function Name: motion_sensor_update_orientation
 *******************************************************************************
 * Summary:
 *  Function that updates the orientation status to one of the 6 types,
 *  'ORIENTATION_UP, ORIENTATION_DOWN, TOP_EDGE, BOTTOM_EDGE,
 *  LEFT_EDGE, and RIGHT_EDGE'. This functions detects the axis that is most 
 *  perpendicular to the ground based on the absolute value of acceleration in 
 *  that axis. The sign of the acceleration signifies whether the axis is 
 *  facing the ground or the opposite.
 *
 * Return:
 *  CY_RSLT_SUCCESS upon successful orientation update, else a non-zero value
 *  that indicates the error.
 *
 ******************************************************************************/
static cy_rslt_t motion_sensor_update_orientation(void)
{
    /* Status variable */
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int16_t abs_x;
    int16_t abs_y;
    int16_t abs_z;

    /* Read x, y, z components of acceleration */
    result = mtb_bmi270_read(&bmi270, &bmi270_data);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("read data failed\r\n");
    }
    abs_x = abs(bmi270_data.sensor_data.acc.x);
    abs_y = abs(bmi270_data.sensor_data.acc.y);
    abs_z = abs(bmi270_data.sensor_data.acc.z);

    if ((abs_z > abs_x) && (abs_z > abs_y))
    {
        if (bmi270_data.sensor_data.acc.z < RESET_VAL_ZERO)
        {
            /* Kit faces down (towards the ground) */
            printf("Orientation = ORIENTATION_DOWN       \r\n");
        }
        else
        {
            /* Kit faces up (towards the sky/ceiling) */
            printf("Orientation = ORIENTATION_UP         \r\n");
        }
    }
    /* Y axis (parallel with shorter edge of board) is most aligned with
     * gravity.
     */
    else if ((abs_y > abs_x) && (abs_y > abs_z))
    {
        if (bmi270_data.sensor_data.acc.y > RESET_VAL_ZERO)
        {
            /* Kit has an inverted landscape orientation */
            printf("Orientation = ORIENTATION_BOTTOM_EDGE\r\n");
        }
        else
        {
            /* Kit has landscape orientation */
            printf("Orientation = ORIENTATION_TOP_EDGE  \r\n");
        }
    }
    /* X axis (parallel with longer edge of board) is most aligned with
     * gravity.
     */
    else
    {
        if (bmi270_data.sensor_data.acc.x < RESET_VAL_ZERO)
        {
            /* Kit has an inverted portrait orientation */
            printf("Orientation = ORIENTATION_RIGHT_EDGE \r\n");
        }
        else
        {
            /* Kit has portrait orientation */
            printf("Orientation = ORIENTATION_LEFT_EDGE  \r\n");
        }
    }
    return result;
}

/*******************************************************************************
 * Function Name: sensor_hub_daq_task
 *******************************************************************************
* Summary:
* This function initializes the I2C and BMI270 sensor, reads the sensor values,
* and prints them on the UART terminal.
*
* Parameters:
*  void *
*
* Return:
*  void
*
*******************************************************************************/
static void sensor_hub_daq_task(void *pvParameters)
{

    TickType_t xLastWakeTime = 0;
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    (void)pvParameters;

    initStatus = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW, 
                                &CYBSP_I2C_CONTROLLER_config, 
                                &CYBSP_I2C_CONTROLLER_context);

    /* I2C initialization failed. Stop program execution. */
    if (CY_SCB_I2C_SUCCESS != initStatus)
    {
        handle_app_error();
    }

    Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);

    result = mtb_hal_i2c_setup(&CYBSP_I2C_CONTROLLER_hal_obj,
                                &CYBSP_I2C_CONTROLLER_hal_config, 
                                &CYBSP_I2C_CONTROLLER_context,
                                NULL);

    /* HAL I2C setup failed. Stop program execution. */
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Initialize can configure platform-dependent function pointers. */
    result = mtb_bmi270_init_i2c(&bmi270, 
                                &CYBSP_I2C_CONTROLLER_hal_obj, 
                                MTB_BMI270_ADDRESS_DEFAULT);
    
    /* BMI270 sensor initialization failed. Stop program execution. */
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Configure accelerometer and gyroscope. */
    result = mtb_bmi270_config_default(&bmi270);

    /* BMI270 sensor configuration failed. Stop program execution. */
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("************************************************************\n");
    printf("               PSOC Edge MCU: Sensor hub IMU                \n");
    printf("************************************************************\r\n");
    printf("Code example configured in data acquisition mode\r\n\n");

    /* Initialize the xLastWakeTime with the current tick count. */
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* Get accelerometer and gyroscope sensor data. */
        result = mtb_bmi270_read(&bmi270, &bmi270_data);

        /* BMI270 sensor read failed. Stop program execution. */
        if (CY_RSLT_SUCCESS != result)
        {
            handle_app_error();
        }

        x = lsb_to_mps2(bmi270_data.sensor_data.acc.x, 
                        ACC_RANGE_2G,
                        bmi270.sensor.resolution);
        y = lsb_to_mps2(bmi270_data.sensor_data.acc.y, 
                        ACC_RANGE_2G,
                        bmi270.sensor.resolution);
        z = lsb_to_mps2(bmi270_data.sensor_data.acc.z, 
                        ACC_RANGE_2G,
                        bmi270.sensor.resolution);
        printf("Accelerometer: \n\rx:%9.6f, y:%9.6f, z:%9.6f \n\n",
                 (double)x, (double)y, (double)z);

        x = lsb_to_rps(bmi270_data.sensor_data.gyr.x, 
                        GYR_RANGE_DPS,
                        bmi270.sensor.resolution);
        y = lsb_to_rps(bmi270_data.sensor_data.gyr.y, 
                        GYR_RANGE_DPS,
                        bmi270.sensor.resolution);
        z = lsb_to_rps(bmi270_data.sensor_data.gyr.z, 
                        GYR_RANGE_DPS,
                        bmi270.sensor.resolution);
        printf("Gyroscope: \n\rx:%9.6f, y:%9.6f, z:%9.6f \n\n",
                 (double)x, (double)y, (double)z);

        motion_sensor_update_orientation();

        /* \x1b[7A\x1b[42D - ANSI ESC sequence to get the cursor back 
         * to the start. */
        printf("\x1b[7A\x1b[42D");

        /* Toggle user led */
        Cy_GPIO_Inv(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN);

        /* Wait for UART traffic to stop */
        while(!(Cy_SCB_UART_IsTxComplete(CYBSP_DEBUG_UART_HW))) {};

        /* Wait for the next cycle.*/
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

/*******************************************************************************
* Function Name: create_sensor_hub_daq_task
********************************************************************************
* Summary:
*  Function that creates Sensor Hub data acquisition task
*
* Parameters:
*  None
*
* Return:
*  CY_RSLT_SUCCESS upon successful creation of the motion sensor task, else a
*  non-zero value that indicates the error.
*
*******************************************************************************/
cy_rslt_t create_sensor_hub_daq_task(void)
{
    BaseType_t status;

    /* Create the "sensor_hub_daq" Task */
    status = xTaskCreate(sensor_hub_daq_task, "Sensor Hub DAQ",
                            TASK_SENSOR_HUB_DAQ_STACK_SIZE, NULL,
                            TASK_SENSOR_HUB_DAQ_PRIORITY, NULL);

    return (pdPASS == status) ? \
        CY_RSLT_SUCCESS : APP_RSLT_ACQ_TASK_CREATE_FAILED;
}


/* [] END OF FILE */