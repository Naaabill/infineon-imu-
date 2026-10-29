/******************************************************************************
 * File Name:   sensor_hub_data_fusion.c
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
******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include <math.h>
#include "cybsp.h"
#include "semphr.h"
#include "mtb_bmi270.h"
#include "sensor_hub_fusion.h"
#include "retarget_io_init.h"

#include "bsxlite_interface.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define GRAVITY_EARTH                       (9.80665f)
#define DEG_TO_RAD                          (0.01745f)
#define GYR_RANGE_DPS                       (2000.0f)
#define ACC_RANGE_2G                        (2.0f)
#define BSXLITE_INVALID_INSTANCE            (0U)

/* Custom app module ID to avoid collisions */
#define APP_RSLT_MODULE_ID                  (1U)

/* App error for task creation failure */
#define APP_RSLT_FUSION_TASK_CREATE_FAILED  CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,\
                                                        APP_RSLT_MODULE_ID,\
                                                        2U)

typedef enum {
    OUTPUT_MODE_QUATERNION = 1,
    OUTPUT_MODE_EULER      = 2
} output_mode_t;

/*******************************************************************************
* Global Variables
*******************************************************************************/
static mtb_hal_i2c_t CYBSP_I2C_CONTROLLER_hal_obj;
static cy_stc_scb_i2c_context_t CYBSP_I2C_CONTROLLER_context;

static cy_en_scb_i2c_status_t initStatus;

static const output_mode_t output_mode = OUTPUT_MODE_QUATERNION;

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
********************************************************************************
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
 * Function Name: sensor_hub_fusion_task
 *******************************************************************************
* Summary:
* This function initializes the I2C and BMI270 sensor, reads the sensor values,
* and uses BSXLite library function to fuse 6 -axis sensor data and print the 
* Quaternion vectors and/or Euler angles over serial terminal.
*
* Parameters:
*  void *
*
* Return:
*  void
*
*******************************************************************************/
static void sensor_hub_fusion_task(void *pvParameters)
{
    TickType_t xLastWakeTime = 0;
    TickType_t xCurrWakeTime = 0;
    TickType_t xElapsedTime  = 0;
    
    /* BMI270 driver handle*/
    mtb_bmi270_t bmi270;
    /* BMI270 sensor data */
    mtb_bmi270_data_t bmi270_data;

    /* BSX Library related variables*/
    bsxlite_out_t bsxlite_fusion_out;
    bsxlite_return_t  result;
    bsxlite_instance_t instance = BSXLITE_INVALID_INSTANCE;
    vector_3d_t acc_in;
    vector_3d_t gyr_in;

    (void)pvParameters;

    memset(&acc_in, 0, sizeof(acc_in));
    memset(&acc_in, 0, sizeof(acc_in));
    memset(&bsxlite_fusion_out, 0, sizeof(bsxlite_fusion_out));

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

    /* Initialize BSXLite library */
    result = bsxlite_init(&instance);
    CY_ASSERT(BSXLITE_OK == result);

    bsxlite_set_to_default(&instance);
    CY_ASSERT(BSXLITE_OK == result);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("************************************************************\n");
    printf("               PSOC Edge MCU: Sensor hub IMU                \n");
    printf("************************************************************\r\n");
    printf("Code example configured in data fusion mode\r\n\n");

    /* Initialize the xCurrWakeTime and xLastWakeTime with the current ticks.*/
    xCurrWakeTime = xTaskGetTickCount();
    xLastWakeTime = xCurrWakeTime;

    for (;;)
    {
        /* Turn user led on.*/
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, 
                        CYBSP_USER_LED1_PIN, 
                        CYBSP_LED_STATE_ON);

        /* Get accelerometer and gyroscope sensor data. */
        result = mtb_bmi270_read(&bmi270, &bmi270_data);

        /* BMI270 sensor read failed. Stop program execution. */
        if (CY_RSLT_SUCCESS != result)
        {
            handle_app_error();
        }

        acc_in.x = lsb_to_mps2(bmi270_data.sensor_data.acc.x,
                                ACC_RANGE_2G, bmi270.sensor.resolution);
        acc_in.y = lsb_to_mps2(bmi270_data.sensor_data.acc.y,
                                ACC_RANGE_2G, bmi270.sensor.resolution);
        acc_in.z = lsb_to_mps2(bmi270_data.sensor_data.acc.z,
                                ACC_RANGE_2G, bmi270.sensor.resolution);

        gyr_in.x = lsb_to_rps(bmi270_data.sensor_data.gyr.x,
                               GYR_RANGE_DPS, bmi270.sensor.resolution);
        gyr_in.y = lsb_to_rps(bmi270_data.sensor_data.gyr.y,
                              GYR_RANGE_DPS, bmi270.sensor.resolution);
        gyr_in.z = lsb_to_rps(bmi270_data.sensor_data.gyr.z,
                               GYR_RANGE_DPS, bmi270.sensor.resolution);

        result = bsxlite_do_step(&instance, xElapsedTime, &acc_in, &gyr_in,
                                  &bsxlite_fusion_out);

        /** evaluate library return */
        if (result != BSXLITE_OK)
        {
            printf("BSXLite: error in processing data!!!\n");
        }

        switch (output_mode) 
        {
            case OUTPUT_MODE_QUATERNION:
                printf("Quaternion: %f, %f, %f, %f\r\n",
                    bsxlite_fusion_out.rotation_vector.w,
                    bsxlite_fusion_out.rotation_vector.x,
                    bsxlite_fusion_out.rotation_vector.y,
                    bsxlite_fusion_out.rotation_vector.z);
                break;

            case OUTPUT_MODE_EULER:
                printf("Orientation: %f, %f, %f, %f\r\n",
                    bsxlite_fusion_out.orientation.heading,
                    bsxlite_fusion_out.orientation.pitch,
                    bsxlite_fusion_out.orientation.roll,
                    bsxlite_fusion_out.orientation.yaw);
                break;

            default:
                printf("Error: Invalid output mode.\r\n");
                break;
        }
        /* Turn user led off.*/
        Cy_GPIO_Write(CYBSP_USER_LED1_PORT, 
                        CYBSP_USER_LED1_PIN, 
                        CYBSP_LED_STATE_OFF);

        /* Update elapsed time.*/
        xElapsedTime += (xCurrWakeTime - xLastWakeTime) * configTICK_RATE_HZ;
        xLastWakeTime = xCurrWakeTime;

        /* Wait for the next cycle.*/
        vTaskDelayUntil(&xCurrWakeTime, 
                    (const TickType_t) TASK_SENSOR_HUB_FUSION_RATE_MS);
    }
}


/*******************************************************************************
 * Function Name: create_sensor_hub_fusion_task
 *******************************************************************************
* Summary:
*  Function that creates "sensor_hub_fusion_task"
*
* Parameters:
*  None
*
* Return:
*  CY_RSLT_SUCCESS upon successful creation of the motion sensor task, else a
*  non-zero value that indicates the error.
*
*******************************************************************************/
cy_rslt_t create_sensor_hub_fusion_task(void) {

    BaseType_t status;

    /* Create the "sensor_hub_daq" Task */
    status = xTaskCreate(sensor_hub_fusion_task, "Sensor Hub Fusion",
            TASK_SENSOR_HUB_FUSION_STACK_SIZE, NULL,
            TASK_SENSOR_HUB_FUSION_PRIORITY, NULL);

    return (pdPASS == status) ? \
        CY_RSLT_SUCCESS : APP_RSLT_FUSION_TASK_CREATE_FAILED;
}

/* [] END OF FILE */