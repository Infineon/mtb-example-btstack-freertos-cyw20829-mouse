/*******************************************************************************
 * File Name: app_motion.h
 *
 * Description: This file consists of the function prototypes that are
 *              necessary for developing the HAL applications
 *
 *******************************************************************************
 * Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

#ifndef __APP_MOTION_H_
#define __APP_MOTION_H_

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/

#include "cybsp_types.h"

#if (HW_SPI_ENABLED == true) && (ENABLE_LOGGING == false)
#define SPI_FREQ_HZ                (2000000UL)
#else
#define SPI_FREQ_HZ                (500000UL)
#endif

#define MOTION_TIMER_INT_PRIORITY   (6U)
#define SPI_INTERRUPT_PRIORITY      (3U)

#define MOTION_DATA_READ_INTERVAL   (7U) // 7ms

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
typedef enum
{
    MOTION_CPI_836 = 1,
    MOTION_CPI_1026,
    MOTION_CPI_1634,
    MOTION_CPI_MAX
}motion_cpi_mode_e;

typedef enum
{
    MOTION_LIGHT_SLEEP,
    MOTION_DEEP_SLEEP
}motion_sleep_mode_e;

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
typedef struct
{
    int16_t delta_x;
    int16_t delta_y;
    uint8_t cpi;
    uint8_t motion_status;
} motion_sensor_data_t;

extern volatile bool motion_activity_stat;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
cy_rslt_t app_motion_sensor_init(void);
cy_rslt_t app_motion_data_fetch(void);
cy_rslt_t app_motion_cpi_change(void);
cy_rslt_t app_motion_cpi_reset(void);
cy_rslt_t app_motion_sleep(motion_sleep_mode_e sleep_mode);
cy_rslt_t app_motion_disable(void);
void app_motion_activity_handler(void);
cy_rslt_t app_motion_restore_cpi_mode(void);
cy_rslt_t app_motion_read_cpi_mode_data(void);
void app_motion_update_read_interval(uint16_t conn_interval);

#endif /* __APP_MOTION_H_ */
