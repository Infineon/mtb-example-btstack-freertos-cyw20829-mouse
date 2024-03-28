/*******************************************************************************
 * File Name: app_mouse.h
 *
 * Description: This file consists of the function prototypes that are
 *              necessary for developing mouse module use cases.
 *
 *******************************************************************************
 * Copyright 2022-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef __APP_MOUSE_H_
#define __APP_MOUSE_H_

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "cy_sysint.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "cycfg_peripherals.h"
#include "app_config.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

#pragma pack(1)

/* Boot mode mouse report */
typedef struct
{
    /* Button state in bitmap */
    uint8_t buttonState;
    /* X motion 8 bits */
    int8_t xMotion;
    /* Y motion 8 bits */
    int8_t yMotion;
    /* The scroll wheel motion */
    int8_t scroll;
} boot_mouse_rpt_t;

/* Report mode mouse report */
typedef struct
{
    /* Button state in bitmap */
    uint8_t buttonState;
    /* X motion 8 bits */
    int8_t xMotion;
    /* Y motion 8 bits */
    int8_t yMotion;
    /* Scroll wheel motion */
    int8_t scroll;

} mouse_rpt_t;
#pragma pack()

typedef enum
{
    MOUSE_BUTTON = 1,
    MOUSE_MOTION = 2,
    MOUSE_SCROLL = 4,
    MOUSE_CONEC_PARAM_UPDATE=5
} mouse_event_e;

typedef enum
{
    LEFT_BUTTON_BIT,
    RIGHT_BUTTON_BIT,
    MIDDLE_BUTTON_BIT
} mouse_button_e;

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
/* Task and Queue Handles of Bluetooth LE HID Mouse Application  */
extern TaskHandle_t mouse_task_h;
extern volatile bool mouse_rpt_updated;
/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

void app_mouse_task(void *args);
void app_mouse_click_change(int bit, bool clicked);
void app_mouse_quadrature_change(int8_t delta);
void app_mouse_motion_change(int16_t x, int16_t y);

#endif
