/*******************************************************************************
 * File Name: app_mouse.c
 *
 * Description: This file consists of the function definitions that are
 *              necessary for developing mouse use cases.
 *
 *******************************************************************************
 * Copyright 2022-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "app_mouse.h"
#include "app_handler.h"
#include "app_bt_hid.h"
#include "app_bt_gatt_handler.h"
extern int temp_connection_flag;

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
#define RPT_ID_IN_MOUSE 0

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
/* Mouse report structure for storing data */
mouse_rpt_t mouse_rpt;

/* Flag is set when mouse report is updated */
volatile bool mouse_rpt_updated = false;
/* Task and Queue Handles of Bluetooth LE HID Mouse Application  */
TaskHandle_t mouse_task_h;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
static void send_msg_to_hid_msg_q(mouse_rpt_t *mouse_rpt);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/**
 *  Function name:
 *  app_mouse_click_change
 *
 *  Function Description:
 *  @brief    Function to Update the button state in mouse report
 *
 *  @param    bit: bit to update corresponding to the button state change
 *  @param    clicked: TRUE if button pressed, else false
 *
 *  @return   void
 */
void app_mouse_click_change(int bit, bool clicked)
{
    uint8_t mask = 1 << bit;

    if (clicked)
    {
        mouse_rpt.buttonState |= mask;
    }
    else
    {
        mouse_rpt.buttonState &= ~mask;
    }

    mouse_rpt_updated = true;
}

/**
 *  Function name:
 *  app_mouse_quadrature_change
 *
 *  Function Description:
 *  @brief    Function to Update the scroll delta in mouse report
 *
 *  @param    delta: Scroll delta
 *
 *  @return   void
 */
void app_mouse_quadrature_change(int8_t delta)
{
    mouse_rpt.scroll = delta;

    mouse_rpt_updated = true;
}

/**
 *  Function name:
 *  app_mouse_motion_change
 *
 *  Function Description:
 *  @brief    Function to Update the motion data in mouse report
 *
 *  @param    x: X axis change
 *  @param    y: Y axis change
 *
 *  @return   void
 */
void app_mouse_motion_change(int16_t x, int16_t y)
{
    mouse_rpt.xMotion = x;
    mouse_rpt.yMotion = y;

    mouse_rpt_updated = true;
}

/**
 *  Function name:
 *  send_msg_to_hid_msg_q
 *
 *  Function Description:
 *  @brief    Function to send mouse report to hid message queue
 *
 *  @param    mouse_rpt: Pointer to mouse report data structure
 *
 *  @return   void
 */
static void send_msg_to_hid_msg_q(mouse_rpt_t *mouse_rpt)
{
    struct hid_rpt_msg mouse_msg;

    mouse_msg.msg_type = MOUSE_MSG_TYPE;
    mouse_msg.data.mouse = *mouse_rpt;

    if (pdPASS != xQueueSend(hid_rpt_q, &mouse_msg, TASK_MAX_WAIT))
    {
        printf("Failed to send msg to HID rpt Queue\r\n");

        /* Start Connection parameter update timer */
        conn_param_updated_flag = FALSE;
        if (pdFAIL == xTimerStart(conn_param_update_timer, TIMER_MAX_WAIT))
        {
            printf("Failed to start Connection parameter update Timer\r\n");
        }
    }

    /* clear mouse report data */
    mouse_rpt->xMotion = mouse_rpt->yMotion = 0;
    mouse_rpt->scroll = 0;
}

/**
 *  Function name:
 *  app_mouse_task
 *
 *  Function Description:
 *  @brief    Task that takes care of mouse activities
 *
 *  @param    args : Task parameter defined during task creation
 *
 *  @return   void
 */
void app_mouse_task(void *args)
{
    uint32_t ulNotifiedValue;

    app_motion_read_cpi_mode_data();

    while (1)
    {
        xTaskNotifyWait(0, 0xffffffff, &ulNotifiedValue, portMAX_DELAY);

        if ((ulNotifiedValue & MOUSE_MOTION) == MOUSE_MOTION)
        {
            app_motion_activity_handler();
        }

        if ((ulNotifiedValue & MOUSE_BUTTON) == MOUSE_BUTTON)
        {
            app_button_activity_handler();
        }

        if ((ulNotifiedValue & MOUSE_SCROLL) == MOUSE_SCROLL)
        {

        }
        if ((ulNotifiedValue & MOUSE_CONEC_PARAM_UPDATE) == MOUSE_CONEC_PARAM_UPDATE)
        {
            if(temp_connection_flag)
            {
                if (pdFAIL == xTimerStart(conn_param_update_timer, TIMER_MAX_WAIT))
                   {
                      printf("Failed to start Connection parameter update Timer\r\n");
                   }

            }

            else
            {
                if (pdFAIL == xTimerStop(conn_param_update_timer, TIMER_MAX_WAIT))
                    {
                         printf("Failed to start Connection parameter update Timer\r\n");
                    }
            }
        }

        if (mouse_rpt_updated)
        {
            send_msg_to_hid_msg_q((mouse_rpt_t*)&mouse_rpt.buttonState);
            mouse_rpt_updated = FALSE;
        }
    }
}
