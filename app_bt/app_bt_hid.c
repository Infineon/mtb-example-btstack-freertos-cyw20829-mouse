/*******************************************************************************
 * File Name: ble_hid.c
 *
 * Description:
 * This file contains the BLE HID task to send reports on Mouse events and
 * battery change events to the connected HID Host.
 *
 * Related Document: See README.md
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

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "app_bt_hid.h"
#include "app_bt_advert.h"
#include "app_bt_bonding.h"
#include "app_bt_event_handler.h"
#include "app_handler.h"
#include "app_batmon.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
/* Task and Queue Handles of  Bluetooth LE HID Mouse Application  */
TaskHandle_t ble_task_h;
/* Queue for sending Mouse events and battery reports to Bluetooth LE Task */
QueueHandle_t hid_rpt_q;

struct hid_rpt_msg rpt_msg;

/* Timer handle for triggering disconnection when idle for sometime */
TimerHandle_t ble_disconnection_timer;
/* Timer handle for triggering connection parameter update */
TimerHandle_t conn_param_update_timer;
/* Timer handle for stopping advertisement */
TimerHandle_t adv_stop_timer;

uint8_t app_hid_mouse_report[MOUSE_REPORT_BUFF_SIZE][MOUSE_REPORT_SIZE];

extern wiced_bt_device_address_t peer_bd_addr;
extern TimerHandle_t motion_data_read_timer;
extern uint16_t conn_interval;
/* Current Application state */
app_ble_state_t current_app_state = UNPAIRED_ON;
/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
void app_bt_hid_idle_disconnection_timer_cb(TimerHandle_t cb_params);
void app_bt_hid_conn_param_update_timer_cb(TimerHandle_t cb_params);
void app_bt_hid_send_batt_report(uint8_t battery_percentage);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
/**
 *  Function name:
 *  app_bt_hid_get_device_state
 *
 *  Function Description:
 *  @brief Function to get the current device state
 *
 *  @param    void
 *
 *  @return   app_ble_state_t: Current BLE state of the application
 */
inline app_ble_state_t app_bt_hid_get_device_state(void)
{
    return current_app_state;
}

/**
 *  Function name:
 *  app_bt_hid_update_device_state
 *
 *  Function Description:
 *  @brief Function to update the current device state
 *
 *  @param    app_ble_state_t: Current BLE state of the application
 *
 *  @return   void
 */
inline void app_bt_hid_update_device_state(app_ble_state_t app_state)
{
    current_app_state = app_state;
}

/**
 *  Function name:
 *  app_bt_hid_handle_low_battery
 *
 *  Function Description:
 *  @brief Function to handle battery low condition
 *
 *  @param    void
 *
 *  @return   void
 */
void app_bt_hid_handle_low_battery(void)
{
    /* If connected, Disconnect from current host and configure sleep modes for motion and scroll */
    if (app_bt_conn_id != 0)
    {
        if (CY_RSLT_SUCCESS != wiced_bt_gatt_disconnect(app_bt_conn_id))
        {
            printf("LE Disconnection failed \r\n");
            return;
        }
    }
    /* Stop ongoing adv if any */
    app_bt_adv_stop();
    /* LED Blink indication before Hibernation */
    app_led_update_blink_period(LED_BLINK_RATE_MS);
    vTaskDelay(SHUTDOWN_LED_BLINK_INDICATION_MS);
    /* Turn OFF LED */
    app_status_led_off();
    /* Disable motion sensor */
    app_motion_disable();
    /* Disable Button pullups */
    app_button_disable();
    /* Enter Hibernation state */
    Cy_SysPm_SystemEnterHibernate();
}

/**
 *  Function name:
 *  app_bt_hid_pairing_mode_switch
 *
 *  Function Description:
 *  @brief This Function is used to switch device to pairing mode
 *
 *  @param    void
 *
 *  @return    void
 */
void app_bt_hid_pairing_mode_switch(void)
{
    /* Stop Connection parameter update timer */
    if (pdFAIL == xTimerStop(conn_param_update_timer, TIMER_MIN_WAIT))
    {
        printf("Failed to stop Connection param update Timer\r\n");
    }

    /* If in connected state, disconnect from current host */
    if (app_bt_conn_id != 0)
    {
        if (CY_RSLT_SUCCESS != wiced_bt_gatt_disconnect(app_bt_conn_id))
        {
            printf("LE Disconnection failed \r\n");
            return;
        }
    }

    /* Stop ongoing adv if any */
    app_bt_adv_stop();

    /* update current state to any host adv */
    app_bt_hid_update_device_state(PAIRED_ADVERTISING_ANY_HOST);

    if (!app_bt_conn_id)
    {
        /* Start undirected adv for pairing to new host device if not in connected state, else start from disconnect callback */
        app_bt_adv_start_any_host();
    }
}

/**
 *  Function name:
 *  app_bt_hid_bond_index_switch
 *
 *  Function Description:
 *  @brief This Function is used to change the bond info slot
 *
 *  @param    void
 *
 *  @return    void
 */
void app_bt_hid_bond_index_switch(void)
{
    /* If in connected state, disconnect from current host */
    if (app_bt_conn_id != 0)
    {
        if (CY_RSLT_SUCCESS != wiced_bt_gatt_disconnect(app_bt_conn_id))
        {
            printf("LE Disconnection failed \r\n");
            return;
        }
    }

    /* Stop ongoing adv if any */
    app_bt_adv_stop();

    /*Update current bond index */
    app_bt_bond_update_index();

    /* update current state to any host adv */
    if (CY_RSLT_SUCCESS == app_bt_bond_check_info())
    {
        app_bt_hid_update_device_state(PAIRED_ADVERTISING_KNOWN_HOST);
    }
    else
    {
        app_bt_hid_update_device_state(PAIRED_ADVERTISING_ANY_HOST);
    }

    if (!app_bt_conn_id)
    {
        /* Start adv if not in connected state, else start from disconnect callback */
        app_bt_adv_start();
    }
}

/**
 *  Function name:
 *  app_bt_hid_idle_disconnection_timer_cb
 *
 *  Function Description:
 *  @brief Timer cb to disconnect from current host device if idle for some time
 *
 *  @param    cb_param: Argument to cb
 *
 *  @return   void
 */
void app_bt_hid_idle_disconnection_timer_cb(TimerHandle_t cb_params)
{
    if (app_bt_conn_id != 0)
    {
        if (CY_RSLT_SUCCESS != wiced_bt_gatt_disconnect(app_bt_conn_id))
        {
            printf("LE Disconnection failed \r\n");
            return;
        }
    }
}

/**
 *  Function name:
 *  app_bt_hid_conn_param_update_timer_cb
 *
 *  Function Description:
 *  @brief Timer cb to trigger BLE connection parameter update
 *
 *  @param    cb_param: Argument to cb
 *
 *  @return   void
 */
void app_bt_hid_conn_param_update_timer_cb(TimerHandle_t cb_params)
{
    if((app_bt_conn_id !=0) &&
            (conn_param_updated_flag == FALSE))
    {
        wiced_bool_t conn_update_status = 0;

        printf("sending conn param request\r\n");
        conn_update_status = wiced_bt_l2cap_update_ble_conn_params(
                        peer_bd_addr,
                        MIN_CI,
                        MAX_CI,
                        SLAVE_LATENCY,
                        SUPERVISION_TO);

        if (0 != conn_update_status)
        {
            printf("Connection parameter update successful\r\n");
        }


        if(conn_interval > 0)
        {
            app_motion_update_read_interval(conn_interval);
        }


    }
}

/**
 *  Function name:
 *  app_bt_hid_send_batt_report
 *
 *  Function Description:
 *  @brief Function to send battery level percentage
 *
 *  @param    battery_percentage: Battery percentage calculated using voltage measured by ADC
 *
 *  @return   void
 */
void app_bt_hid_send_batt_report(uint8_t battery_percentage)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    app_bas_battery_level[0] = battery_percentage;

    /* Check the cccd */
    if (app_bas_battery_level_client_char_config[0] == GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        /* send the Battery HID report data */
        gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                             HDLC_BAS_BATTERY_LEVEL_VALUE,
                                                             app_bas_battery_level_len,
                                                             app_bas_battery_level,
                                                             NULL);
        if (WICED_BT_GATT_SUCCESS == gatt_status)
        {
            printf("Battery level Notification sent\r\n\r\n");
        }
    }
}

/**
 *  Function name:
 *  app_bt_send_mouse_report
 *
 *  Function Description:
 *  @brief Function to send mouse reports to peer device
 *
 *  @param    mouse_rpt: Pointer to mouse report data structure
 *
 *  @return   wiced_bt_gatt_status_t: WICED_BT_GATT_SUCCESS if notification was successful,
 *  an error code otherwise.
 */
static wiced_bt_gatt_status_t app_bt_hid_send_mouse_report(mouse_rpt_t *mouse_rpt)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    uint32_t ulNotifiedValue;
    static uint8_t rpt_buf_idx = 0;

    /* Checking protocol mode selected */
    if (app_hids_protocol_mode[0] == PROTOCOL_REPORT_MODE)
    {
        /* Report mode selected, Check the cccd */
        if (app_hids_mouse_in_report_client_char_config[0] == GATT_CLIENT_CONFIG_NOTIFICATION)
        {
            app_hid_mouse_report[rpt_buf_idx][0] = mouse_rpt->buttonState;
            app_hid_mouse_report[rpt_buf_idx][1] = mouse_rpt->xMotion;
            app_hid_mouse_report[rpt_buf_idx][2] = mouse_rpt->yMotion;
            app_hid_mouse_report[rpt_buf_idx][3] = mouse_rpt->scroll;
            app_hids_mouse_in_report[0] = app_hid_mouse_report[rpt_buf_idx][0];
            app_hids_mouse_in_report[1] = app_hid_mouse_report[rpt_buf_idx][1];
            app_hids_mouse_in_report[2] = app_hid_mouse_report[rpt_buf_idx][2];
            app_hids_mouse_in_report[3] = app_hid_mouse_report[rpt_buf_idx][3];

            /* If motion event is present, continue processing it only after notification is sent */
            if(motion_activity_stat)
            {
                /* Stop the the motion data read timer and
                 * start timer again once notification sent event(GATT_HANDLE_VALUE_NOTIF) received*/
                if (pdFAIL == xTimerStop(motion_data_read_timer, TIMER_MAX_WAIT))
                {
                    printf("Failed to stop motion data read Timer\r\n");
                }
            }

            /* Send the Mouse HID report data notification */
            gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                 HDLC_HIDS_MOUSE_IN_REPORT_VALUE,
                                                                 app_hids_mouse_in_report_len,
                                                                 app_hid_mouse_report[rpt_buf_idx],
                                                                 NULL);

            /* Check if notification api call success */
            if (gatt_status != WICED_BT_GATT_SUCCESS)
            {
                /* If api call failed, Wait for notification from GATT congestion clear event */
                xTaskNotifyWait(0, 0, &ulNotifiedValue, pdMS_TO_TICKS(TASK_MAX_WAIT));
                /* Retry the HID report data notification again */
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                     HDLC_HIDS_MOUSE_IN_REPORT_VALUE,
                                                                     app_hids_mouse_in_report_len,
                                                                     app_hid_mouse_report[rpt_buf_idx],
                                                                     NULL);
            }

            rpt_buf_idx = ((rpt_buf_idx + 1) % MOUSE_REPORT_BUFF_SIZE);
        }
        else
        {
            gatt_status = WICED_BT_GATT_WRONG_STATE;
            printf("Mouse CCCD (Notification) is not enabled\n");
        }
    }
    else if (app_hids_protocol_mode[0] == PROTOCOL_BOOT_MODE)
    {
        /* Protocol mode selected, Check the cccd */
        if (app_hids_boot_mouse_input_report_client_char_config[0] == GATT_CLIENT_CONFIG_NOTIFICATION)
        {
            app_hid_mouse_report[rpt_buf_idx][0] = mouse_rpt->buttonState;
            app_hid_mouse_report[rpt_buf_idx][1] = mouse_rpt->xMotion;
            app_hid_mouse_report[rpt_buf_idx][2] = mouse_rpt->yMotion;
            app_hid_mouse_report[rpt_buf_idx][3] = mouse_rpt->scroll;
            app_hids_boot_mouse_input_report[0] = app_hid_mouse_report[rpt_buf_idx][0];
            app_hids_boot_mouse_input_report[1] = app_hid_mouse_report[rpt_buf_idx][1];
            app_hids_boot_mouse_input_report[2] = app_hid_mouse_report[rpt_buf_idx][2];
            app_hids_boot_mouse_input_report[3] = app_hid_mouse_report[rpt_buf_idx][3];

            /* If motion event is present, continue processing it only after notification is sent */
            if(motion_activity_stat)
            {
                /* Stop the the motion data read timer and
                 * start timer again once notification sent event(GATT_HANDLE_VALUE_NOTIF) received*/
                if (pdFAIL == xTimerStop(motion_data_read_timer, TIMER_MAX_WAIT))
                {
                    printf("Failed to stop motion data read Timer\r\n");
                }
            }

            /* Send the Mouse HID report data notification */
            gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                 HDLC_HIDS_BOOT_MOUSE_INPUT_REPORT_VALUE,
                                                                 app_hids_boot_mouse_input_report_len,
                                                                 app_hid_mouse_report[rpt_buf_idx],
                                                                 NULL);

            /* Check if notification api call success */
            if (gatt_status != WICED_BT_GATT_SUCCESS)
            {
                /* If api call failed, Wait for notification from GATT congestion clear event */
                xTaskNotifyWait(0, 0, &ulNotifiedValue, pdMS_TO_TICKS(TASK_MAX_WAIT));
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                     HDLC_HIDS_BOOT_MOUSE_INPUT_REPORT_VALUE,
                                                                     app_hids_boot_mouse_input_report_len,
                                                                     app_hid_mouse_report[rpt_buf_idx],
                                                                     NULL);
            }

            rpt_buf_idx = ((rpt_buf_idx + 1) % MOUSE_REPORT_BUFF_SIZE);
        }
        else
        {
            gatt_status = WICED_BT_GATT_WRONG_STATE;
            printf("Mouse CCCD (Notification) is not enabled\n");
        }
    }

    // clear mouse report data
    mouse_rpt->xMotion = mouse_rpt->yMotion = 0;
    mouse_rpt->scroll = 0;

    return gatt_status;
}

/**
 *  Function name:
 *  app_mouse_activity_handler
 *
 *  Function Description:
 *  @brief Function to handle mouse events based on current_app_state
 *
 *  @param    void
 *
 *  @return   void
 */
static void app_bt_hid_mouse_activity_handler(void)
{
    switch (current_app_state)
    {
        case PAIRED_ON:
            /* Pairing Succesful, GATT connection is not yet done */
            printf("Device is in PAIRED_ON state\r\n");
            /* Start advertisements */
            app_bt_adv_start();
            break;

        case UNPAIRED_ADVERTISING:
            /* Device is already advertising */
            break;

        case PAIRED_ADVERTISING_KNOWN_HOST:
            /* Device is already advertising */
            break;

        case PAIRED_ADVERTISING_ANY_HOST:
            /* Device is already advertising */
            break;

        case UNPAIRED_IDLE:
            /* When woke up from sleep */
            printf("Device is in UNPAIRED_IDLE state\r\n");
            /* Start advertisements to any host */
            app_bt_adv_start();
            break;

        case PAIRED_IDLE:
            /* When woke up from sleep */
            printf("Device is in PAIRED_IDLE state\r\n");
            /* Start advertisements to known host */
            app_bt_adv_start();
            break;

        case CONNECTED_NON_ADVERTISING:
            /* Device is paired and connected. Send data to connected device */
            app_bt_hid_send_mouse_report(&rpt_msg.data.mouse);
            break;

        case CONNECTED_ADVERTISING:
            /* Device is paired and connected. Sending connectable advertisements */
            break;

        case UNPAIRED_ON:
            /* Device is ON but the stack/hardware is not yet initialized */
            break;

        default:
            printf("ERROR: Unknown Remote state\r\rn");
    }
}

/**
 *  Function name:
 *  app_bt_hid_task
 *
 *  Function Description:
 *  @brief Task to handle BLE related events and send Reports through BLE GATT notifications
 *
 *  @param    pvParameters: Parameters to task
 *
 *  @return   void
 */
void app_bt_hid_task(void *pvParameters)
{
    BaseType_t xResult = pdFAIL;

    /* Create timer to trigger Disconnection if mouse idle for sometime */
    ble_disconnection_timer = xTimerCreate("Disconnection Timer",
                                           pdMS_TO_TICKS(DISCONNECTION_TIMEOUT_MS),
                                           pdFALSE,
                                           NULL,
                                           app_bt_hid_idle_disconnection_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL == ble_disconnection_timer)
    {
        printf("BLE Disconnection Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    /* Create timer to trigger connection parameter update */
    conn_param_update_timer = xTimerCreate("Conn Param Update Timer",
                                           pdMS_TO_TICKS(CONN_PARAM_UPDATE_TIMER_DELAY),
                                           pdTRUE,
                                           NULL,
                                           app_bt_hid_conn_param_update_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL == conn_param_update_timer)
    {
        printf("Connection parameter update Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    /* Initialize timer for reconnection adv */
    adv_stop_timer = xTimerCreate("Reconnection Adv Timer",
                                  pdMS_TO_TICKS(RECONNECTION_ADV_TIMEOUT_MS),
                                  pdFALSE,
                                  NULL,
                                  app_bt_adv_stop_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL == adv_stop_timer)
    {
        printf("Advertisement stop Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    while (1)
    {
        /* Main application loop waiting for events from Mouse and Batmon Tasks */
        /* Blocks until mouse event or battery value notifications are received in queue */
        /* The Mouse will be in deepsleep state if it is idle and no events to process */
        xResult = xQueueReceive(hid_rpt_q, &(rpt_msg), portMAX_DELAY);
        if (xResult != pdPASS)
        {
            continue;
        }

        /* Reset disconnection timer on each Mouse event when connected */
        if (app_bt_conn_id != 0)
        {
            if (pdFAIL == xTimerStart(ble_disconnection_timer, TIMER_MAX_WAIT))
            {
                printf("Failed to start Disconnection Timer\r\n");
            }
        }

        /* Msg from Mouse task */
        if (rpt_msg.msg_type == MOUSE_MSG_TYPE)
        {
            app_bt_hid_mouse_activity_handler();
        }

        /* Msg from Battery monitor task */
        if (rpt_msg.msg_type == BATT_MSG_TYPE)
        {
            app_bt_hid_send_batt_report(rpt_msg.data.batt_level);
        }

        /* Low battery handle */
        if (app_bas_battery_level[0] == 1)
        {
            app_bt_hid_handle_low_battery();
        }

    }
}
