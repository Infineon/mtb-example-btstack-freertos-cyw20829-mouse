/*******************************************************************************
 * File Name: app_bt_hid.h
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

#ifndef __APP_BT_HID_H__
#define __APP_BT_HID_H__

/*******************************************************************************
 *                                Include Headers
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "cy_utils.h"
#include "app_mouse.h"

/*******************************************************************************
 *                                Constants
 ******************************************************************************/

/* Message type to differentiate Mouse msg and Battery level msg */
#define MOUSE_MSG_TYPE                          (1u)
#define BATT_MSG_TYPE                           (2u)

/* hid queue size */
#define HID_MSG_Q_SZ                            ((uint8_t)10u)
/* hid queue item size */
#define HID_MSG_Q_ITEM_SZ                       (sizeof(struct hid_rpt_msg))

#define TASK_MAX_WAIT  1000 // Delay in ms for HID task notify wait

/*******************************************************************************
 *                                Conditional Macros
 ******************************************************************************/

#define MOUSE_REPORT_SIZE   4

#define MOUSE_REPORT_BUFF_SIZE  10
/*******************************************************************************
 *                          Variable Declarations
 ******************************************************************************/

typedef enum
{
    PROTOCOL_BOOT_MODE,
    PROTOCOL_REPORT_MODE
} hid_protocol_mode_e;

/* HID Message type : Mouse Message or Battery level */
union msg
{
    mouse_rpt_t mouse;
    uint8_t batt_level;
};

/* HID Message and HID Message type */
struct hid_rpt_msg
{
    uint8_t msg_type;
    union msg data;
};

/* State of the application */
typedef enum
{
    UNPAIRED_ON,                    /* Unpaired state when the Device is just powered up */
    PAIRED_ON,                      /* Paired state when the Device is just powered up */
    UNPAIRED_ADVERTISING,           /* Unpaired Undirected advertisement state in which the device is not bonded and making it discoverable for the hosts */
    PAIRED_ADVERTISING_KNOWN_HOST,  /* Paired Undirected advertisement state in which the device has the bond information to connect again to the host */
    PAIRED_ADVERTISING_ANY_HOST,    /* Paired Undirected advertisement state in which the device is bonded but making it discoverable for the other new hosts */
    UNPAIRED_IDLE,                  /* Unpaired Sleep state in which advertisement has timed out due to no interest from any new hosts */
    PAIRED_IDLE,                    /* Paired Sleep state in which advertisement has timed out due to no interest from any new hosts or connected host */
    CONNECTED_NON_ADVERTISING,      /* Paired Connected state in which the device is not interested in advertising */
    CONNECTED_ADVERTISING,          /* Paired Connected state in which the device is advertising to get paired to a new host */
} app_ble_state_t;

/* Task and Queue Handles of Bluetooth LE HID Mouse Application  */
extern TaskHandle_t ble_task_h;
/* Queue for sending Mouse events and battery reports to Bluetooth LE Task */
extern QueueHandle_t hid_rpt_q;
/* Timer handle for triggering connection parameter update */
extern TimerHandle_t conn_param_update_timer;
/* Current Application state */
extern app_ble_state_t current_app_state;

/*******************************************************************************
 *                          Function Declarations
 ******************************************************************************/

void app_bt_hid_task(void *pvParameters);
app_ble_state_t app_bt_hid_get_device_state(void);
void app_bt_hid_update_device_state(app_ble_state_t app_state);

/* This Function is used to switch device to pairing mode */
void app_bt_hid_pairing_mode_switch(void);
/* This Function is used to switch the current bond index */
void app_bt_hid_bond_index_switch(void);

#endif // __APP_BT_HID_H__
