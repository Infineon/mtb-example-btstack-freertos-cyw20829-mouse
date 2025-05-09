/*******************************************************************************
 * File Name: app_bt_advert.c
 *
 * Description: This File provides the implementations necessary for Bluetooth
 * Advertisements.
 *
 * Related Document: See README.md
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
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"

#include "app_bt_advert.h"
#include "app_bt_gatt_handler.h"
#include "app_bt_hid.h"

#include "app_handler.h"
#include "app_bt_utils.h"

/*******************************************************************************
 *                      MACROS / VARIABLE DEFINITIONS
 ******************************************************************************/
#define NON_DISCOVERABLE_MODE_ADV   (BTM_BLE_BREDR_NOT_SUPPORTED)
#define DISCOVERABLE_MODE_ADV       (BTM_BLE_BREDR_NOT_SUPPORTED|BTM_BLE_LIMITED_DISCOVERABLE_FLAG)
#define NUM_ADV_ELEM                (CY_BT_ADV_PACKET_DATA_SIZE)
/* Timer handle for stopping advertisement */
extern TimerHandle_t adv_stop_timer;
/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/**
 *  Function name:
 *  app_bt_adv_start
 *
 *  Function Description:
 *  @brief    Function used to start connectable advertisements once
 *  BTM_ENABLED_EVT event occurs in Bluetooth management callback
 *
 *  @param    void
 *
 *  @return   void
 */
void app_bt_adv_start(void)
{
    /* Stop ongoing adv if any */
    app_bt_adv_stop();

    /* Check if Bond Information is present */
    if (CY_RSLT_SUCCESS == app_bt_bond_check_info())
    {
        /* Bonded Device found */
        app_bt_hid_update_device_state(PAIRED_ADVERTISING_KNOWN_HOST);
        app_bt_adv_start_known_host();
    }
    else
    {
        /* No Bonded Device */
        /* Allow new devices to bond */
        app_bt_hid_update_device_state(UNPAIRED_ADVERTISING);
        app_bt_adv_start_any_host();
    }
}

/**
 *  Function name:
 *  app_bt_adv_state_handler
 *
 *  Function Description:
 *  @brief Application State change handler based on current advertisement state.
 *
 *  @param    current_adv_mode:  Current Advertisement mode of the application
 *
 *  @return   void
 */
void app_bt_adv_state_handler(wiced_bt_ble_advert_mode_t current_adv_mode)
{
    switch (current_adv_mode)
    {
        case BTM_BLE_ADVERT_OFF:
            /* If advertisements was turned off due to time out after disconnection,
             * then the previous state is PAIRED_ADVERTISING_ANY_HOST
             */
            if (app_bt_conn_id == 0)
            {
                if ((app_bt_hid_get_device_state() == PAIRED_ADVERTISING_ANY_HOST) ||
                        (app_bt_hid_get_device_state() == PAIRED_ADVERTISING_KNOWN_HOST))
                {
                    app_bt_hid_update_device_state(PAIRED_IDLE);
                    Cy_SysPm_SetDeepSleepMode(CY_SYSPM_MODE_DEEPSLEEP_RAM);
                }
                else if (app_bt_hid_get_device_state() == UNPAIRED_ADVERTISING)
                {
                    app_bt_hid_update_device_state(UNPAIRED_IDLE);

                    //Start swift pairing cool down adv
                    app_bt_adv_stop();
                    if (CY_RSLT_SUCCESS != app_bt_bond_check_info())
                    {
                        printf("Start swift pairing cool down adv \n ");
                        app_bt_efi_swift_pair_start_adv(DISCOVERABLE_MODE_ADV, true);
                    }
                }

                /* Enable deep_sleep for motion sensor to reduce power consumption in disconnected state */
                app_motion_sleep(MOTION_DEEP_SLEEP);

            }
            else
            {
                app_bt_hid_update_device_state(CONNECTED_NON_ADVERTISING);
            }
            break;
        case BTM_BLE_ADVERT_DIRECTED_HIGH:
        case BTM_BLE_ADVERT_DIRECTED_LOW:
        case BTM_BLE_ADVERT_UNDIRECTED_HIGH:
        case BTM_BLE_ADVERT_UNDIRECTED_LOW:
        case BTM_BLE_ADVERT_NONCONN_HIGH:
        case BTM_BLE_ADVERT_NONCONN_LOW:
        case BTM_BLE_ADVERT_DISCOVERABLE_HIGH:
            Cy_SysPm_SetDeepSleepMode(CY_SYSPM_MODE_DEEPSLEEP);
            break;
        default:
            printf("ERROR: Unknown advertisement state\r\n");
            break;
    }

}

static void app_bt_adv_set_data(uint8_t adv_flag)
{
    uint8_t cy_bt_adv_packet_elem_0[1] = { adv_flag };
    uint8_t cy_bt_adv_packet_elem_10[4] = { 0x06, 0x00, 0x03, 0x00 };


    /* Set the adv flag */
    cy_bt_adv_packet_data[0].p_data = (uint8_t*)cy_bt_adv_packet_elem_0;
    cy_bt_adv_packet_data[10].p_data = (uint8_t*)cy_bt_adv_packet_elem_10;

    /* Set Advertisement data without EFI payload */
    if (WICED_SUCCESS != wiced_bt_ble_set_raw_advertisement_data((NUM_ADV_ELEM), cy_bt_adv_packet_data))
    {
        printf("Setting advertisement data Failed\r\n");
    }
}

/**
 *  Function name:
 *  app_bt_adv_start_known_host
 *
 *  Function Description:
 *  @brief This Function starts undirected Bluetooth LE advertisement for reconnection to known host
 *
 *  @param   void
 *
 *  @return  void
 */
void app_bt_adv_start_known_host(void)
{
    printf("app_bt_start_adv_known_host\r\n");

    wiced_bt_device_address_t local_bdaddr;

    /* Set Advertisement Data */
    app_bt_adv_set_data(NON_DISCOVERABLE_MODE_ADV);

    /* Start timer to stop undirected adv after required duration */
    if (pdFAIL == xTimerStart(adv_stop_timer, TIMER_MIN_WAIT))
    {
        printf("Failed to start Advertisement stop Timer\r\n");
    }

    /* Update LED blink period for directed adv indication */
    app_led_update_blink_period(RECONNECTION_ADV_LED_BLINK_TIME_MS);

    /* Disable pairing mode */
    wiced_bt_set_pairable_mode(FALSE, TRUE);

    /*local bd update is needed for device switching scenario */
    app_bt_bond_get_local_bd_addr(local_bdaddr);
    wiced_bt_set_local_bdaddr(local_bdaddr, BLE_ADDR_RANDOM);

    /* Start Undirected LE Advertisements */
    if (WICED_SUCCESS != wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL))
    {
        printf("Starting undirected Bluetooth LE advertisements Failed\r\n");
    }
}

/**
 *  Function name:
 *  app_bt_adv_start_any_host
 *
 *  Function Description:
 *  @brief This Function starts undirected Bluetooth LE advertisement for pairing to new host
 *
 *  @param  void
 *
 *  @return void
 */
void app_bt_adv_start_any_host(void)
{
    printf("app_bt_start_adv_any_host\r\n");

    wiced_bt_device_address_t local_bdaddr;

    /* Start timer to stop adv after required duration */
    if (pdFAIL == xTimerStop(adv_stop_timer, TIMER_MIN_WAIT))
    {
        printf("Failed to stop Advertisement stop Timer\r\n");
    }
    /* Update LED blink period for pairing mode undirected adv indication */
    app_led_update_blink_period(PAIRING_MODE_ADV_LED_BLINK_TIME_MS);

    /* Enable pairing mode */
    wiced_bt_set_pairable_mode(TRUE, FALSE);

    /* Update the local address to be used for pairing a new host */
    app_bt_bond_get_new_bd_addr(local_bdaddr);
    wiced_bt_set_local_bdaddr(local_bdaddr, BLE_ADDR_RANDOM);

    /* Set Advertisement Data and Start Undirected LE Advertisements*/
    app_bt_efi_swift_pair_start_adv(DISCOVERABLE_MODE_ADV, false);
}

/**
 *  Function name:
 *  app_bt_adv_start_known_host_dir_adv
 *
 *  Function Description:
 *  @brief This Function starts directed Bluetooth LE advertisement for reconnection to known host
 *
 *  @param   void
 *
 *  @return  void
 */
void app_bt_adv_start_known_host_dir_adv(void)
{
    printf("app_bt_adv_start_known_host_dir_adv\r\n");

    /* Set Advertisement Data */
    app_bt_adv_set_data(NON_DISCOVERABLE_MODE_ADV);

    /* Start timer to stop undirected adv after required duration */
    if (pdFAIL == xTimerStart(adv_stop_timer, TIMER_MIN_WAIT))
    {
        printf("Failed to start Advertisement stop Timer\r\n");
    }

    /* Update LED blink period for directed adv indication */
    app_led_update_blink_period(RECONNECTION_ADV_LED_BLINK_TIME_MS);

    /* Disable pairing mode */
    wiced_bt_set_pairable_mode(FALSE, TRUE);

    /* Get the link key information */
    wiced_bt_device_link_keys_t link_keys;
    app_bt_bond_get_device_link_keys(&link_keys);

    printf("Peer addr type : %d Peer Device BD ADDR: ", link_keys.key_data.ble_addr_type);
    app_bt_util_print_bd_address(link_keys.bd_addr);

    /*local bd update is needed for device switching scenario */
    wiced_bt_device_address_t local_bdaddr;
    app_bt_bond_get_local_bd_addr(local_bdaddr);
    wiced_bt_set_local_bdaddr(local_bdaddr, BLE_ADDR_RANDOM);

    /* Start Directed LE Advertisements */
    if (WICED_SUCCESS != wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_HIGH,
                         link_keys.key_data.ble_addr_type,
                         link_keys.bd_addr))
    {
        printf("Starting Directed Bluetooth LE advertisements Failed\r\n");
    }
}

/**
 *  Function name:
 *  app_bt_adv_stop
 *
 *  Function Description:
 *  @brief  Function used to stop ongoing Bluetooth LE advertisement
 *
 *  @param    void
 *
 *  @return    void
 */
void app_bt_adv_stop(void)
{
    printf("app_bt_adv_stop\r\n");

    if (WICED_SUCCESS != wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF,
                                                       BLE_ADDR_PUBLIC,
                                                       NULL))
    {
        printf("Stop Bluetooth LE advertisements Failed\r\n");
    }
}

/**
 *  Function name:
 *  app_bt_adv_stop_timer_cb
 *
 *  Function Description:
 *  @brief Timer cb to stop ongoing Bluetooth LE adv.
 *
 *  @param    cb_param: Argument to cb
 *
 *  @return   void
 */
void app_bt_adv_stop_timer_cb(TimerHandle_t cb_params)
{
    /* Stop ongoing adv if any */
    app_bt_adv_stop();
}
