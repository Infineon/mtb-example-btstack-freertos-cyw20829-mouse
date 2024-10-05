/*******************************************************************************
 * File Name: main.c
 *
 * Description:
 * This file contains the starting point of Bluetooth LE HID Mouse application.
 * The wiced_bt_stack_init() registers for Bluetooth events in this main function.
 * The Bluetooth Management callback manages the Bluetooth events and the
 * application developer can customize the functionality and behavior depending on
 * the Bluetooth events. The Bluetooth Management callback acts like a
 * Finite State Machine (FSM) for the SoC.
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
 *                               Header Files
 *******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cy_sysclk.h"
#include "cybsp_smif_init.h"

/* FreeRTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "timers.h"

#include "cyhal_wdt.h"

#include "wiced_memory.h"
#include "wiced_bt_stack.h"

/* BT header files */
#include "cybt_platform_trace.h"
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "app_bt_cfg.h"
#include "app_bt_event_handler.h"
#include "app_bt_hid.h"

#include "app_handler.h"
#include "app_batmon.h"

#include "cy_retarget_io.h"

/* OTA header files */
#ifdef ENABLE_OTA
#include "cy_log.h"
#include "cy_ota_api.h"
#include "app_ota_context.h"
#include "cy_ota_storage_api.h"
#include "cybsp_bt_config.h"
#include "cy_ota_api.h"
#endif

/*******************************************************************************
 *                               Macro Definitions
 *******************************************************************************/
/* FreeRTOS Task Configurations of various Bluetooth LE HID Mouse Tasks */

/* Task names for Bluetooth LE HID Mouse tasks */
#define BLE_TASK_NAME                   "BLE Task"
#define MOUSE_TASK_NAME                 "Mouse Task"
#define BATMON_TASK_NAME                "Battery Monitoring Task"

/* Stack sizes for Bluetooth LE HID Mouse tasks */
#define BLE_TASK_STACK_SIZE                 (512u)
#define MOUSE_TASK_STACK_SIZE               (512u)
#define BATMON_TASK_STACK_SIZE              (256u)

/* Task Priorities of Bluetooth LE HID Mouse Tasks */
#define MOUSE_TASK_PRIORITY                 (3)
#define BLE_TASK_PRIORITY                   (2)
#define BATMON_TASK_PRIORITY                (1)

/* Sufficient Heap size for Bluetooth activities */
#define BT_HEAP_SIZE                        (0x1000)

/*******************************************************************************
 *                           Global Variables
 *******************************************************************************/

/*******************************************************************************
 *                           Function Prototypes
 *******************************************************************************/
/* Function to initialize the various tasks for the Bluetooth LE application */
static void app_tasks_init(void);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
/**
 *  Function name:
 *  app_tasks_init
 *
 *  Function Description:
 *  @brief    This function initializes all the FreeRTOS tasks needed for the HID Mouse
 *            application.
 *
 *  @param    void
 *
 *  @return   void
 */
static void app_tasks_init(void)
{

#if ENABLE_LOGGING
    /* Initialize retarget-io to use the debug UART port */
    cy_rslt_t status = cy_retarget_io_init(CYBSP_DEBUG_UART_TX,
                                           CYBSP_DEBUG_UART_RX,
                                           CY_RETARGET_IO_BAUDRATE);
    if(CY_RSLT_SUCCESS != status)
    {
        CY_ASSERT(0);
    }
#endif

    /* Initializing the HCI UART for Host control */
    cybt_platform_config_init(&app_bt_platform_cfg_settings);


#ifdef ENABLE_OTA
    cy_rslt_t cy_result;
#if (ENABLE_LOGGING == true)
    cy_log_init(CY_LOG_INFO, NULL, NULL);
    cy_ota_set_log_level(CY_LOG_INFO);;
#else
    cy_log_init(CY_LOG_OFF, NULL, NULL);
#endif
#endif



    /* Debug logs on UART port */
    printf("\r\n****** Bluetooth LE HID Mouse Application******\r\n ");
    printf("\r\nThis application implements HoGP and sends HID reports on Mouse events over BLE \r\n");
    printf("\r\nDiscover this device with the name:%s\r\n", app_gap_device_name);

    /* Initialize SMIF interface */
    if (cybsp_smif_init() != CY_SMIF_SUCCESS)
    {
        printf("ERROR returned from cybsp_smif_init()!!!\r\n");
    }

#ifdef ENABLE_OTA

    app_ota_initialize_default_values();

    /* Need to init from every ext flash write
     * See serial_flash.h in app for more details.
     */
    printf("call cy_ota_storage_init()\n");
    if (cy_ota_storage_init() != CY_RSLT_SUCCESS)
    {
        printf("ERROR returned from cy_ota_storage_init()!!!!!\n");
    }
    /* Validate the update so we do not revert on reboot */
    cy_result =cy_ota_storage_image_validate(0);
    if (cy_result != CY_RSLT_SUCCESS)
     {
         printf("cy_ota_storage_image_validate() Failed\n");
     }
     else
     {
         printf("cy_ota_storage_image_validate() Successful\n");
     }
#endif

#ifdef ENABLE_OTA
    /* Verify that the Non-Secure / Secure build choice matches the Bluetooth? Configurator output */
    if (app_ota_verify_ota_uuid() != CY_RSLT_SUCCESS)
    {
        printf("Failed configurator check \r \n");
        while(true)
        {
            cy_rtos_delay_milliseconds(1000);
        }

    }
#endif

    /* Initialize kv_store library */
    app_flash_kv_store_init();

    /* Configure Bluetooth LE configuration & registers Bluetooth LE event callback function
     * with the BT stack
     */
    if (WICED_BT_SUCCESS != wiced_bt_stack_init(app_bt_event_management_callback, &wiced_bt_cfg_settings))
    {
        /* Check if stack initialization was successful */
        printf("Bluetooth Stack Initialization failed!!\r\n");
    }

    /* Create a buffer heap, make it the default heap.  */
    if ( NULL == wiced_bt_create_heap("app", NULL, BT_HEAP_SIZE, NULL, WICED_TRUE))
    {
        printf("Heap create Failed");
    }

    /* Initialize Queue to send messages to BLE Task */
    hid_rpt_q = xQueueCreate(HID_MSG_Q_SZ, HID_MSG_Q_ITEM_SZ);

    if (NULL == hid_rpt_q)
    {
        printf("HID Report Queue creation failed! \r\n");
        CY_ASSERT(0);
    }

    /* Initialize HID BLE Task */
    if ( pdPASS != xTaskCreate(app_bt_hid_task,
                               BLE_TASK_NAME,
                               BLE_TASK_STACK_SIZE,
                               NULL,
                               BLE_TASK_PRIORITY,
                               &ble_task_h))
    {
        /* Task is not created due to insufficient Heap memory.
         * Use vApplicationMallocFailedHook() callback to trap.
         * And xPortGetFreeHeapSize() to query unallocated heap memory
         */
        printf("HID Bluetooth LE Task creation failed");
        CY_ASSERT(0);
    }

    /* Initialize Mouse task */
    if ( pdPASS != xTaskCreate(app_mouse_task,
                               MOUSE_TASK_NAME,
                               MOUSE_TASK_STACK_SIZE,
                               NULL,
                               MOUSE_TASK_PRIORITY,
                               &mouse_task_h))
    {
        /* Task is not created due to insufficient Heap memory.
         * Use vApplicationMallocFailedHook() callback to trap.
         * And xPortGetFreeHeapSize() to query unallocated heap memory
         */
        printf("Mouse Task creation failed\n");
        CY_ASSERT(0);
    }

    /* Initialize the Battery Monitoring task */
    if ( pdPASS != xTaskCreate(app_batmon_task,
                               BATMON_TASK_NAME,
                               BATMON_TASK_STACK_SIZE,
                               NULL,
                               BATMON_TASK_PRIORITY,
                               &batmon_task_h))
    {
        /* Task is not created due to insufficient Heap memory.
         * Use vApplicationMallocFailedHook() callback to trap.
         * And xPortGetFreeHeapSize() to query unallocated heap memory
         */
        printf("Battery Monitoring Task creation Failed! \r\n");
        CY_ASSERT(0);
    }

    /* Initialize the status LED for connection and advertisement */
    app_status_led_init();
    /* Initialize the mouse components */
    app_button_init();

    app_motion_sensor_init();

    app_quaddec_init();
}

/**
 *  Function name:
 *  main
 *
 *  Function Description:
 *  @brief    Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 *
 *  @param    void
 *
 *  @return   int: Return status
 */
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

#ifdef ENABLE_OTA
       cyhal_wdt_t  wdt_obj;
#endif

    /* Initialize the device and board peripherals */
    if(result != cybsp_init())
    {
        CY_ASSERT(0);
    }

#ifdef ENABLE_OTA
    cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the tasks */
    app_tasks_init();
    printf("Application tasks initialized\r\n");

#ifdef ENABLE_OTA
    printf("OTA Version %d %d %d \r\n",APP_VERSION_MAJOR,APP_VERSION_MINOR,APP_VERSION_BUILD);
#endif

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    printf("Scheduler exited unexpectedly\r\n");
    CY_ASSERT(0);
}

/* [] END OF FILE */
