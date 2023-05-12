/*******************************************************************************
 * File Name: app_button.c
 *
 * Description: This file consists of the function definitions that are
 *              necessary for developing push button use cases.
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

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/

#include "app_handler.h"
#include "app_bt_event_handler.h"
#include "app_bt_advert.h"
#include "app_bt_hid.h"
#include "timers.h"
#include "cycfg_pins.h"
#include "cyhal_gpio.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
/* Flags for button press/ release indication */
bool left_button_press = false;
bool right_button_press = false;
bool middle_button_press = false;
bool multi_button_press = false;
bool dpi_button_press = false;
bool dpi_change = false;

/* Timer handle for button debounce handle */
TimerHandle_t button_debounce_timer;
/* Timer handle for pairing mode switch */
TimerHandle_t pairing_mode_timer;
/* Timer handle for device channel switch */
#if MULTI_DEVICE_FEATURE_ENABLE
TimerHandle_t device_switch_timer;
#endif

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
void app_button_event_enable(void);
void app_button_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

/* Structure for GPIO interrupt */
cyhal_gpio_callback_data_t app_left_button_isr_data =
{
    .callback     = app_button_interrupt_handler,
    .callback_arg = NULL
};

cyhal_gpio_callback_data_t app_right_button_isr_data =
{
    .callback     = app_button_interrupt_handler,
    .callback_arg = NULL
};

cyhal_gpio_callback_data_t app_middle_button_isr_data =
{
    .callback     = app_button_interrupt_handler,
    .callback_arg = NULL
};

cyhal_gpio_callback_data_t app_dpi_button_isr_data =
{
    .callback     = app_button_interrupt_handler,
    .callback_arg = NULL
};

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/**
 *  Function name:
 *  app_button_disable
 *
 *  Function Description:
 *  @brief    Disable buttons by removing the internal pullup during hibernation state
 *
 *  @param    void
 *
 *  @return   void
 */
void app_button_disable(void)
{
    cyhal_gpio_configure(USER_BTN_LEFT, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_write(USER_BTN_LEFT, 0);

    cyhal_gpio_configure(USER_BTN_RIGHT, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_write(USER_BTN_RIGHT, 0);

    cyhal_gpio_configure(USER_BTN_MIDDLE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_write(USER_BTN_MIDDLE, 0);

    cyhal_gpio_configure(USER_BTN_DPI, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_write(USER_BTN_DPI, 0);
}

/**
 *  Function name:
 *  app_button_activity_handler
 *
 *  Function Description:
 *  @brief    Handler function for mouse button events
 *
 *  @param    void
 *
 *  @return   void
 */
void app_button_activity_handler(void)
{
    /* Check if dpi button pressed */
    if (dpi_change)
    {
        dpi_change = 0;
        /* Change the cpi in order 1026(Default)->1634->836->1026(Default)->... */

        app_motion_cpi_change();

        /* Start LED blink to indicate the DPI change */
        app_led_update_blink_period(LED_BLINK_RATE_MS);
        app_status_led_start_blinking();
    }
    else
    {
        /* Update all button state in mouse report */
        app_mouse_click_change(LEFT_BUTTON_BIT, left_button_press);
        app_mouse_click_change(RIGHT_BUTTON_BIT, right_button_press);
        app_mouse_click_change(MIDDLE_BUTTON_BIT, middle_button_press);
    }
}

/**
 *  Function name:
 *  app_button_event_enable
 *
 *  Function Description:
 *  @brief    This function is used to notify Mouse task of Button events
 *
 *  @param    void
 *
 *  @return   void
 */
void app_button_event_enable(void)
{
    /* Notify Mouse task to process the button event */
    xTaskNotify(mouse_task_h, MOUSE_BUTTON, eSetBits);
}

/**
 *  Function name:
 *  app_button_interrupt_handler
 *
 *  Function Description:
 *  @brief    Button interrupt handler
 *
 *  @param    handler_arg: Interrupt handler arguments
 *  @param    event: GPIO Event
 *
 *  @return   void
 */
void app_button_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken;

    if (pdFAIL == xTimerStartFromISR(button_debounce_timer, &xHigherPriorityTaskWoken))
    {
        printf("Failed to start Button debounce Timer\r\n");
        return;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 *  Function name:
 *  app_button_debounce_timer_cb
 *
 *  Function Description:
 *  @brief    This function is used to update current button states after debounce timeout
 *
 *  @param    cb_params: Callback parameters of timer callback
 *
 *  @return   void
 */
void app_button_debounce_timer_cb(TimerHandle_t cb_params)
{
    /* Read all the button state if DPI button is not pressed */
    dpi_button_press = !cyhal_gpio_read(USER_BTN_DPI);

    if (cyhal_gpio_read(USER_BTN_DPI))
    {
        left_button_press = !cyhal_gpio_read(USER_BTN_LEFT);
        right_button_press = !cyhal_gpio_read(USER_BTN_RIGHT);
        middle_button_press = !cyhal_gpio_read(USER_BTN_MIDDLE);
    }

    /* Check if DPI button pressed */
    if (!cyhal_gpio_read(USER_BTN_DPI))
    {
        /* Check and start pairing mode timer if not already in pairing mode */
        if ((!multi_button_press) &&
                (app_bt_hid_get_device_state() != UNPAIRED_ADVERTISING) &&
                (app_bt_hid_get_device_state() != PAIRED_ADVERTISING_ANY_HOST))
        {
            if (pdFAIL == xTimerStart(pairing_mode_timer, TIMER_MIN_WAIT))
            {
                printf("Failed to start Pairing mode switch Timer\r\n");
            }
        }
#if MULTI_DEVICE_FEATURE_ENABLE
        if ((!cyhal_gpio_read(USER_BTN_MIDDLE)) &&
                (!cyhal_gpio_read(USER_BTN_RIGHT)))
        {
            multi_button_press = 1;
            /* Stop pairing mode switch timer */
            if (pdFAIL == xTimerStop(pairing_mode_timer, TIMER_MIN_WAIT))
            {
                printf("Failed to stop Pairing mode switch Timer\r\n");
            }
            /* Send all button release event before switching device channel */
            left_button_press = 0;
            right_button_press = 0;
            middle_button_press = 0;
            dpi_button_press = 0;
            dpi_change = 0;
            /* Start timer for device channel switch timer */
            if (pdFAIL == xTimerStart(device_switch_timer, TIMER_MIN_WAIT))
            {
                printf("Failed to start device channel switch Timer\r\n");
            }
        }
#endif
    }
    else if (xTimerIsTimerActive(pairing_mode_timer))
    { /* DPI button pressed and released before pairing mode timer expiry */
        /* Change the DPI of the Mouse if in connected state */
        if (app_bt_hid_get_device_state() == CONNECTED_NON_ADVERTISING)
        {
            dpi_change = 1;
        }

        if (pdFAIL == xTimerStop(pairing_mode_timer, TIMER_MIN_WAIT))
         {
             printf("Failed to start Pairing mode switch Timer\r\n");
         }
    }

    /* Check if all buttons used for device switch are released */
    if ((multi_button_press) &&
            (cyhal_gpio_read(USER_BTN_DPI)) &&
            (cyhal_gpio_read(USER_BTN_MIDDLE)) &&
            (cyhal_gpio_read(USER_BTN_RIGHT)))
    {
        multi_button_press = 0;
    }

    /* If only DPI button is pressed, no notifications to be sent */
    if(!dpi_button_press)
    {
        /* Notify Mouse task to process the button event */
        app_button_event_enable();
    }
}

#if MULTI_DEVICE_FEATURE_ENABLE
/**
 *  Function name:
 *  app_button_device_switch_timer_cb
 *
 *  Function Description:
 *  @brief    Timer cb to switch device channel
 *
 *  @param    cb_params: Callback parameters of timer callback
 *
 *  @return   void
 */
void app_button_device_switch_timer_cb(TimerHandle_t cb_params)
{
    if ((!cyhal_gpio_read(USER_BTN_DPI)) &&
            (!cyhal_gpio_read(USER_BTN_MIDDLE)) &&
            (!cyhal_gpio_read(USER_BTN_RIGHT)))
    {
        app_bt_hid_bond_index_switch();
    }
}
#endif
/**
 *  Function name:
 *  app_button_pair_mode_switch_timer_cb
 *
 *  Function Description:
 *  @brief    Timer cb to start pairing mode advertisement
 *
 *  @param    cb_params: Callback parameters of timer callback
 *
 *  @return   void
 */
void app_button_pair_mode_switch_timer_cb(TimerHandle_t cb_params)
{
#if PAIRING_MODE_SWITCH_ENABLE
    if (!cyhal_gpio_read(USER_BTN_DPI))
    {
        app_bt_hid_pairing_mode_switch();
    }
#endif
}

/**
 *  Function name:
 *  app_button_configure_pin
 *
 *  Function Description:
 *  @brief    This function is used to initialize button GPIO's and configure interrupt
 *
 *  @param    pin: GPIO to configure
 *  @param    drive_mode: Drive mode configuration for the GPIO
 *  @param    event: GPIO events to register interrupt
 *  @param    intr_priority: Interrupt priority
 *  @param    callback_data: Callback data structure
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
static cy_rslt_t app_button_configure_pin(cyhal_gpio_t pin, cyhal_gpio_drive_mode_t drive_mode,
                                      cyhal_gpio_event_t event, uint8_t intr_priority,
                                      cyhal_gpio_callback_data_t* callback_data)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Initialize the user button */
    status = cyhal_gpio_init(pin,
                             CYHAL_GPIO_DIR_INPUT,
                             drive_mode,
                             CYBSP_BTN_OFF);

    if (CY_RSLT_SUCCESS != status)
    {
        printf("GPIO Init failed\n");
        return status;
    }

    /* Configure GPIO interrupt for User Button */
    cyhal_gpio_register_callback(pin, callback_data);

    /* Enable GPIO Interrupt on both edge for User Button */
    cyhal_gpio_enable_event(pin, event, intr_priority, TRUE);

    return status;
}

/**
 *  Function name:
 *  app_button_init
 *
 *  Function Description:
 *  @brief    This function is used to initialize all user button and related timers
 *
 *  @param    void
 *
 *  @return   void
 */
void app_button_init(void)
{
#if BUTTONS_EXTERNAL_PULLUP_ENABLED
    cyhal_gpio_drive_mode_t drive_mode = CYHAL_GPIO_DRIVE_NONE;
#else
    cyhal_gpio_drive_mode_t drive_mode = CYHAL_GPIO_DRIVE_PULLUP;
#endif

    /* Initialize the User Left Button */
    if (CY_RSLT_SUCCESS != app_button_configure_pin(USER_BTN_LEFT,
                                                drive_mode,
                                                CYHAL_GPIO_IRQ_BOTH,
                                                GPIO_INTERRUPT_PRIORITY,
                                                &app_left_button_isr_data))
    {
        printf("Left Button Init failed\n");
    }

    /* Initialize the User Right Button */
    if (CY_RSLT_SUCCESS != app_button_configure_pin(USER_BTN_RIGHT,
                                                drive_mode,
                                                CYHAL_GPIO_IRQ_BOTH,
                                                GPIO_INTERRUPT_PRIORITY,
                                                &app_right_button_isr_data))
    {
        printf("Right Button Init failed\n");
    }

    /* Initialize the User Middle Button */
    if (CY_RSLT_SUCCESS != app_button_configure_pin(USER_BTN_MIDDLE,
                                                drive_mode,
                                                CYHAL_GPIO_IRQ_BOTH,
                                                GPIO_INTERRUPT_PRIORITY,
                                                &app_middle_button_isr_data))
    {
        printf("Middle Button Init failed\n");
    }

    /* Initialize the User DPI Button */
    if (CY_RSLT_SUCCESS != app_button_configure_pin(USER_BTN_DPI,
                                                drive_mode,
                                                CYHAL_GPIO_IRQ_BOTH,
                                                GPIO_INTERRUPT_PRIORITY,
                                                &app_dpi_button_isr_data))
    {
        printf("DPI Button Init failed\n");
    }

    /* Create timer for handling button debounce */
    button_debounce_timer = xTimerCreate("Button Debounce Timer",
                                         pdMS_TO_TICKS(BUTTON_DEBOUNCE_DELAY_MS),
                                         pdFALSE,
                                         NULL,
                                         app_button_debounce_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL == button_debounce_timer)
    {
        printf("Button debounce Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    /* Create timer for handling pairing mode switch */
    pairing_mode_timer = xTimerCreate("Pairing Mode Timer",
                                      pdMS_TO_TICKS(PAIRING_MODE_SWITCH_DELAY_MS),
                                      pdFALSE,
                                      NULL,
                                      app_button_pair_mode_switch_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL == pairing_mode_timer)
    {
        printf("Pairing mode switch Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

#if MULTI_DEVICE_FEATURE_ENABLE
    /* Create timer for handling device channel switch */
    device_switch_timer = xTimerCreate("Device Channel Switch Timer",
                                      pdMS_TO_TICKS(DEVICE_SWITCH_DELAY_MS),
                                      pdFALSE,
                                      NULL,
                                      app_button_device_switch_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL == device_switch_timer)
    {
        printf("Device Channel switch Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
#endif
}

