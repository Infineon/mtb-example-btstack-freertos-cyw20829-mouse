/*******************************************************************************
 * File Name: app_scroll.c
 *
 * Description: This file consists of the function definitions that are
 *              necessary for developing quadrature use cases.
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
#include "cyhal.h"
#include "app_handler.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

void app_quaddec_za_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
void app_quaddec_zb_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
/* QUADDEC object */
cyhal_quaddec_t quaddec_h;

/* Structure for ZA GPIO interrupt */
cyhal_gpio_callback_data_t app_quaddec_za_isr_data =
{
    .callback     = app_quaddec_za_interrupt_handler,
    .callback_arg = NULL
};
/* Structure for ZB GPIO interrupt */
cyhal_gpio_callback_data_t app_quaddec_zb_isr_data =
{
    .callback     = app_quaddec_zb_interrupt_handler,
    .callback_arg = NULL
};

/* Timer handle for Quadrature data read */
TimerHandle_t quaddec_data_read_timer;
TimerHandle_t quaddec_stop_timer;

volatile bool quadenc_za, quadenc_zb;
volatile bool quadenc_za_change, quadenc_zb_change;
/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
void app_quaddec_data_fetch_timer_cb(TimerHandle_t cb_params);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
/**
 *  Function name:
 *  app_scroll_disable
 *
 *  Function Description:
 *  @brief    Function to disable scroll during disconnected state
 *            To avoid current leakage in disconnected state
 *
 *  @param    void
 *
 *  @return   void
 */
void app_scroll_disable(void)
{
    /* To avoid current leakage in disconnected state */
    /* Disable GPIO Interrupt on falling/rising edge for Encoder A */
    cyhal_gpio_enable_event(ENCODER_ZA,
                            CYHAL_GPIO_IRQ_BOTH,
                            GPIO_INTERRUPT_PRIORITY,
                            false);

    /* Disable GPIO Interrupt on falling/rising edge for Encoder B */
    cyhal_gpio_enable_event(ENCODER_ZB,
                            CYHAL_GPIO_IRQ_BOTH,
                            GPIO_INTERRUPT_PRIORITY,
                            false);

    /* Disable internal pullup for ZA and ZB pins */
    cyhal_gpio_configure(ENCODER_ZA, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_configure(ENCODER_ZB, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);

    cyhal_gpio_write(ENCODER_ZA, 0);
    cyhal_gpio_write(ENCODER_ZB, 0);

#if QUAD_ENCODER_EXT_CONTROL_ENABLED
    cyhal_gpio_write(ENCODER_COM, 0);
#endif
}

/**
 *  Function name:
 *  app_scroll_enable
 *
 *  Function Description:
 *  @brief    Function to enable scroll during connected state
 *
 *  @param    void
 *
 *  @return   void
 */
void app_scroll_enable(void)
{
#if QUAD_ENCODER_EXT_CONTROL_ENABLED
    /* Enable Internal pullups for ZA and ZB pins */
    cyhal_gpio_configure(ENCODER_ZA, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE);
    cyhal_gpio_configure(ENCODER_ZB, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE);
    /* Enable External pullups for ZA and ZB pins */
    cyhal_gpio_write(ENCODER_COM, 1);
#else
    /* Enable Internal pullups for ZA and ZB pins */
    cyhal_gpio_configure(ENCODER_ZA, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP);
    cyhal_gpio_configure(ENCODER_ZB, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP);
#endif
    /* Enable input for ZA and ZB pins */
    cyhal_gpio_write(ENCODER_ZA, 1);
    cyhal_gpio_write(ENCODER_ZB, 1);

    /* Enable GPIO Interrupt on falling/rising edge for Encoder A */
    cyhal_gpio_enable_event(ENCODER_ZA,
                            CYHAL_GPIO_IRQ_BOTH,
                            GPIO_INTERRUPT_PRIORITY,
                            TRUE);

    /* Enable GPIO Interrupt on falling/rising edge for Encoder B */
    cyhal_gpio_enable_event(ENCODER_ZB,
                            CYHAL_GPIO_IRQ_BOTH,
                            GPIO_INTERRUPT_PRIORITY,
                            TRUE);

    /* Save current state of Encode pins for scroll direction finding later */
    quadenc_za = cyhal_gpio_read(ENCODER_ZA);
    quadenc_zb = cyhal_gpio_read(ENCODER_ZB);
}

/**
 *  Function name:
 *  app_quaddec_data_fetch_timer_cb
 *
 *  Function Description:
 *  @brief    Timer callback for handling scroll event after chattering window
 *
 *  @param    cb_params: Callback parameters for timer callback
 *
 *  @return   void
 */
void app_quaddec_data_fetch_timer_cb(TimerHandle_t cb_params)
{
    int8_t delta = 0;

    if (quadenc_za_change)
    {
        quadenc_za_change = 0;

        if (quadenc_za != cyhal_gpio_read(ENCODER_ZA))
        {
            quadenc_za = cyhal_gpio_read(ENCODER_ZA);

            if (quadenc_za)
            {
                delta = (quadenc_zb) ? -1 : 1;
            }
            else
            {
                delta = (quadenc_zb) ? 1 : -1;
            }

            app_mouse_quadrature_change(delta);

            /* Notify Mouse task to handle mouse scroll event */
            xTaskNotify(mouse_task_h, MOUSE_SCROLL, eSetBits);
        }
    }
    if (quadenc_zb_change)
    {
        quadenc_zb_change = 0;
        quadenc_zb = cyhal_gpio_read(ENCODER_ZB);
    }
}

/**
 *  Function name:
 *  app_quadrature_event_enable
 *
 *  Function Description:
 *  @brief    Function used to notify mouse task of scroll events
 *
 *  @param    void
 *
 *  @return   void
 */
static void app_quaddec_event_enable(void)
{
    BaseType_t xHigherPriorityTaskWoken;

    xTaskNotifyFromISR(mouse_task_h, MOUSE_SCROLL, eSetBits, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

/**
 *  Function name:
 *  app_quaddec_za_interrupt_handler
 *
 *  Function Description:
 *  @brief    Interrupt handler for Encode ZA pin
 *
 *  @param    handler_arg: Arguments to the interrupt handler
 *  @param    event: GPIO event
 *
 *  @return   void
 */
void app_quaddec_za_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken;

    quadenc_za_change = 1;

    if (pdFAIL == xTimerStartFromISR(quaddec_data_read_timer, &xHigherPriorityTaskWoken))
    { /* Notify the Scroll event directly if timer start failed */
        app_quaddec_event_enable();
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 *  Function name:
 *  app_quaddec_zb_interrupt_handler
 *
 *  Function Description:
 *  @brief    Interrupt handler for Encode ZB pin
 *
 *  @param    handler_arg: Arguments to the interrupt handler
 *  @param    event: GPIO event
 *
 *  @return   void
 */
void app_quaddec_zb_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken;

    quadenc_zb_change = 1;

    if (pdFAIL == xTimerStartFromISR(quaddec_data_read_timer, &xHigherPriorityTaskWoken))
    { /* Notify the Scroll event directly if timer start failed */
        app_quaddec_event_enable();
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 *  Function name:
 *  app_quaddec_init
 *
 *  Function Description:
 *  @brief    Function to Initialize the Quadrature decoder pins and configure interrupts
 *
 *  @param    void
 *
 *  @return   void
 */
void app_quaddec_init(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Initialize the Encoder A pin */
    status = cyhal_gpio_init(ENCODER_ZA,
                             CYHAL_GPIO_DIR_INPUT,
                             CYHAL_GPIO_DRIVE_NONE,
                             1);

    if (CY_RSLT_SUCCESS != status)
    {
        CY_ASSERT(false);
    }

    /* Initialize the Encoder B pin */
    status = cyhal_gpio_init(ENCODER_ZB,
                             CYHAL_GPIO_DIR_INPUT,
                             CYHAL_GPIO_DRIVE_NONE,
                             1);

    if (CY_RSLT_SUCCESS != status)
    {
        CY_ASSERT(false);
    }

#if QUAD_ENCODER_EXT_CONTROL_ENABLED
    /* Initialize the Encoder control pin */
    status = cyhal_gpio_init(ENCODER_COM,
                             CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG,
                             0);

    if (CY_RSLT_SUCCESS != status)
    {
        CY_ASSERT(false);
    }
#endif

    /* Configure GPIO interrupt on Encoder A */
    cyhal_gpio_register_callback(ENCODER_ZA, &app_quaddec_za_isr_data);

    /* Configure GPIO interrupt on Encoder B */
    cyhal_gpio_register_callback(ENCODER_ZB, &app_quaddec_zb_isr_data);

    /* Enable Quad decoder briefly to save the current encoder line states */
    app_scroll_enable();
#if (!WAKEUP_ON_SCROLL_ENABLED)
    app_scroll_disable();
#endif
    /* Create timer for Quadrature decoder data read */
    quaddec_data_read_timer = xTimerCreate("Quad decoder Data Read Timer",
                                           pdMS_TO_TICKS(QUADDEC_DATA_READ_INTERVAL),
                                           pdFALSE,
                                           NULL,
                                           app_quaddec_data_fetch_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL == quaddec_data_read_timer)
    {
        printf("Quadrature data read Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    printf("Quaddec started successfully\n");
}
