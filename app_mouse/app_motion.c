/*******************************************************************************
 * File Name: app_motion.c
 *
 * Description: This file consists of the function definitions that are
 *              necessary for developing mouse optical sensor use cases.
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
#include <inttypes.h>
#include "app_motion.h"
#include "app_handler.h"
#include "app_soft_spi.h"
#include "app_paw3212.h"
#include "app_bt_hid.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

/* SPI object */
#if (HW_SPI_ENABLED == true) && (ENABLE_LOGGING == false)
cyhal_spi_t mSPI;
#endif

/* Timer handle for motion data read */
TimerHandle_t motion_data_read_timer;

/* Flag is set when motion trigger received */
volatile bool motion_activity_stat = false;
/* Structure containing Motion data read from sensor */
motion_sensor_data_t motion_data = { .cpi = MOTION_CPI_1026 };

uint8_t motion_cpi_mode[BOND_MAX];

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
static uint8_t app_motion_next_cpi(void);
static void app_motion_event_enable(void);
static void app_motion_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

/* Structure for GPIO interrupt */
cyhal_gpio_callback_data_t app_motion_isr_data =
{
    .callback     = app_motion_interrupt_handler,
    .callback_arg = NULL
};

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
/**
 *  Function name:
 *  app_motion_update_read_interval
 *
 *  Function Description:
 *  @brief    Function to update motion sensor data read interval when motion interrupt is present
 *
 *  @param    conn_interval: Connection interval received from peer device
 *
 *  @return   void
 */
void app_motion_update_read_interval(uint16_t conn_interval)
{
    /* Convert connection interval to ms */
    conn_interval = ((conn_interval*125)/100);

    xTimerChangePeriod(motion_data_read_timer,
                       pdMS_TO_TICKS(conn_interval), TIMER_MAX_WAIT);
}

/**
 *  Function name:
 *  app_motion_read_cpi_mode_data
 *
 *  Function Description:
 *  @brief    Function to Read CPI mode of all channels from flash
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_motion_read_cpi_mode_data(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint32_t data_size = sizeof(motion_cpi_mode);

    cy_rslt_t rslt = mtb_kvstore_read( &kvstore_obj,
                                        "cpi_data",
                                        (uint8_t *)&motion_cpi_mode,
                                        &data_size);
    if (CY_RSLT_SUCCESS == rslt)
    {
        printf("CPI mode data read from Flash \r\n");
    }
    else
    {
        printf("Flash Read Error,Error code: %" PRIu32 "\r\n", rslt);
    }

    return status;
}

/**
 *  Function name:
 *  app_motion_save_cpi_mode
 *
 *  Function Description:
 *  @brief    Function to Save CPI mode of the selected channel
 *
 *  @param    cpi_mode: CPI mode of the motion sensor
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_motion_save_cpi_mode(uint8_t cpi_mode)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    motion_cpi_mode[app_bt_bond_get_index()] = cpi_mode;

    cy_rslt_t rslt = mtb_kvstore_write( &kvstore_obj,
                                        "cpi_data",
                                        (uint8_t *)&motion_cpi_mode,
                                        sizeof(motion_cpi_mode));
    if (CY_RSLT_SUCCESS == rslt)
    {
        printf("CPI mode data saved to Flash \r\n");
    }
    else
    {
        printf("Flash Write Error,Error code: %" PRIu32 "\r\n", rslt);
    }

    return status;
}

/**
 *  Function name:
 *  app_motion_get_cpi_mode
 *
 *  Function Description:
 *  @brief    Function to Get CPI mode of the selected channel
 *
 *  @param    void
 *
 *  @return   uint8_t: Current CPI mode set in motion sensor
 */
uint8_t app_motion_get_cpi_mode(void)
{
    return motion_cpi_mode[app_bt_bond_get_index()];
}

/**
 *  Function name:
 *  app_motion_restore_cpi_mode
 *
 *  Function Description:
 *  @brief    Function to Restore the CPI mode of the selected channel
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_motion_restore_cpi_mode(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Get the next CPI mode to use */
    motion_data.cpi = app_motion_get_cpi_mode();
    /* Update the CPI setting of the sensor */
    status = app_paw3212_update_config(PAW3212_CONFIG_CPI, app_paw3212_cpi_val(motion_data.cpi));

    return status;
}

/**
 *  Function name:
 *  app_motion_disable
 *
 *  Function Description:
 *  @brief    Function to Disable motion by powering down the sensor
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_motion_disable(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Disable GPIO Interrupt on both edge for MOTION IRQ */
    cyhal_gpio_enable_event(PAW3212_MOTION_PIN,
                            CYHAL_GPIO_IRQ_FALL,
                            GPIO_INTERRUPT_PRIORITY,
                            FALSE);

    /* Send power down command to the sensor */
    status = app_paw3212_power_down();

    /* Configure Motion trigger pin as output and set high to prevent current leakage */
    cyhal_gpio_configure(PAW3212_MOTION_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_write(PAW3212_MOTION_PIN, 1);

    /* Configure Motion data transfer pins as output and set low to prevent current leakage */
    cyhal_gpio_configure(mSPI_MISO, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_write(mSPI_MISO, 0);
    /* Configure Motion data transfer pins as output and set low to prevent current leakage */
    cyhal_gpio_configure(mSPI_MOSI, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_write(mSPI_MOSI, 0);

    /* Stop active timers if any */
    if (pdFAIL == xTimerStop(motion_data_read_timer, TIMER_MAX_WAIT))
    {
        printf("Failed to stop motion data read Timer\r\n");
    }

    return status;
}

/**
 *  Function name:
 *  app_motion_sleep
 *
 *  Function Description:
 *  @brief    Function to configure the sleep modes of motion sensor for better response or
 *            lower power consumption depending on device state
 *
 *  @param    sleep_mode: Sleep mode to be configured in the motion sensor
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_motion_sleep(motion_sleep_mode_e sleep_mode)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    if(MOTION_LIGHT_SLEEP == sleep_mode)
    {
        status = app_paw3212_light_sleep();
    }
    else if(MOTION_DEEP_SLEEP == sleep_mode)
    {
        status = app_paw3212_deep_sleep();
    }

    return status;
}

/**
 *  Function name:
 *  app_motion_next_cpi
 *
 *  Function Description:
 *  @brief    Function to Get the next CPI mode of PAW3212 sensor
 *
 *  @param    void
 *
 *  @return   uint16_t: Next CPI mode to be set in motion sensor
 */
static uint8_t app_motion_next_cpi(void)
{
    uint8_t cpi = motion_data.cpi;

    switch (cpi)
    {
        case MOTION_CPI_836:
            cpi = MOTION_CPI_1026;
            break;
        case MOTION_CPI_1026:
            cpi = MOTION_CPI_1634;
            break;
        case MOTION_CPI_1634:
            cpi = MOTION_CPI_836;
            break;
        default:
            cpi = MOTION_CPI_1026;
            break;
    }

    return cpi;
}

/**
 *  Function name:
 *  app_motion_cpi_change
 *
 *  Function Description:
 *  @brief    Function to Update the PAW3212 sensor CPI to the next mode
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_motion_cpi_change(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Get the next CPI mode to use */
    motion_data.cpi = app_motion_next_cpi();
    /* Update the CPI setting of the sensor */
    status = app_paw3212_update_config(PAW3212_CONFIG_CPI, app_paw3212_cpi_val(motion_data.cpi));
    /* Save the CPI to flash */
    app_motion_save_cpi_mode(motion_data.cpi);

    if(app_bt_hid_get_device_state() == CONNECTED_NON_ADVERTISING)
    {
        for(int no_blinks=0; no_blinks < motion_data.cpi; no_blinks++)
        {
            cyhal_gpio_write(STATUS_LED, CYBSP_LED_STATE_ON);
            Cy_SysLib_Delay(500);
            cyhal_gpio_write(STATUS_LED, CYBSP_LED_STATE_OFF);
            Cy_SysLib_Delay(500);
        }
    }

    return status;
}

/**
 *  Function name:
 *  app_motion_cpi_reset
 *
 *  Function Description:
 *  @brief    Function to Reset the PAW3212 sensor CPI to the default mode
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_motion_cpi_reset(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Set the CPI mode to default */
    motion_data.cpi = MOTION_CPI_1026;
    /* Update the CPI setting of the sensor */
    status = app_paw3212_update_config(PAW3212_CONFIG_CPI, app_paw3212_cpi_val(motion_data.cpi));
    /* Save the CPI to flash */
    app_motion_save_cpi_mode(motion_data.cpi);

    return status;
}

/**
 *  Function name:
 *  app_motion_data_fetch
 *
 *  Function Description:
 *  @brief    Function to Read motion data from the PAW3212 sensor
 *
 *  @param    void
 *
 *  @return   cy_rslt_t
 */
cy_rslt_t app_motion_data_fetch(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    status = app_paw3212_sample_fetch_3_wire(); // For 3 wire SPI

    return status;
}

/**
 *  Function name:
 *  app_motion_activity_handler
 *
 *  Function Description:
 *  @brief    Handler function for motion event trigger from the PAW3212 sensor
 *
 *  @param    void
 *
 *  @return   void
 */
void app_motion_activity_handler(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Fetch motion data from sensor */
    status = app_motion_data_fetch();

    /* Process the motion data if read successful */
    if (status == CY_RSLT_SUCCESS)
    {
        app_paw3212_motion_data_handle(&motion_data);
    }

    /* Check if motion data pending to be read */
    if ((!cyhal_gpio_read(PAW3212_MOTION_PIN)) ||
           (0 != motion_data.motion_status))
    {
        motion_activity_stat = true;
        /* Update the motion data in mouse report */
        app_mouse_motion_change(motion_data.delta_x, motion_data.delta_y);
        motion_data.delta_x = 0;
        motion_data.delta_y = 0;

        /* Start timer to get next motion data from sensor */

        if (pdFAIL == xTimerStart(motion_data_read_timer, TIMER_MAX_WAIT))
        { /* Notify the motion event directly if timer start failed */
            xTaskNotify(mouse_task_h, MOUSE_MOTION, eSetBits);
        }
    }
    else
    {
        motion_activity_stat = false;
    }
}

/**
 *  Function name:
 *  app_motion_data_fetch_timer_cb
 *
 *  Function Description:
 *  @brief    Timer callback for motion event notification
 *
 *  @param    cb_params: Callback params for the timer callback
 *
 *  @return   void
 */
void app_motion_data_fetch_timer_cb(TimerHandle_t cb_params)
{
    /* Notify the mouse task to process motion event */
    xTaskNotify(mouse_task_h, MOUSE_MOTION, eSetBits);
}

/**
 *  Function name:
 *  app_motion_interrupt_handler
 *
 *  Function Description:
 *  @brief    Mouse Motion interrupt handler
 *
 *  @param    handler_arg: Arguments to interrupt handler
 *  @param    event: GPIO event
 *
 *  @return   void
 */
static void app_motion_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    /* Start timer to handle motion event if not already done */
    motion_activity_stat = true;

    BaseType_t xHigherPriorityTaskWoken;

    if (pdFAIL == xTimerStartFromISR(motion_data_read_timer, &xHigherPriorityTaskWoken))
    { /* Notify the motion event directly if timer start failed */
        app_motion_event_enable();
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 *  Function name:
 *  app_motion_event_enable
 *
 *  Function Description:
 *  @brief    Function to notify the motion event to the mouse task
 *
 *  @param    void
 *
 *  @return   void
 */
static void app_motion_event_enable(void)
{
    BaseType_t xHigherPriorityTaskWoken;

    xTaskNotifyFromISR(mouse_task_h, MOUSE_MOTION, eSetBits, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}
#if (HW_SPI_ENABLED == true) && (ENABLE_LOGGING == false)
/**
 *  Function name:
 *  app_spi_interrupt_callback
 *
 *  Function Description:
 *  @brief    SPI Interrupt callback received(if enabled) when data transfer is done
 *
 *  @param    arg: Arguments to interrupt handler
 *  @param    event: SPI event
 *
 *  @return   void
 */
static void app_spi_interrupt_callback(void *arg, cyhal_spi_event_t event)
{
    (void)arg;
    if ((event & CYHAL_SPI_IRQ_DONE) != 0u)
    {
        // Transmission is complete. Handle Event
    }
}
#endif
/**
 *  Function name:
 *  app_motion_spi_init
 *
 *  Function Description:
 *  @brief    Function to Initialize SPI peripheral
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_motion_spi_init(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

#if (HW_SPI_ENABLED == true) && (ENABLE_LOGGING == false)
    /* Initialize SPI pins */
    status = cyhal_spi_init(&mSPI, mSPI_MOSI, mSPI_MISO, mSPI_SCLK,
                            mSPI_SS, NULL, 8, CYHAL_SPI_MODE_00_MSB, false);

    if (status != CY_RSLT_SUCCESS)
    {
        printf("Error Configuring SPI master...\r\n");
        return status;
    }

    /* Configure the SPI Frequency */
    status = cyhal_spi_set_frequency(&mSPI, SPI_FREQ_HZ);

    if (status != CY_RSLT_SUCCESS)
    {
        printf("Error Configuring SPI frequency...\r\n");
        return status;
    }

    // Register a callback function to be called when the interrupt fires
    cyhal_spi_register_callback(&mSPI,
                                (cyhal_spi_event_callback_t)app_spi_interrupt_callback,
                                NULL);
    // Enable the events that will trigger the call back function - All Disabled
    cyhal_spi_enable_event(&mSPI, CYHAL_SPI_IRQ_DONE, SPI_INTERRUPT_PRIORITY, false);
#else
    /* Initialize Soft SPI */
    status = app_soft_spi_init(mSPI_MISO, mSPI_SCLK, mSPI_SS, SPI_FREQ_HZ);

    if (status != CY_RSLT_SUCCESS)
    {
        printf("Error Configuring SPI master...\r\n");
        return status;
    }
#endif

    return status;
}

/**
 *  Function name:
 *  app_motion_pin_init
 *
 *  Function Description:
 *  @brief    Function to Initialize Motion Interrupt pins
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_motion_pin_init(void)
{
    cy_rslt_t status;

    /* Initialize the PAW3212 MOTION IRQ pin */
    status = cyhal_gpio_init(PAW3212_MOTION_PIN,
                             CYHAL_GPIO_DIR_INPUT,
                             CYHAL_GPIO_DRIVE_NONE,
                             1);

    if (CY_RSLT_SUCCESS != status)
    {
        printf("MOTION Pin Init failed\r\n");
        return status;
    }

    /* Configure GPIO interrupt for MOTION IRQ Pin */
    cyhal_gpio_register_callback(PAW3212_MOTION_PIN, &app_motion_isr_data);

    /* Enable GPIO Interrupt on both edge for MOTION IRQ */
    cyhal_gpio_enable_event(PAW3212_MOTION_PIN,
                            CYHAL_GPIO_IRQ_FALL,
                            GPIO_INTERRUPT_PRIORITY,
                            TRUE);

    return status;
}

/**
 *  Function name:
 *  app_motion_sensor_init
 *
 *  Function Description:
 *  @brief    Initialize Motion sensor
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_motion_sensor_init(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Initialize Motion Interrupt pin */
    status = app_motion_pin_init();
    if (status)
    {
        return status;
    }

    /* Initialize SPI */
    status = app_motion_spi_init();
    if (status)
    {
        CY_ASSERT(false);
        return status;
    }

    /* Reset and initialize motion sensor */
    status = app_paw3212_power_up_reset();
    if (status)
    {
        return status;
    }

    /* Verify the chip id */
    status = app_paw3212_verify_id();
    if (status)
    {
        CY_ASSERT(false);
        return status;
    }

    /* Configure motion sensor mode and orientation */
    status = app_paw3212_mode_configure();
    if (status)
    {
        return status;
    }

    /* Enable deep_sleep for motion sensor to reduce power consumption in disconnected state */
    app_motion_sleep(MOTION_DEEP_SLEEP);

    /* Create timer for motion data read trigger */
    motion_data_read_timer = xTimerCreate("Motion Data Read Timer",
                                          pdMS_TO_TICKS(MOTION_DATA_READ_INTERVAL),
                                          pdFALSE,
                                          NULL,
                                          app_motion_data_fetch_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL == motion_data_read_timer)
    {
        printf("Motion data read Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    return status;
}
