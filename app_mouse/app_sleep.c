/*******************************************************************************
 * File Name: app_sleep.c
 *
 * Description: This file consists of the function definitions that are
 *              necessary for handling device sleep
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
#include "FreeRTOS.h"
#include "task.h"
#include "cyabs_rtos.h"
#if defined(CY_USING_HAL)
#include "cyhal.h"
#endif

#include "app_sleep.h"
#include "app_serial_flash.h"
#include "app_handler.h"
#include "cybsp_smif_init.h"
#include "app_button.h"
#include "app_motion.h"
#include "cy_retarget_io.h"



/*******************************************************************************
 *                              Macro Definitions
 *******************************************************************************/

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
cy_en_syspm_status_t
app_syspm_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
                                 cy_en_syspm_callback_mode_t mode);
cy_en_syspm_status_t
app_syspm_dsram_cb(cy_stc_syspm_callback_params_t *callbackParams,
                              cy_en_syspm_callback_mode_t mode);
cy_en_syspm_status_t
app_syspm_uart_dsram_cb(cy_stc_syspm_callback_params_t *callbackParams,
                              cy_en_syspm_callback_mode_t mode);
static void app_btn_pin_denit();
static void app_quaddec_pin_deint();
static void app_quaddec_pin_deint();
static void app_spi_deint();


/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

cy_stc_syspm_callback_params_t app_syspm_deep_sleep_params;
cy_stc_syspm_callback_params_t app_syspm_dsram_params;
cy_stc_syspm_callback_params_t app_syspm_uart_dsram_params;
extern cyhal_gpio_callback_data_t app_left_button_isr_data;
extern cyhal_gpio_callback_data_t app_right_button_isr_data;
extern cyhal_gpio_callback_data_t app_middle_button_isr_data;
extern cyhal_gpio_callback_data_t app_dpi_button_isr_data;
extern cyhal_gpio_callback_data_t app_quaddec_za_isr_data;
extern cyhal_gpio_callback_data_t app_quaddec_zb_isr_data;
#if (HW_SPI_ENABLED == true) && (ENABLE_LOGGING == false)
extern cyhal_spi_t mSPI;
#endif

cy_stc_syspm_callback_t app_syspm_deep_sleep_cb_handler =
{
    app_syspm_ds_cb,
    CY_SYSPM_DEEPSLEEP,
    0u,
    &app_syspm_deep_sleep_params,
    NULL,
    NULL,
    253
};

cy_stc_syspm_callback_t app_syspm_dsram_cb_handler =
{
        app_syspm_dsram_cb,
    CY_SYSPM_DEEPSLEEP_RAM,
    0u,
    &app_syspm_uart_dsram_params,
    NULL,
    NULL,
    250
};
cy_stc_syspm_callback_t app_syspm_uart_dsram_cb_handler =
{
    app_syspm_uart_dsram_cb,
    CY_SYSPM_DEEPSLEEP_RAM,
    0u,
    &app_syspm_uart_dsram_params,
    NULL,
    NULL,
    0
};

#if (ENABLE_WDT == true) && (ENABLE_LOGGING == false)
/* WDT object */
extern cyhal_wdt_t wdt_obj;
#endif
/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/


/**
 *  Function name:
 *  syspm_ds_cb
 *
 *  Function Description:
 *  @brief DeepSleep Callback Function
 *
 *  @param callbackParams: Pointer to cy_stc_syspm_callback_params_t
 *  @param mode: cy_en_syspm_callback_mode_t
 *
 *  @return cy_en_syspm_status_t: CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t app_syspm_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
                                 cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
#if (ENABLE_WDT == true) && (ENABLE_LOGGING == false)
            /* Stop watchdog timer */
            cyhal_wdt_kick(&wdt_obj);
            cyhal_wdt_stop(&wdt_obj);
#endif
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        /* Performs the actions to be done after exiting the low power mode */
        {
#if (ENABLE_WDT == true) && (ENABLE_LOGGING == false)
            /* Start watchdog timer */
            cyhal_wdt_start(&wdt_obj);
            cyhal_wdt_kick(&wdt_obj);
#endif
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END

/**
 * Function Name:
 * app_syspm_dsram_cb
 *
 * Function Description:
 * @brief DeepSleep Callback Function
 *
 * @param callbackParams Pointer to cy_stc_syspm_callback_params_t
 * @param mode cy_en_syspm_callback_mode_t
 *
 * @return cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
app_syspm_dsram_cb( cy_stc_syspm_callback_params_t *callbackParams,
                cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    switch(mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
            app_scroll_disable();
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        /* Performs the actions to be done after entering the low power mode */
        {
            cyhal_gpio_free(PAW3212_MOTION_PIN);
            /*MOTION SENSOR PIN REINT */
            app_motion_pin_init();
            app_btn_pin_denit();
            app_quaddec_pin_deint();
            app_spi_deint();

            /*BUTTON'S REINT */
            app_button_configure_pin(USER_BTN_LEFT,
                                    CYHAL_GPIO_DRIVE_NONE,
                                     CYHAL_GPIO_IRQ_BOTH,
                                     GPIO_INTERRUPT_PRIORITY,
                                     &app_left_button_isr_data);


            app_button_configure_pin(USER_BTN_RIGHT,
                                     CYHAL_GPIO_DRIVE_NONE,
                                     CYHAL_GPIO_IRQ_BOTH,
                                     GPIO_INTERRUPT_PRIORITY,
                                     &app_right_button_isr_data);



            app_button_configure_pin(USER_BTN_MIDDLE,
                                     CYHAL_GPIO_DRIVE_NONE,
                                     CYHAL_GPIO_IRQ_BOTH,
                                     GPIO_INTERRUPT_PRIORITY,
                                     &app_middle_button_isr_data);

            app_button_configure_pin(USER_BTN_DPI,
                                     CYHAL_GPIO_DRIVE_NONE,
                                     CYHAL_GPIO_IRQ_BOTH,
                                     GPIO_INTERRUPT_PRIORITY,
                                     &app_dpi_button_isr_data);
            app_motion_spi_init();
            app_motion_sleep(MOTION_LIGHT_SLEEP);

            cyhal_gpio_init(ENCODER_ZA,
                                     CYHAL_GPIO_DIR_INPUT,
                                     CYHAL_GPIO_DRIVE_NONE,
                                     1);
            cyhal_gpio_register_callback(ENCODER_ZA, &app_quaddec_za_isr_data);

            cyhal_gpio_init(ENCODER_ZB,
                                     CYHAL_GPIO_DIR_INPUT,
                                     CYHAL_GPIO_DRIVE_NONE,
                                     1);
            cyhal_gpio_register_callback(ENCODER_ZB, &app_quaddec_zb_isr_data);



        #if QUAD_ENCODER_EXT_CONTROL_ENABLED
            cyhal_gpio_init(ENCODER_COM,
                                     CYHAL_GPIO_DIR_OUTPUT,
                                     CYHAL_GPIO_DRIVE_STRONG,
                                     0);

        #endif
            app_scroll_enable();
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END


/**
 * Function Name:
 * app_syspm_uart_dsram_cb
 *
 * Function Description:
 * @brief DeepSleep Callback Function
 *
 * @param callbackParams Pointer to cy_stc_syspm_callback_params_t
 * @param mode cy_en_syspm_callback_mode_t
 *
 * @return cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
app_syspm_uart_dsram_cb( cy_stc_syspm_callback_params_t *callbackParams,
                cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    switch(mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        /* Performs the actions to be done after entering the low power mode */
        {
            if(Cy_SysLib_IsDSRAMWarmBootEntry())
            {
#if (ENABLE_LOGGING == TRUE)
                cy_retarget_io_deinit();
                cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,115200);
#endif
            }

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END

static void app_btn_pin_denit()
{
    cyhal_gpio_free(USER_BTN_LEFT);
    cyhal_gpio_free(USER_BTN_RIGHT);
    cyhal_gpio_free(USER_BTN_MIDDLE);
    cyhal_gpio_free(USER_BTN_DPI);
}

static void app_spi_deint()
{
#if ENABLE_LOGGING
    cyhal_gpio_free(mSPI_MISO);
    cyhal_gpio_free(mSPI_SCLK);
    cyhal_gpio_free(mSPI_SS);
#else
    cyhal_spi_free(&mSPI);
#endif
}

static void app_quaddec_pin_deint()
{
    cyhal_gpio_free(ENCODER_ZA);
    cyhal_gpio_free(ENCODER_ZB);
    cyhal_gpio_free(ENCODER_COM);
}

/**
 * Function Name:
 * app_create_deep_sleep_cb
 *
 * Function Description:
 * @brief Creates a syspm Callback for Deep Sleep mode
 *
 * @param  void
 *
 * @return void
 */
void app_create_deep_sleep_cb(void)
{
    Cy_SysPm_RegisterCallback(&app_syspm_deep_sleep_cb_handler);
}
/**
 * Function Name:
 * app_create_deep_sleep_ram__cb
 *
 * Function Description:
 * @brief Creates a syspm Callback for Deep Sleep mode
 *
 * @param  void
 *
 * @return void
 */
void app_create_deep_sleep_ram__cb(void)
{
    Cy_SysPm_RegisterCallback(&app_syspm_dsram_cb_handler);
}
/**
 * Function Name:
 * app_create_deep_sleep_ram_uart_cb
 *
 * Function Description:
 * @brief Creates a syspm Callback for Deep Sleep mode
 *
 * @param  void
 *
 * @return void
 */
void app_create_deep_sleep_ram_uart_cb(void)
{
    Cy_SysPm_RegisterCallback(&app_syspm_uart_dsram_cb_handler);
}

