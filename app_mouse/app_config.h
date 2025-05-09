/******************************************************************************
 * File Name:   app_config.h
 *
 * Description: This file consists of the configurable macros that are used to
 *              change the settings of Mouse.
 *
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
 *                              DEFINES
 *******************************************************************************/

/************************* LED CONFIGURATIONS **********************************/
/**
 * @brief Configure the TURN ON time of LED during startup
 */
#define POWER_UP_LED_ON_TIME_MS                     ((uint32_t)(2000))

/**
 * @brief Configure the LED Blink rate for DPI mode change and
 * just before entering Hibernation
 */
#define LED_BLINK_RATE_MS                           ((uint32_t)(500))

/**
 * @brief Configure the LED Blink rate for connectable advertisements
 */
#define RECONNECTION_ADV_LED_BLINK_TIME_MS          ((uint32_t)(500))

/**
 * @brief Configure the LED Blink rate for undirected advertisements in pairing mode
 */
#define PAIRING_MODE_ADV_LED_BLINK_TIME_MS          ((uint32_t)(100))

/**
 * @brief Configure the LED ON time during low battery
 */
#define LOW_BATT_LED_BLINK_ON_TIME_MS               ((uint32_t)(500))

/**
 * @brief Configure the LED OFF time during low battery
 */
#define LOW_BATT_LED_BLINK_OFF_TIME_MS              ((uint32_t)(3000))

/**
 * @brief Configure the time to allow LED blinking just before
 * entering Hibernation
 */
#define SHUTDOWN_LED_BLINK_INDICATION_MS            ((uint32_t)(2000))
/*******************************************************************************/

/*************** ADVERTISEMENT AND CONNECTION CONFIGURATIONS *******************/
/**
 * @brief Connectable advertisement timeout. This is used during
 * reconnection attempt during power ON or manual reconnection.
 */
#define RECONNECTION_ADV_TIMEOUT_MS                     ((uint32_t)(5000))

/**
 * @brief Configure the idle disconnection timeout to conserve
 * power.
 */
#define DISCONNECTION_TIMEOUT_MS                    ((uint32_t)(600000))
/*******************************************************************************/

/*************** BATTERY VOLTAGE MEASUREMENT CONFIGURATIONS ********************/
/**
 * @brief Voltage in millivolts corresponding to 1%
 */
#define BATT_LVL_1_MV                               ((uint16_t)(1050))

/**
 * @brief Voltage in millivolts corresponding to 10%
 */
#define BATT_LVL_10_MV                              ((uint16_t)(1150))

/**
 * @brief Voltage in millivolts corresponding to 100%
 */
#define BATT_LVL_100_MV                             ((uint16_t)(1500))

/**
 * @brief Battery percentage to start indicating Low battery
 */
#define LOW_BATT_VOLTAGE_PERCENT                    ((uint8_t)(10))

/**
 * @brief Battery read delay during Power ON. ADC measurement will
 * start after this timeout.
 */
#define BATT_LVL_INIT_READ_DELAY_MS                 ((uint32_t)(4000))

/**
 * @brief Configure the frequency of battery voltage measurement
 */
#define BATT_LVL_UPDATE_INTERVAL_MS                 ((uint32_t)(1800000))
/*******************************************************************************/

/*********************** USER BUTTON CONFIGURATIONS ****************************/
/**
 * @brief Configure the debounce timeout in milliseconds for User buttons
 */
#define BUTTON_DEBOUNCE_DELAY_MS                    ((uint8_t)(10))

/**
 * @brief Configure external / internal pullup resistors for User buttons
 * Set true to use external pullups, set false to use internal pullups if present
 */
#define BUTTONS_EXTERNAL_PULLUP_ENABLED             (true)
/*******************************************************************************/

/********************** MOTION SENSOR CONFIGURATIONS ***************************/
/**
 * @brief Configure HW / SW SPI for Motion sensor communication
 * Set true to use HW SPI, set false to use SW SPI
 */
#define HW_SPI_ENABLED                              (true)

/*******************************************************************************/

/************** SCROLL WHEEL / QUADRATURE ENCODER CONFIGURATIONS ***************/
/**
 * @brief Configure external / internal pullup resistors Quadrature Encoder lines
 * Set true to use external pullups, set false to use internal pullups
 */
#define QUAD_ENCODER_EXT_CONTROL_ENABLED            (true)

/**
 * @brief Configure wake up on scroll event in disconnected state
 * Set true to enable wake up, set false to disable wake up
 */
#define WAKEUP_ON_SCROLL_ENABLED                    (false)
/*******************************************************************************/

/******************** MULTI-DEVICE FEATURE CONFIGURATIONS **********************/
/**
 * @brief To enable multi-device feature support, Refer README for feature details
 * Set true to enable multi-device feature, set false to disable multi-device feature
 */
#define MULTI_DEVICE_FEATURE_ENABLE                 (true)

/**
 * @brief Configure the duration of multi-button press in milliseconds for
 * switching to the next device channel
 */
#define DEVICE_SWITCH_DELAY_MS                ((uint32_t)(100))
/*******************************************************************************/

/******************** PAIRING MODE SWITCH CONFIGURATIONS ***********************/
/**
 * @brief To enable pairing mode switch using DPI button
 * Set true to enable pairing mode switch, set false to disable pairing mode switch
 */
#define PAIRING_MODE_SWITCH_ENABLE                  (true)

/**
 * @brief Configure the duration of pairing button press in milliseconds for
 * switching the pairing mode in current device channel
 */
#define PAIRING_MODE_SWITCH_DELAY_MS                ((uint32_t)(3000))
/*******************************************************************************/


/************************** UART LOG CONFIGURATIONS ****************************/
/**
 * @brief To enable the UART logs
 * Disabled by default , set this to true to enable UART logs
 * MTB debugging wont work when UART logging enabled, since same pins used
 */
#define ENABLE_LOGGING                              (false)
/*******************************************************************************/

/* [] END OF FILE */
