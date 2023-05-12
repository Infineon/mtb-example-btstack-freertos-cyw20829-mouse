/*******************************************************************************
 * File Name: app_soft_spi.h
 *
 * Description: This file consists of the function prototypes that are
 *              necessary for developing software SPI use cases.
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

#ifndef __APP_SOFT_SPI_H_
#define __APP_SOFT_SPI_H_

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "cycfg_pins.h"
#include "cybsp_types.h"
#include "cyhal_gpio.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
#define MAX_SOFT_SPI_FREQ   500000 // 500 Khz

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
cy_rslt_t app_soft_spi_init(cyhal_gpio_t sdio, cyhal_gpio_t sclk, cyhal_gpio_t ss, uint32_t freq_hz);
cy_rslt_t app_soft_spi_transfer(const uint8_t *tx, size_t tx_length, uint8_t *rx, size_t rx_length);
cy_rslt_t app_soft_spi_read(uint8_t *rx, size_t rx_length);

#endif // __APP_SOFT_SPI_H_
