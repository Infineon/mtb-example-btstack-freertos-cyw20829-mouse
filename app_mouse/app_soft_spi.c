/*******************************************************************************
 * File Name: app_soft_spi.c
 *
 * Description: This file consists of the function definitions that are
 *              necessary for developing software SPI use cases.
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
#include "app_soft_spi.h"
#include "app_handler.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
static unsigned long soft_spi_delay;
static cyhal_gpio_t soft_spi_sdio, soft_spi_sclk, soft_spi_ss;

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
/**
 *  Function name:
 *  app_soft_spi_write
 *
 *  Function Description:
 *  @brief    Function used for software SPI write
 *
 *  @param    data: Data byte to write
 *
 *  @return   void
 */
void app_soft_spi_write(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        /* Reset the sdio line to low state */
        cyhal_gpio_write(soft_spi_sdio, 0);

        /* Sends clock pulse (CHPOL = 0) */
        cyhal_gpio_write(soft_spi_sclk, 0);
        cyhal_system_delay_us(soft_spi_delay);

        /* Check the most significant bit and put it in */
        if ((data << i) & 0x80)
            cyhal_gpio_write(soft_spi_sdio, 1);
        else
            cyhal_gpio_write(soft_spi_sdio, 0);

        /* Sends clock pulse (CHPOL = 0) */
        cyhal_gpio_write(soft_spi_sclk, 1);
        cyhal_system_delay_us(soft_spi_delay);
    }

    /* Reset clock and data lines to low state */
    cyhal_gpio_write(soft_spi_sclk, 0);
    cyhal_gpio_write(soft_spi_sdio, 0);
}

/**
 *  Function name:
 *  app_soft_spi_readwrite
 *
 *  Function Description:
 *  @brief    Function used for software SPI write and read data
 *
 *  @param    data: Data byte to write
 *
 *  @return   uint8_t
 */
uint8_t app_soft_spi_readwrite(uint8_t data)
{
    uint8_t receive_data = 0;

    for (int i = 0; i < 8; i++)
    {
        /* Left shift the bits read so far */
        receive_data = receive_data << 1;

        /* Reset the sdio line to low state */
        cyhal_gpio_write(soft_spi_sdio, 0);

        /* Sends clock pulse (CHPOL = 0) */
        cyhal_gpio_write(soft_spi_sclk, 0);
        cyhal_system_delay_us(soft_spi_delay);

        /* Check the most significant bit and put it in */
        if ((data << i) & 0x80)
            cyhal_gpio_write(soft_spi_sdio, 1);
        else
            cyhal_gpio_write(soft_spi_sdio, 0);

        /* Read the bit received in MISO */
        if (cyhal_gpio_read(soft_spi_sdio))
            receive_data |= 1;

        /* Sends clock pulse (CHPOL = 0) */
        cyhal_gpio_write(soft_spi_sclk, 1);
        cyhal_system_delay_us(soft_spi_delay);
    }

    /* Reset clock and data lines to low state */
    cyhal_gpio_write(soft_spi_sclk, 0);
    cyhal_gpio_write(soft_spi_sdio, 0);

    return receive_data;
}

/**
 *  Function name:
 *  app_soft_spi_transfer
 *
 *  Function Description:
 *  @brief    Function used for software SPI data transfer
 *
 *  @param    tx: Pointer to write data
 *  @param    tx_length: Length of write data
 *  @param    rx: Pointer to read data
 *  @param    rx_length: Length of read data
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_soft_spi_transfer(const uint8_t *tx, size_t tx_length, uint8_t *rx, size_t rx_length)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Slave select enable */
    cyhal_gpio_write(soft_spi_ss, 0);

    /* Transfer data byte by byte */
    for (uint16_t i = 0; i < tx_length; i++)
    {
        rx[i] = app_soft_spi_readwrite(tx[i]);
    }

    /* Slave select disable */
    cyhal_gpio_write(soft_spi_ss, 1);

    return status;
}

/**
 *  Function name:
 *  app_soft_spi_read
 *
 *  Function Description:
 *  @brief    Function used for software SPI data read
 *
 *  @param    rx: Pointer to read data
 *  @param    rx_length: Length of data to read
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_soft_spi_read(uint8_t *rx, size_t rx_length)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Slave select enable */
    cyhal_gpio_write(soft_spi_ss, 0);

    /* Read data byte by byte */
    for (uint16_t i = 0; i < rx_length; i++)
    {
        rx[i] = app_soft_spi_readwrite(0xff);
    }

    /* Slave select disable */
    cyhal_gpio_write(soft_spi_ss, 1);

    return status;
}

/**
 *  Function name:
 *  app_soft_spi_init
 *
 *  Function Description:
 *  @brief    Function used to initialize software SPI
 *
 *  @param    sdio: GPIO used for data input output
 *  @param    sclk: GPIO used for Clock generation
 *  @param    ss: GPIO used as Slave Select
 *  @param    freq_hz: SPI Frequency
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_soft_spi_init(cyhal_gpio_t sdio, cyhal_gpio_t sclk, cyhal_gpio_t ss, uint32_t freq_hz)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Save pins for later use */
    soft_spi_sdio = sdio;
    soft_spi_sclk = sclk;
    soft_spi_ss = ss;

    /* Initialize the SDIO pin */
    status = cyhal_gpio_init(sdio, CYHAL_GPIO_DIR_BIDIRECTIONAL, CYHAL_GPIO_DRIVE_PULLUPDOWN, 0);

    if (CY_RSLT_SUCCESS != status)
    {
        printf("SDIO Pin Init failed\r\n");
        return status;
    }

    /* Initialize the CLK pin */
    status = cyhal_gpio_init(sclk, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);

    if (CY_RSLT_SUCCESS != status)
    {
        printf("CLK Pin Init failed\r\n");
        return status;
    }

    /* Initialize the NCS pin */
    status = cyhal_gpio_init(ss, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);

    if (CY_RSLT_SUCCESS != status)
    {
        printf("NCS Pin Init failed\r\n");
    }

    /* Max frequency check and limit */
    if (freq_hz >= MAX_SOFT_SPI_FREQ)
    {
        soft_spi_delay = 1;
    }
    else
    {
        soft_spi_delay = (1000000UL / (2 * freq_hz));
    }

    return status;
}
