/*******************************************************************************
 * File Name: app_paw3212.c
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
#include "app_paw3212.h"
#include "app_handler.h"
#include "app_soft_spi.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
#if (HW_SPI_ENABLED == true) && (ENABLE_LOGGING == false)
extern cyhal_spi_t mSPI;
#endif

/* SPI TX and RX buffers */
#if CONFIG_PAW3212_12_BIT_MODE
uint8_t motion_rx_data[8];
uint8_t motion_tx_data[8] = { PAW3212_REG_MOTION, 0xff,
                              PAW3212_REG_DELTA_X_LOW, 0xff,
                              PAW3212_REG_DELTA_Y_LOW, 0xff,
                              PAW3212_REG_DELTA_XY_HIGH, 0xff };
#else
uint8_t motion_rx_data[6];
uint8_t motion_tx_data[6] = { PAW3212_REG_MOTION, 0xff,
                              PAW3212_REG_DELTA_X_LOW, 0xff,
                              PAW3212_REG_DELTA_Y_LOW, 0xff };
#endif

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
/**
 *  Function name:
 *  app_paw3212_motion_data_handle
 *
 *  Function Description:
 *  @brief    Function to Process the motion data read from the PAW3212 sensor
 *
 *  @param    motion_data: Pointer to fill in the motion data read by sensor
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_motion_data_handle(motion_sensor_data_t *motion_data)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint8_t xy_motion_status;
    int8_t x_low;
    int8_t y_low;
#if CONFIG_PAW3212_12_BIT_MODE
    int8_t xy_high;
#endif

    /* Motion status */
    xy_motion_status = motion_rx_data[1];

    /* Check if motion data is present */
    if ((xy_motion_status & PAW3212_MOTION_STATUS_MOTION) != 0)
    {
        motion_data->motion_status = 1;
#if CONFIG_PAW3212_12_BIT_MODE
        xy_high = motion_rx_data[7];
#endif
        /* Ignore X motion data if overflow detected */
        if ((xy_motion_status & PAW3212_MOTION_STATUS_DXOVF) != 0)
        {
            printf("X delta overflowed\r\n");
            motion_data->delta_x = 0;
        }

        /* Motion delta in X direction */
        x_low = motion_rx_data[3];
#if CONFIG_PAW3212_12_BIT_MODE
        // Calculating the 12 bit delta X motion data
        motion_data->delta_x = PAW3212_DELTA_X(xy_high, x_low);
#else
        motion_data->delta_x = (int8_t)x_low;
#endif

        /* Ignore Y motion data if overflow detected */
        if ((xy_motion_status & PAW3212_MOTION_STATUS_DYOVF) != 0)
        {
            printf("Y delta overflowed\r\n");
            motion_data->delta_y = 0;
        }

        /* Motion delta in Y direction */
        y_low = motion_rx_data[5];
#if CONFIG_PAW3212_12_BIT_MODE
        // Calculating the 12 bit delta Y motion data
        motion_data->delta_y = PAW3212_DELTA_Y(xy_high, y_low);
#else
        motion_data->delta_y = (int8_t)y_low;
#endif
    }
    else
    {
        /* Clear the motion data reports if motion data is not present */
        motion_data->delta_x = 0;
        motion_data->delta_y = 0;
        motion_data->motion_status = 0;

        status = EINVAL;
    }

    return status;
}

/**
 *  Function name:
 *  app_paw3212_cpi_val
 *
 *  Function Description:
 *  @brief    Function to get the CPI value to be set corresponding to the mode selected
 *
 *  @param    cpi_mode: CPI mode selected
 *
 *  @return   uint16_t: CPI value to be set in motion sensor
 */
uint16_t app_paw3212_cpi_val(uint8_t cpi_mode)
{
    uint16_t cpi_val = 0;

    switch (cpi_mode)
    {
        case MOTION_CPI_836:
            cpi_val = PAW3212_CPI_836;
            break;
        case MOTION_CPI_1026:
            cpi_val = PAW3212_CPI_1026;
            break;
        case MOTION_CPI_1634:
            cpi_val = PAW3212_CPI_1634;
            break;
        default:
            cpi_val = PAW3212_CPI_1026;
            break;
    }

    return cpi_val;
}

/**
 *  Function name:
 *  app_paw3212_power_down
 *
 *  Function Description:
 *  @brief    Function to Power down the PAW3212 sensor
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_power_down(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint8_t config = PAW3212_DEFAULT_CONFIGURATION;

    status = app_paw3212_reg_write(PAW3212_REG_CONFIGURATION,
                               (config | PAW3212_CONFIG_CHIP_RESET));
    /* Power down configuration */
    status = app_paw3212_reg_write(PAW3212_REG_CONFIGURATION,
                               (config | PAW3212_CONFIG_PD_ENABLE));
    if (status)
    {
        printf("Cannot power down the sensor \r\n");
    }

    return status;
}

/**
 *  Function name:
 *  app_paw3212_deep_sleep
 *
 *  Function Description:
 *  @brief    Function to Configure the sleep mode of motion sensor
 *   for lower power consumption by reducing scan up interval and response rate
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_deep_sleep(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint8_t slp2_freq = 14; // 960ms(8 * 64ms)
    uint8_t config = PAW3212_DEFAULT_CONFIGURATION;

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);
    if (status)
    {
        printf("Cannot disable write protect\r\n");
        return status;
    }

    status = app_paw3212_reg_write(PAW3212_REG_SLEEP2, (slp2_freq << 4));
    if (status)
    {
        printf("Failed to change Sleep 2 Frequency\r\n");
    }

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, 0);
    if (status)
    {
        printf("Cannot enable write protect\r\n");
    }

    /* Disable Sleep 3 mode */
    status = app_paw3212_reg_write(PAW3212_REG_CONFIGURATION, config);
    /* Force Sleep 2 mode */
    app_paw3212_reg_write(PAW3212_REG_OPERATION_MODE, PAW3212_FORCE_SLEEP2);

    return status;
}

/**
 *  Function name:
 *  app_paw3212_light_sleep
 *
 *  Function Description:
 *  @brief    Function to Configure the sleep mode of motion sensor
 *   for better response with moderate sleep
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_light_sleep(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint8_t slp2_freq = 1; // 128ms(2 * 64ms)
    uint8_t config = PAW3212_DEFAULT_CONFIGURATION;

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);
    if (status)
    {
        printf("Cannot disable write protect\r\n");
        return status;
    }

    status = app_paw3212_reg_write(PAW3212_REG_SLEEP2, (slp2_freq << 4));
    if (status)
    {
        printf("Failed to change Sleep 2 Frequency\r\n");
    }

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, 0);
    if (status)
    {
        printf("Cannot enable write protect\r\n");
    }

    /* Wakeup sensor from sleep */
    status = app_paw3212_reg_write(PAW3212_REG_OPERATION_MODE, PAW3212_WAKEUP);
    /* Enable Sleep 3 mode */
    status = app_paw3212_reg_write(PAW3212_REG_CONFIGURATION, (config | PAW3212_CONFIG_SLP3_ENABLE));

    return status;
}

#if CONFIG_PAW3212_12_BIT_MODE
/**
 *  Function name:
 *  app_expand_s12
 *
 *  Function Description:
 *  @brief    Function to Convert 16 bit signed value to 12 bit signed integer
 *
 *  @param    x: 16 bit value to be converted
 *
 *  @return   int16_t: 12 bit signed integer
 */
int16_t app_expand_s12(int16_t x)
{
    /* Left shifting of negative values is undefined behavior, so we cannot
     * depend on automatic integer promotion (it will convert int16_t to int).
     * To expand sign we cast s16 to unsigned int and left shift it then
     * cast it to signed integer and do the right shift. Since type is
     * signed compiler will perform arithmetic shift. This last operation
     * is implementation defined in C but defined in the compiler's
     * documentation and common for modern compilers and CPUs.
     */
    return ((signed int)((unsigned int)x << 20)) >> 20;
}
#endif

/**
 *  Function name:
 *  app_paw3212_reg_read
 *
 *  Function Description:
 *  @brief    Function to Read data from PAW3212 sensor
 *
 *  @param    reg: Register address to read
 *  @param    value: Pointer to store data
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_reg_read(uint8_t reg, uint8_t *value)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint8_t receive_data[2];
    uint8_t transmit_data[2] = { reg, 0xff };

    /* Send the command byte to the slave. */
#if (HW_SPI_ENABLED == true) && (ENABLE_LOGGING == false)
    status = cyhal_spi_transfer(&mSPI, transmit_data, sizeof(transmit_data), receive_data,
                                sizeof(receive_data), 0xff);
#else
    transmit_data[1] = 0xff;
    status = app_soft_spi_transfer(transmit_data, sizeof(transmit_data),
                              receive_data, sizeof(receive_data));
#endif

    if (CY_RSLT_SUCCESS == status)
    {
        printf("Reg read failed on SPI write\r\n");
        *value = receive_data[1];
    }

    return status;
}

/**
 *  Function name:
 *  app_paw3212_reg_write
 *
 *  Function Description:
 *  @brief    Function to Write data to PAW3212 sensor
 *
 *  @param    reg: Register address to write
 *  @param    val: Data to be written
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_reg_write(uint8_t reg, uint8_t val)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint8_t transmit_data[2] = { SPI_WRITE_BIT | reg, val };
    uint8_t receive_data[2];

    /* Send the command byte to the slave. */
#if (HW_SPI_ENABLED == true) && (ENABLE_LOGGING == false)
    status = cyhal_spi_transfer(&mSPI, transmit_data, sizeof(transmit_data), receive_data,
                                sizeof(receive_data), 0xff);
#else
    status = app_soft_spi_transfer(transmit_data, sizeof(transmit_data),
                              receive_data, sizeof(receive_data));
#endif

    if (status)
    {
        printf("Reg write failed on SPI write\r\n");
    }

    return status;
}

/**
 *  Function name:
 *  app_paw3212_update_cpi
 *
 *  Function Description:
 *  @brief    Function to Update CPI setting of PAW3212 sensor
 *
 *  @param    cpi: CPI value to be written
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_update_cpi(uint16_t cpi)
{
    cy_rslt_t status;

    if (cpi > PAW3212_CPI_MAX)
    {
        printf("CPI %u out of range\r\n", cpi);
        return EINVAL;
    }

    uint8_t regval = (cpi / PAW3212_CPI_STEP);

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);
    if (status)
    {
        printf("Cannot disable write protect\r\n");
        return status;
    }

    status = app_paw3212_reg_write(PAW3212_REG_CPI_X, regval);
    if (status)
    {
        printf("Failed to change x CPI\r\n");
    }

    status = app_paw3212_reg_write(PAW3212_REG_CPI_Y, regval);
    if (status)
    {
        printf("Failed to change y CPI\r\n");
    }

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, 0);
    if (status)
    {
        printf("Cannot enable write protect\r\n");
    }

    return status;
}

/**
 *  Function name:
 *  app_paw3212_update_sleep_timeout
 *
 *  Function Description:
 *  @brief    Function tot Update sleep timeout settings of PAW3212 sensor
 *
 *  @param    reg_addr: Register address of the sleep mode
 *  @param    timeout_ms: Timeout value in ms
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_update_sleep_timeout(uint8_t reg_addr, uint32_t timeout_ms)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint32_t timeout_step_ms;

    switch (reg_addr)
    {
        case PAW3212_REG_SLEEP1:
            timeout_step_ms = 32;
            break;

        case PAW3212_REG_SLEEP2:
        case PAW3212_REG_SLEEP3:
            timeout_step_ms = 20480;
            break;

        default:
            printf("Not supported\r\n");
            return ENOTSUP;
    }

    uint8_t etm = ((timeout_ms / timeout_step_ms) - 1);

    if (etm > PAW3212_ETM_MAX)
    {
        printf("Sleep timeout %lu out of range\r\n", timeout_ms);
        return EINVAL;
    }

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);
    if (status)
    {
        printf("Cannot disable write protect\r\n");
        return status;
    }

    uint8_t regval;

    status = app_paw3212_reg_read(reg_addr, &regval);

    if (status == CY_RSLT_SUCCESS)
    {
        regval &= ~PAW3212_ETM_MASK;
        regval |= (etm << PAW3212_ETM_POS);

        status = app_paw3212_reg_write(reg_addr, regval);
        if (status)
        {
            printf("Failed to change sleep time\r\n");
        }
    }

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, 0);

    if (status)
    {
        printf("Cannot enable write protect\r\n");
    }

    return status;
}

/**
 *  Function name:
 *  app_paw3212_update_sample_time
 *
 *  Function Description:
 *  @brief    Function to Update sampling interval settings of PAW3212 sensor
 *
 *  @param    reg_addr: Register address of the sleep mode
 *  @param    sample_time_ms: Sampling Time value in ms
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_update_sample_time(uint8_t reg_addr, uint16_t sample_time_ms)
{
    uint16_t sample_time_step;
    uint16_t sample_time_min;
    uint16_t sample_time_max;

    switch (reg_addr)
    {
        case PAW3212_REG_SLEEP1:
            sample_time_step = 4;
            sample_time_min = 4;
            sample_time_max = 64;
            break;

        case PAW3212_REG_SLEEP2:
        case PAW3212_REG_SLEEP3:
            sample_time_step = 64;
            sample_time_min = 64;
            sample_time_max = 1024;
            break;

        default:
            printf("Not supported\r\n");
            return ENOTSUP;
    }

    if ((sample_time_ms > sample_time_max) || (sample_time_ms < sample_time_min))
    {
        printf("Sample time %u out of range\r\n", sample_time_ms);
        return EINVAL;
    }

    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint8_t reg_freq = (sample_time_ms - sample_time_min) / sample_time_step;

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);
    if (status)
    {
        printf("Cannot disable write protect\r\n");
        return status;
    }

    uint8_t regval;

    status = app_paw3212_reg_read(reg_addr, &regval);
    if (status == CY_RSLT_SUCCESS)
    {
        regval &= ~PAW3212_FREQ_MASK;
        regval |= (reg_freq << PAW3212_FREQ_POS);

        status = app_paw3212_reg_write(reg_addr, regval);
        if (status)
        {
            printf("Failed to change sample time\r\n");
        }
    }

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, 0);
    if (status)
    {
        printf("Cannot enable write protect\r\n");
    }

    return status;
}

/**
 *  Function name:
 *  app_paw3212_config_sleep_modes
 *
 *  Function Description:
 *  @brief    Function to Update Sleep mode settings of PAW3212 sensor
 *
 *  @param    reg_addr1: Register address of the sleep mode
 *  @param    reg_addr2: Register address of the sleep mode
 *  @param    enable: Enable or disable the sleep mode
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_config_sleep_modes(uint8_t reg_addr1, uint8_t reg_addr2, bool enable)
{
    cy_rslt_t status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);

    if (status)
    {
        printf("Cannot disable write protect\r\n");
        return status;
    }

    uint8_t regval;

    printf("%sable sleep\r\n", (enable) ? ("En") : ("Dis"));

    /* Sleep 1 and Sleep 2 */
    status = app_paw3212_reg_read(reg_addr1, &regval);
    if (status)
    {
        printf("Failed to read operation mode register\r\n");
        return status;
    }

    uint8_t sleep_enable_mask = BIT(PAW3212_SLP_ENH_POS) | BIT(PAW3212_SLP2_ENH_POS);

    if (enable)
    {
        regval |= sleep_enable_mask;
    }
    else
    {
        regval &= ~sleep_enable_mask;
    }

    status = app_paw3212_reg_write(reg_addr1, regval);
    if (status)
    {
        printf("Failed to %sable sleep\r\n", (enable) ? ("en") : ("dis"));
        return status;
    }

    /* Sleep 3 */
    status = app_paw3212_reg_read(reg_addr2, &regval);
    if (status)
    {
        printf("Failed to read configuration register\r\n");
        return status;
    }

    sleep_enable_mask = BIT(PAW3212_SLP3_ENH_POS);

    if (enable)
    {
        regval |= sleep_enable_mask;
    }
    else
    {
        regval &= ~sleep_enable_mask;
    }

    status = app_paw3212_reg_write(reg_addr2, regval);
    if (status)
    {
        printf("Failed to %sable sleep\r\n", (enable) ? ("en") : ("dis"));
        return status;
    }

    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, 0);
    if (status)
    {
        printf("Cannot enable write protect\r\n");
    }

    return status;
}

/**
 *  Function name:
 *  app_paw3212_update_config
 *
 *  Function Description:
 *  @brief    Function to Update settings of PAW3212 sensor
 *
 *  @param    config: Configuration to update
 *  @param    val: Value to update
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_update_config(enum paw3212_config_e config, uint32_t val)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    switch (config)
    {
        case PAW3212_CONFIG_CPI:
            status = app_paw3212_update_cpi(val);
            break;

        case PAW3212_CONFIG_SLEEP_EN:
            status = app_paw3212_config_sleep_modes(PAW3212_REG_OPERATION_MODE,
                                                PAW3212_REG_CONFIGURATION,
                                                val);
            break;

        case PAW3212_CONFIG_SLEEP1_TIMEOUT:
            status = app_paw3212_update_sleep_timeout(PAW3212_REG_SLEEP1, val);
            break;

        case PAW3212_CONFIG_SLEEP2_TIMEOUT:
            status = app_paw3212_update_sleep_timeout(PAW3212_REG_SLEEP2, val);
            break;

        case PAW3212_CONFIG_SLEEP3_TIMEOUT:
            status = app_paw3212_update_sleep_timeout(PAW3212_REG_SLEEP3, val);
            break;

        case PAW3212_CONFIG_SLEEP1_SAMPLE_TIME:
            status = app_paw3212_update_sample_time(PAW3212_REG_SLEEP1, val);
            break;

        case PAW3212_CONFIG_SLEEP2_SAMPLE_TIME:
            status = app_paw3212_update_sample_time(PAW3212_REG_SLEEP2, val);
            break;

        case PAW3212_CONFIG_SLEEP3_SAMPLE_TIME:
            status = app_paw3212_update_sample_time(PAW3212_REG_SLEEP3, val);
            break;

        default:
            printf("Unknown attribute");
            return ENOTSUP;
    }

    return status;
}

/**
 *  Function name:
 *  app_paw3212_sample_fetch_3_wire
 *
 *  Function Description:
 *  @brief    Function to Fetch the motion data from PAW3212 sensor
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_sample_fetch_3_wire(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

#if (HW_SPI_ENABLED == true) && (ENABLE_LOGGING == false)
    /* Send the command byte to the slave. */
    status = cyhal_spi_transfer(&mSPI,
                                motion_tx_data, sizeof(motion_tx_data),
                                motion_rx_data, sizeof(motion_rx_data),
                                0xff);
#else
    status = app_soft_spi_transfer(motion_tx_data, sizeof(motion_tx_data),
                              motion_rx_data, sizeof(motion_rx_data));
#endif

    return status;
}

/**
 *  Function name:
 *  app_paw3212_mode_configure
 *
 *  Function Description:
 *  @brief    Initialization and Configurations for PAW3212 sensor
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_mode_configure(void)
{
    cy_rslt_t status;

    /* Disable write protection */
    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, PAW3212_WPMAGIC);

    if (!status)
    {
        uint8_t mouse_option = 0;
        uint8_t slp3_freq = 3; // 256ms(4 * 64ms)
        uint8_t slp3_etm = 0; // 20.48s

#if CONFIG_PAW3212_ORIENTATION_0
        mouse_option |= PAW3212_MOUSE_OPT_Y_INV;
#endif
#if CONFIG_PAW3212_ORIENTATION_90
        mouse_option |= PAW3212_MOUSE_OPT_XY_SWAP;
#endif
#if CONFIG_PAW3212_ORIENTATION_180
        mouse_option |= PAW3212_MOUSE_OPT_Y_INV |
                        PAW3212_MOUSE_OPT_X_INV;
#endif
#if CONFIG_PAW3212_ORIENTATION_270
        mouse_option |= PAW3212_MOUSE_OPT_XY_SWAP |
                        PAW3212_MOUSE_OPT_X_INV;
#endif
#if CONFIG_PAW3212_12_BIT_MODE
        mouse_option |= PAW3212_MOUSE_OPT_12BITMODE;
#endif
        /* Configure mouse sensor orientation */
        app_paw3212_reg_write(PAW3212_REG_MOUSE_OPTION, mouse_option);
        /* Configure sleep mode */
        app_paw3212_reg_write(PAW3212_REG_SLEEP3, ((slp3_freq << 4) | slp3_etm));
    }

    /* Enable write protection */
    status = app_paw3212_reg_write(PAW3212_REG_WRITE_PROTECT, 0);

    return status;
}

/**
 *  Function name:
 *  app_paw3212_verify_id
 *
 *  Function Description:
 *  @brief    Function to verify Product ID of PAW3212 sensor
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_verify_id(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint8_t product_id;

    status = app_paw3212_reg_read(PAW3212_REG_PRODUCT_ID, &product_id);
    if (status)
    {
        printf("Cannot obtain product ID");
        return status;
    }

    printf("Product ID: 0x%x", product_id);

    if (product_id != PAW3212_PRODUCT_ID)
    {
        printf("Invalid product ID (0x%x)!", product_id);
        return EINVAL;
    }

    return status;
}

/**
 *  Function name:
 *  app_paw3212_power_up_reset
 *
 *  Function Description:
 *  @brief    Power up initialization sequence for PAW3212 sensor
 *
 *  @param    void
 *
 *  @return   cy_rslt_t: CY_RSLT_SUCCESS if success, else an error code
 */
cy_rslt_t app_paw3212_power_up_reset(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    uint8_t config = 0x11;

    status = app_paw3212_reg_write(PAW3212_REG_CONFIGURATION,
                              (config | PAW3212_CONFIG_CHIP_RESET | PAW3212_CONFIG_SLP3_ENABLE));

    if (status)
    {
        printf("Cannot Reset chip\r\n");
        return status;
    }

    return status;
}
