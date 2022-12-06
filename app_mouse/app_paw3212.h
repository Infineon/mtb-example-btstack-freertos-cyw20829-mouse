/*******************************************************************************
 * File Name: app_paw3212.h
 *
 * Description: This file consists of the function prototypes that are
 *              necessary for developing the HAL applications
 *
 *******************************************************************************
 * Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef __APP_PAW3212_H_
#define __APP_PAW3212_H_

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/

#include "cybsp_types.h"
#include "app_config.h"
#include "app_motion.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
#define BIT(x) (1UL << (x))
#define BIT_MASK(x) (unsigned int)((1UL << x) - 1)

#define SPI_WRITE_BIT           BIT(7)

/* PAW3212 Product ID value for verification */
#define PAW3212_PRODUCT_ID      0x30

/* Write protect magic */
#define PAW3212_WPMAGIC         0x5A

/* PAW3212 Sensor registers */
#define PAW3212_REG_PRODUCT_ID      0x00 // Read only - Default = 0x30
#define PAW3212_REG_REVISION_ID     0x01 // Read only - Default = 0x02
#define PAW3212_REG_MOTION          0x02 // Read only - Motion Status information
#define PAW3212_REG_DELTA_X_LOW     0x03 // Read only - Eight bits 2’s complement number for X-axis motion data
#define PAW3212_REG_DELTA_Y_LOW     0x04 // Read only - Eight bits 2’s complement number for Y-axis motion data
#define PAW3212_REG_OPERATION_MODE  0x05 // Operation mode selection. Default = 0xB8
#define PAW3212_REG_CONFIGURATION   0x06 // Software power down and reset. Default = 0x11
#define PAW3212_REG_WRITE_PROTECT   0x09 // Write Protect to avoid mis-writing registers. Default = 0x00
#define PAW3212_REG_SLEEP1          0x0A // Sleep1 configuration. Default = 0x77
#define PAW3212_REG_SLEEP2          0x0B // Sleep2 configuration. Default = 0x10
#define PAW3212_REG_SLEEP3          0x0C // Sleep3 configuration. Default = 0x70
#define PAW3212_REG_CPI_X           0x0D // CPI setting for X axis. Default = 0x1B
#define PAW3212_REG_CPI_Y           0x0E // CPI setting for Y axis. Default = 0x1B
#define PAW3212_REG_DELTA_XY_HIGH   0x12 // Read only - Upper 4 bits of Delta_X and Delta_Y for 12-bit data format
#define PAW3212_REG_IQC             0x13 // Read only - Image Quality Complement
#define PAW3212_REG_SHUTTER         0x14 // Read only - Index of LED shutter time
#define PAW3212_REG_FRAME_AVG       0x17 // Read only - Average brightness of a frame
#define PAW3212_REG_MOUSE_OPTION    0x19 // Mouse orientation selection. Default = 0x00
#define PAW3212_REG_SPI_MODE        0x26 // 3-wired or 2-wired SPI interface. Default = 0xB4

/* Default sensor register values */
#define PAW3212_DEFAULT_CONFIGURATION   0x11

/* CPI Settings */
#define PAW3212_CPI_STEP        38u
#define PAW3212_CPI_MIN         (0x00 * PAW3212_CPI_STEP)
#define PAW3212_CPI_MAX         (0x3F * PAW3212_CPI_STEP)
#define PAW3212_CPI_836         (0x16 * PAW3212_CPI_STEP)
#define PAW3212_CPI_1026        (0x1B * PAW3212_CPI_STEP) // Default
#define PAW3212_CPI_1634        (0x2B * PAW3212_CPI_STEP)

/* Sleep Mode Settings */
#define PAW3212_SLP_ENH_POS     4u
#define PAW3212_SLP2_ENH_POS    3u
#define PAW3212_SLP3_ENH_POS    5u

#define PAW3212_ETM_POS         0u
#define PAW3212_ETM_SIZE        4u
#define PAW3212_FREQ_POS        (PAW3212_ETM_POS + PAW3212_ETM_SIZE)
#define PAW3212_FREQ_SIZE       4u

#define PAW3212_ETM_MIN         0u
#define PAW3212_ETM_MAX         BIT_MASK(PAW3212_ETM_SIZE)
#define PAW3212_ETM_MASK        (PAW3212_ETM_MAX << PAW3212_ETM_POS)

#define PAW3212_FREQ_MIN        0u
#define PAW3212_FREQ_MAX        BIT_MASK(PAW3212_FREQ_SIZE)
#define PAW3212_FREQ_MASK       (PAW3212_FREQ_MAX << PAW3212_FREQ_POS)

#define PAW3212_FORCE_SLEEP1    (BIT(7) | BIT(5) | BIT(4) | BIT(3) | BIT(1))
#define PAW3212_FORCE_SLEEP2    (BIT(7) | BIT(5) | BIT(4) | BIT(3) | BIT(2))
#define PAW3212_WAKEUP          (BIT(7) | BIT(5) | BIT(4) | BIT(3) | BIT(0))

/* Motion status bits */
#define PAW3212_MOTION_STATUS_MOTION    BIT(7)
#define PAW3212_MOTION_STATUS_DYOVF     BIT(4)
#define PAW3212_MOTION_STATUS_DXOVF     BIT(3)

/* Configuration bits */
#define PAW3212_CONFIG_CHIP_RESET   BIT(7)
#define PAW3212_CONFIG_SLP3_ENABLE  BIT(5) // Sleep 3 Enable
#define PAW3212_CONFIG_PD_ENABLE    BIT(3) // Power Down mode

/* Mouse option bits */
#define PAW3212_MOUSE_OPT_XY_SWAP   BIT(5)
#define PAW3212_MOUSE_OPT_Y_INV     BIT(4)
#define PAW3212_MOUSE_OPT_X_INV     BIT(3)
#define PAW3212_MOUSE_OPT_12BITMODE BIT(2)

/* Mouse orientation */
#define CONFIG_PAW3212_ORIENTATION_90   1
#define CONFIG_PAW3212_ORIENTATION_180  0
#define CONFIG_PAW3212_ORIENTATION_270  0

/* SPI Mode bits */
#define CONFIG_SPI_2_WIRE_MODE  BIT(2)
#define CONFIG_SPI_3_WIRE_MODE  BIT(7) | BIT(2) // Default

#define CONFIG_SPI_PIN1_HW_RESET        0
#define CONFIG_SPI_PIN1_QUICK_BURST     BIT(4)
#define CONFIG_SPI_PIN1_HW_POWER_DOWN   BIT(5)
#define CONFIG_SPI_PIN1_NO_FUNCTION     BIT(4) | BIT(5) // Default

#define CONFIG_SPI_2_WIRE_QB_MODE       (CONFIG_SPI_2_WIRE_MODE | CONFIG_SPI_PIN1_QUICK_BURST)

/* Error Values */
#define ENOTSUP -1
#define EINVAL  -2

/* Convert deltas to x and y */
#define PAW3212_DELTA_X(xy_high, x_low) app_expand_s12((((xy_high) & 0xF0) << 4) | (x_low))
#define PAW3212_DELTA_Y(xy_high, y_low) app_expand_s12((((xy_high) & 0x0F) << 8) | (y_low))

/* Motion data length config */
/* Need to change HID report map and application code also to use 12 bit mode */
#define CONFIG_PAW3212_12_BIT_MODE      0

/* PAW3212 Sensor specific configurations */
enum paw3212_config_e
{
    /* Sensor CPI for both X and Y axes. */
    PAW3212_CONFIG_CPI = 0,
    /* Enable or disable sleep modes. */
    PAW3212_CONFIG_SLEEP_EN,
    /* Entering time from Run mode to Sleep1 mode [ms]. */
    PAW3212_CONFIG_SLEEP1_TIMEOUT,
    /* Entering time from Run mode to Sleep2 mode [ms]. */
    PAW3212_CONFIG_SLEEP2_TIMEOUT,
    /* Entering time from Run mode to Sleep3 mode [ms]. */
    PAW3212_CONFIG_SLEEP3_TIMEOUT,
    /* Sampling frequency time during Sleep1 mode [ms]. */
    PAW3212_CONFIG_SLEEP1_SAMPLE_TIME,
    /* Sampling frequency time during Sleep2 mode [ms]. */
    PAW3212_CONFIG_SLEEP2_SAMPLE_TIME,
    /* Sampling frequency time during Sleep3 mode [ms]. */
    PAW3212_CONFIG_SLEEP3_SAMPLE_TIME,
};

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
cy_rslt_t app_paw3212_power_up_reset(void);
cy_rslt_t app_paw3212_verify_id(void);
cy_rslt_t app_paw3212_mode_configure(void);
cy_rslt_t app_paw3212_reg_write(uint8_t reg, uint8_t val);
cy_rslt_t app_paw3212_reg_read(uint8_t reg, uint8_t *value);
cy_rslt_t app_paw3212_update_config(enum paw3212_config_e config, uint32_t val);
cy_rslt_t app_paw3212_sample_fetch_3_wire(void);
#if CONFIG_PAW3212_12_BIT_MODE
int16_t app_expand_s12(int16_t x);
#endif

cy_rslt_t app_paw3212_light_sleep(void);
cy_rslt_t app_paw3212_deep_sleep(void);
cy_rslt_t app_paw3212_power_down(void);
uint16_t app_paw3212_cpi_val(uint8_t cpi_mode);
cy_rslt_t app_paw3212_motion_data_handle(motion_sensor_data_t *motion_data);

#endif /* __APP_PAW3212_H_ */
