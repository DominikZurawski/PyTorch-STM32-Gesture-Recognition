/*
 * adxl345.c
 *
 *  Created on: Jul 22, 2025
 *      Author: Dominik
 */

/* adxl345.c */
#include "adxl345.h"


#define ADXL345_CS_PORT GPIOA
#define ADXL345_CS_PIN  GPIO_PIN_4

// ADXL345 registers
#define ADXL345_DEVID       0x00
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0      0x32
#define ADXL345_READ        0x80
#define ADXL345_MULTI       0x40

static SPI_HandleTypeDef *adxl_hspi;

void normalizeAccelData(float *x, float *y, float *z) {
    // Usuń grawitację (zakładając, że Z=1g w spoczynku)
    *z -= 1.0f;

    // Filtr dolnoprzepustowy
    static float prev_x = 0, prev_y = 0, prev_z = 0;
    float alpha = 0.1f;
    *x = alpha * (*x) + (1-alpha) * prev_x;
    *y = alpha * (*y) + (1-alpha) * prev_y;
    *z = alpha * (*z) + (1-alpha) * prev_z;
    prev_x = *x; prev_y = *y; prev_z = *z;
}

float calculateMagnitude(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

static void CS_Select(void)   { HAL_GPIO_WritePin(ADXL345_CS_PORT, ADXL345_CS_PIN, GPIO_PIN_RESET); }
static void CS_Deselect(void) { HAL_GPIO_WritePin(ADXL345_CS_PORT, ADXL345_CS_PIN, GPIO_PIN_SET); }

void ADXL345_Write(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    CS_Select();
    HAL_SPI_Transmit(adxl_hspi, buf, 2, HAL_MAX_DELAY);
    CS_Deselect();
}

void ADXL345_Read(uint8_t reg, uint8_t *data, uint8_t len) {
    reg |= ADXL345_READ | ADXL345_MULTI;
    CS_Select();
    HAL_SPI_Transmit(adxl_hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(adxl_hspi, data, len, HAL_MAX_DELAY);
    CS_Deselect();
}

void ADXL345_Init(SPI_HandleTypeDef *hspi) {
    adxl_hspi = hspi;
    ADXL345_Write(ADXL345_DATA_FORMAT, 0x01); // Range setting ±4g
    ADXL345_Write(ADXL345_POWER_CTL, 0x08);   // Activate measurements
}

void ADXL345_GetXYZ(SPI_HandleTypeDef *hspi, float *x, float *y, float *z) {
    uint8_t buf[6];
    ADXL345_Read(ADXL345_DATAX0, buf, 6);
    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    // Conversion to float
    // For a range of ±4g, the sensitivity is about 2mg/LSB in 13-bit mode. We will use a simplified scaling.
    *x = raw_x * 0.004f;
    *y = raw_y * 0.004f;
    *z = raw_z * 0.004f;
}

