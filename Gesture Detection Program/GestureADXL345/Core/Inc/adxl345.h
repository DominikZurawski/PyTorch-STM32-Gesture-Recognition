/*
 * adxl345.h
 *
 *  Created on: Jul 22, 2025
 *      Author: Dominik
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "stm32l4xx_hal.h"

void ADXL345_Init(SPI_HandleTypeDef *hspi);
void ADXL345_GetXYZ(SPI_HandleTypeDef *hspi, float *x, float *y, float *z);
void normalizeAccelData(float *x, float *y, float *z);
float calculateMagnitude(float x, float y, float z);

#endif /* INC_ADXL345_H_ */
