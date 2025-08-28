/**
  ******************************************************************************
  * @file    ai_model_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-07-22T19:58:31+0200
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef AI_MODEL_DATA_PARAMS_H
#define AI_MODEL_DATA_PARAMS_H

#include "ai_platform.h"

/*
#define AI_AI_MODEL_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_ai_model_data_weights_params[1]))
*/

#define AI_AI_MODEL_DATA_CONFIG               (NULL)


#define AI_AI_MODEL_DATA_ACTIVATIONS_SIZES \
  { 8704, }
#define AI_AI_MODEL_DATA_ACTIVATIONS_SIZE     (8704)
#define AI_AI_MODEL_DATA_ACTIVATIONS_COUNT    (1)
#define AI_AI_MODEL_DATA_ACTIVATION_1_SIZE    (8704)



#define AI_AI_MODEL_DATA_WEIGHTS_SIZES \
  { 454944, }
#define AI_AI_MODEL_DATA_WEIGHTS_SIZE         (454944)
#define AI_AI_MODEL_DATA_WEIGHTS_COUNT        (1)
#define AI_AI_MODEL_DATA_WEIGHT_1_SIZE        (454944)



#define AI_AI_MODEL_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_ai_model_activations_table[1])

extern ai_handle g_ai_model_activations_table[1 + 2];



#define AI_AI_MODEL_DATA_WEIGHTS_TABLE_GET() \
  (&g_ai_model_weights_table[1])

extern ai_handle g_ai_model_weights_table[1 + 2];


#endif    /* AI_MODEL_DATA_PARAMS_H */
