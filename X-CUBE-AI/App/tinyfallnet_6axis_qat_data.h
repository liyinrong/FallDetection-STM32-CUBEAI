/**
  ******************************************************************************
  * @file    tinyfallnet_6axis_qat_data.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Thu Jan  4 17:52:26 2024
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef TINYFALLNET_6AXIS_QAT_DATA_H
#define TINYFALLNET_6AXIS_QAT_DATA_H
#pragma once

#include "tinyfallnet_6axis_qat_config.h"
#include "tinyfallnet_6axis_qat_data_params.h"

AI_DEPRECATED
#define AI_TINYFALLNET_6AXIS_QAT_DATA_ACTIVATIONS(ptr_)  \
  ai_tinyfallnet_6axis_qat_data_activations_buffer_get(AI_HANDLE_PTR(ptr_))

AI_DEPRECATED
#define AI_TINYFALLNET_6AXIS_QAT_DATA_WEIGHTS(ptr_)  \
  ai_tinyfallnet_6axis_qat_data_weights_buffer_get(AI_HANDLE_PTR(ptr_))


AI_API_DECLARE_BEGIN


extern const ai_u64 s_tinyfallnet_6axis_qat_weights_array_u64[4145];



/*!
 * @brief Get network activations buffer initialized struct.
 * @ingroup tinyfallnet_6axis_qat_data
 * @param[in] ptr a pointer to the activations array storage area
 * @return an ai_buffer initialized struct
 */
AI_DEPRECATED
AI_API_ENTRY
ai_buffer ai_tinyfallnet_6axis_qat_data_activations_buffer_get(const ai_handle ptr);

/*!
 * @brief Get network weights buffer initialized struct.
 * @ingroup tinyfallnet_6axis_qat_data
 * @param[in] ptr a pointer to the weights array storage area
 * @return an ai_buffer initialized struct
 */
AI_DEPRECATED
AI_API_ENTRY
ai_buffer ai_tinyfallnet_6axis_qat_data_weights_buffer_get(const ai_handle ptr);

/*!
 * @brief Get network weights array pointer as a handle ptr.
 * @ingroup tinyfallnet_6axis_qat_data
 * @return a ai_handle pointer to the weights array
 */
AI_DEPRECATED
AI_API_ENTRY
ai_handle ai_tinyfallnet_6axis_qat_data_weights_get(void);


/*!
 * @brief Get network params configuration data structure.
 * @ingroup tinyfallnet_6axis_qat_data
 * @return true if a valid configuration is present, false otherwise
 */
AI_API_ENTRY
ai_bool ai_tinyfallnet_6axis_qat_data_params_get(ai_network_params* params);


AI_API_DECLARE_END

#endif /* TINYFALLNET_6AXIS_QAT_DATA_H */

