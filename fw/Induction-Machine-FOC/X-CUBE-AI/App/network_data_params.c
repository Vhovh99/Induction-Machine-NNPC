/**
  ******************************************************************************
  * @file    network_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-03-16T23:24:48+0400
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#include "network_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_network_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_network_weights_array_u64[13] = {
  0xbeac0c754070275cU, 0x3fc78f213d87b6c9U, 0x3f5e9dd4bf365f08U, 0x3feecac7beb1fe65U,
  0xbed51f6c3f2a9417U, 0xbd834731bd062eabU, 0xbdad46c83f9062aaU, 0xbf6482d3bea3f9beU,
  0x3f897d5a40f2bdacU, 0xbe686bd1be8cc4f0U, 0xbfd1325c40372bb8U, 0x3fa56160c05c3d59U,
  0xc00be7c4U,
};


ai_handle g_network_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_network_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

