/**
  ******************************************************************************
  * @file    network_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-03-15T23:17:22+0400
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
const ai_u64 s_network_weights_array_u64[113] = {
  0x3f6f192f4063e119U, 0x3fc1e6afbd9eccfcU, 0xbe0284ccbf7ddebdU, 0x3f4cca69beb13bb6U,
  0xbea1f13f402d872eU, 0x3d956fb53d7f12b6U, 0x3e4ad9583f1d9e90U, 0xbedc5e51bd0eec46U,
  0x3fc613973f69d4d9U, 0xbf12ca2d3e5a5a3fU, 0xbf2a3242beb4353cU, 0x3f4ed72cbea600acU,
  0xbec32698bf43676bU, 0x3ea7571bbff2ef9cU, 0xc02331b0bf6984a2U, 0xbf0fc3273e189c33U,
  0xbed335173f5ee9a4U, 0x3db9e166bef092d3U, 0x3f9c792b3fc4f523U, 0xbfeb394bbe8c1c4bU,
  0xbf073472be642113U, 0x3ead80753f70ca69U, 0xc005f43bbd9c40adU, 0xbf105cbfbdbc9e9aU,
  0x3ef04feebfad1845U, 0xbf76d438c019dd03U, 0xbdcdabb1c049fc8fU, 0x3ce498ee3d0310bbU,
  0x401f51753fd42f8aU, 0xbf375f3ebe43ec96U, 0xbf8ba25dbf78243dU, 0x3eede7943f220ca1U,
  0xbf1f49e33fbf3f3cU, 0xbf4366203e86e4a3U, 0xbd209f15bf2bdb04U, 0xbeb3cf9b3f9202c4U,
  0xbffa70d33f2f7e0aU, 0xbf2ca366bf1efdc9U, 0x3bd405e2bf4c52dcU, 0x3f4c012fbe953611U,
  0xbf93a5f9bfd9fb4fU, 0xbe8963703ea78ac3U, 0x3be050793dff30baU, 0xbe70d53a3f908e11U,
  0xbe057803bdb98587U, 0x3e8bb202be65ec8cU, 0xbfa85074bf4c56c1U, 0xbe2e3989becc6103U,
  0xbf08a1db3f1fed6aU, 0x3e8501803f85d0a4U, 0xbf84ba903f272a4cU, 0xbfbfd33bbe75129dU,
  0x3e3eae2a3e75f506U, 0xc006a2d3bd36239bU, 0xbfb5c3213f07f892U, 0xbe27f73d3ff36eeeU,
  0x3e554ab93e0da544U, 0x3deabb4ebee25c29U, 0xbebb1ba13f250cc3U, 0xbfa036e4be9a26dcU,
  0xbe1a67f9be9a1663U, 0xbf5f34483d086ec4U, 0x3f2b2818bdfa9db7U, 0xbf08aea03f532172U,
  0xbee8fa403fd5657aU, 0x3ded98283f6c6244U, 0x3e346382bd03d042U, 0x3bc2cd5fbfa808f4U,
  0xbed6d0063e79167dU, 0x3f33402e3ea5a5b4U, 0xbf93b94a3fba13b3U, 0xbda487753f38715eU,
  0x3d02aa1fbf00726fU, 0x3f645c96be5ff983U, 0xbef545453f3112e3U, 0xbe24403a3d938b06U,
  0x3fdc227e3d9f4b1aU, 0x3e7bfae13f0c5d24U, 0xbd1b953b3ea99cf7U, 0xbf863ccf3dc58897U,
  0x3e0c14843f026a67U, 0xbee1d40f3e723f7bU, 0x3e0e951cbdf08c74U, 0x3eaf962bbeb3a7bdU,
  0xbf9bf90fbed49492U, 0xbe0bdcd6be1247a2U, 0xbc9994383f16fe68U, 0x3e921b83be7a5f9eU,
  0x3f0a9fbc3ecd1974U, 0xbe4379d13d584e27U, 0x3e1187853e67ae1fU, 0x3d997c273eb8e40eU,
  0x3f560fcd3ef520feU, 0x3e15d8643f16119fU, 0x3f80b602bf3d10d3U, 0xbe996575be316188U,
  0xbd3cae483e10fe76U, 0xbf13d05ebf250b13U, 0x3f027defbfa2d3beU, 0x3fb611eb3e9cc2e3U,
  0xbe982f76be0967e4U, 0x3f5575ff3c3385daU, 0x3f84373cbe975060U, 0x3f3ffd48bfe690ccU,
  0xbdd7206a3e95af15U, 0x3e8b9db53da2f0afU, 0xbbc5994fbf0b6f5bU, 0x3e751406bddd6da2U,
  0xbf5e99a13f15bcacU, 0xbf49d9b13eeabac9U, 0x3eb3748bbf1daac0U, 0xbf803030bea9fd0cU,
  0xbf227ee4U,
};


ai_handle g_network_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_network_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

