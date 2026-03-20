/**
  ******************************************************************************
  * @file    network_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-03-20T21:06:47+0400
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
  0xbeb1583d3ffb1301U, 0x4011a9fdbc9f1d61U, 0xbe641741390d123fU, 0x3ee2d1d7be9ab0acU,
  0xbf9a027f3f7c85b9U, 0xbedfbea3bc919795U, 0xbf066f503f56c7fdU, 0xbf0e2ffb3d4ff4a6U,
  0xbefa15053f1c8e05U, 0x3e632e853cb5b5a5U, 0xbe861e61bf87c8abU, 0x3f5537d4be8b1e0dU,
  0xbe95fae7bdf320c2U, 0x3d1adc4cc04677efU, 0xbf84886fc036b0b8U, 0xbf933e7e3eee77f0U,
  0x401c25a3bf3543b2U, 0x3ec067a73ea7dff4U, 0x4008b73abf1babfaU, 0x3ee79ae5bc34b3dcU,
  0xbf0710153f490310U, 0x3ec4bcc9bc6e2ba0U, 0xbf36d67f3f4871c5U, 0x3e8cfc05bd1acdf8U,
  0xc00844c73ffdc771U, 0xbfc8b3f8be4af9bbU, 0x3f6899bdbf66eb51U, 0x3eef3352bcafb744U,
  0x3d10cee33f520f2eU, 0xbea75c2b3f6b0efcU, 0x3e780842bf38f4fbU, 0xbe3e54093f682482U,
  0x3de3856a3ffa262dU, 0xbec250383ed6693dU, 0x3e57dc39be4419d1U, 0xbf1a4a903fc15815U,
  0x3eb0b2f33f3b0ba8U, 0xbe3d7773bef6b1dcU, 0xbdd7a508bfa8ede0U, 0x3f41969a3f58f58fU,
  0xbeda30123ed0994eU, 0xbef61f0cbec7b3e6U, 0xbe2609b3beb8981dU, 0x3f6f6f28bda65f5fU,
  0xbf508b71bfbbf6f8U, 0xbd8c4a40bd9dcf5bU, 0x3f30811e3fab0638U, 0x3ebaf3563e7763a5U,
  0x3f1487e840179ec8U, 0xbec0c2923f13f005U, 0xbdde29cf3d0c6779U, 0xbe2604e5bf8810dcU,
  0xbf1f3743befe4e8eU, 0x3dc85935bee33b4bU, 0xbf41fa873e511181U, 0x3f32ffaa3e9d414bU,
  0xbf245802bf526744U, 0xbdea2b7cbf9b5e0eU, 0xbe64898ebe555a86U, 0x3e101ef3be616cd0U,
  0x3c687f8cbeab3390U, 0xbf55377fbe21d825U, 0x3f6cd633be9d073bU, 0xbf4163ca3b1fa351U,
  0xbdd512013ec36009U, 0x3f41c4393f6a6892U, 0x3da6a44d3dd6e284U, 0x3f80dfd3be8c23feU,
  0xbf29dc8dbed08200U, 0x3f2fc8bd3f233192U, 0xbf8c5a083f3a835fU, 0xbe1f96463ec863d9U,
  0xbd81b4fb3daabea1U, 0x3f897c413f4286f7U, 0xbf20874b3f25163dU, 0xbe1adba5be81a386U,
  0xbf1432d73e8d3072U, 0x3f5bb7193f765baeU, 0xbf7fc4243f16b453U, 0xbeac96903ec95d05U,
  0x3e7f27263f893025U, 0xbec0964abe487c2fU, 0xbe672388bce36083U, 0x3f850fdfbf3189caU,
  0xbf0c3851bf503eb6U, 0x3d8e95bebdd4c404U, 0x3da2b21b3f42aa22U, 0x3d88cb113d1fbdadU,
  0x3e55ca55bf31adc8U, 0xbf0e14383f9ebd85U, 0x3f1d7019bdf15524U, 0xbee93ad63e0a16d1U,
  0x3c36d9b93f8f7d26U, 0xbdf5a5fbbe86d4a4U, 0xbedb93c3bf817d65U, 0x3cded6fbbf51be0fU,
  0x3e6a831d3edc6f4aU, 0x3eb15cff3f4807e6U, 0xbcdbb5ca3e07f9e0U, 0x3f6c41dbbe80d7f4U,
  0xbf09e126bea3f66aU, 0x3f2e48823f1d28fcU, 0xbf87ad8e3ec9d0c2U, 0xbeb397d93c27d907U,
  0x3f5ac1683e814f02U, 0x3db06683bb855b4aU, 0x3d3d6b38bee5ad1fU, 0x3e7dd3233c8612e6U,
  0x3f9572f83f3d492cU, 0xbf8302a33f7ef945U, 0x3f5a1327bf6b42e7U, 0xbf670980bfa13b6dU,
  0xbe143e45U,
};


ai_handle g_network_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_network_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

