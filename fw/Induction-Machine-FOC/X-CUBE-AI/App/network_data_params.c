/**
  ******************************************************************************
  * @file    network_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-03-17T21:47:52+0400
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
  0xbf23b0c5beeec2a1U, 0x3fbe8f323f338236U, 0xbf10c6933e582f89U, 0x3f9b92f3c01a97b4U,
  0xc00912653fd9f6b9U, 0x3f7e4b453dd55f8bU, 0x3f04bbc63f38f6b4U, 0xc00b8114be05ce8aU,
  0x3f5f4861beceb2afU, 0x3f8d189ebd82bf73U, 0x3fb370f6bff7c887U, 0xbe0afb09bfc13e88U,
  0xbf980c1c3f09c489U, 0xbf8f5724bf970780U, 0x3eacf4eabf07ee2dU, 0xbfa8c5483edcbde8U,
  0x3c9cefa9bda4c2dfU, 0x3feafe4c3d2baf63U, 0x3f97f50abe69960dU, 0x402692803c43c301U,
  0xbecce29c3f585886U, 0xbe7bc48c3f43004cU, 0xc00176223fe314a9U, 0x3f0256e43c96e326U,
  0x3e849d733e27b031U, 0xbff34815be766ee3U, 0x3f63b967bfb59749U, 0x3f4cfee8bcd23297U,
  0x3fb438cebfc6d5d7U, 0xbc000a273eae9e6cU, 0xbef449893fd4481eU, 0x3e924f7c3f56cc1bU,
  0x3fc8fb803ea56767U, 0xbfad468cbfa46e6dU, 0xbed086283fa4397cU, 0xbfd7e345bf4decfbU,
  0xc05ac91a401a836eU, 0x3d557769bf921724U, 0xbdefe2b4bffc44b0U, 0x3fdff57cbe32619aU,
  0x3fd8c9a63fcb023eU, 0xbfe74212be909f04U, 0x3faea8603f2f93c7U, 0xbfe7d0afbf1bbc3dU,
  0xbecca6c0400e69c6U, 0xbfaccb193fae639aU, 0xbe8958f9c00f9ab3U, 0xbfe84a4b400a8609U,
  0x3de6d74cbef553c2U, 0xbee56bbfbf941be8U, 0x3f13d526be85c566U, 0xbf58489dbf3088c7U,
  0x3eedb1aebf8a9fd7U, 0xbd712584be4f6fbbU, 0x3eb980cbbd2374a0U, 0xbc3e86853fe90ac3U,
  0x3e9aed10400d7e83U, 0xbf8f953cbf6f14bdU, 0xbf1b31783fac2b11U, 0xbf9bdb003f2c377bU,
  0xbf71a1593fba9805U, 0xbf8fca7d3e9c973fU, 0x3fc76c64bfb386d3U, 0xbe610d58be72e074U,
  0xc0278392be828c34U, 0x3edfc07b3f390583U, 0xbd8430c3bf5d5392U, 0x3f9b9a7dbf959d16U,
  0xbf067417bfca42f9U, 0x3f339eeabf33ed9aU, 0xbfbbed833f98a57cU, 0x3f10f3cc3f1b6306U,
  0xc02e6876c043edaeU, 0x406d09213bc5545bU, 0xbfa341463e6fd5a7U, 0x4021ca28bedf2809U,
  0x3e939845c0829038U, 0x3f273f673f6e81c3U, 0x3f5cbdf940a55edaU, 0xbea7a7143f0e2bd7U,
  0xbe14687b3f0b990cU, 0x3ebbb5b73f861a8aU, 0xbf0532223eea436eU, 0x3f9ceb9a3f197312U,
  0xbed61ca43d5f479cU, 0xbfa4e6f7be7b5791U, 0x3ebf2cfe3e0b3e9dU, 0xbd1665cbbfcc3056U,
  0x3fe576d23db332a8U, 0xbfd0c7593e1e74b0U, 0x3fad56c13f4940feU, 0xbf916ef03eef0957U,
  0xbdf3c2b5407d172bU, 0x3f2955dfbe640102U, 0xbe6aaefdc012c887U, 0xbfe08fed3fd0cfc1U,
  0x400571a9beefac28U, 0xbf892268bef84e05U, 0x3e59a7813f09b929U, 0xbf9219183f35707eU,
  0x3f979303be5e4345U, 0x3ea88fb23c323ccaU, 0x3eb28e2ebe3109d9U, 0x3f33bd6e3eb2f2e5U,
  0x3f16edd83fddf79bU, 0x3dae778fbf9a4351U, 0x3f3a1188bfbe66daU, 0x3f8072ef3f9cbfcfU,
  0x3fe0962fbfd0c7c8U, 0xbf97f9b43fbc631dU, 0x3fdd2c46c03f1121U, 0xbfa5bbe6bf8c0f50U,
  0xbfb53a08U,
};


ai_handle g_network_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_network_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

