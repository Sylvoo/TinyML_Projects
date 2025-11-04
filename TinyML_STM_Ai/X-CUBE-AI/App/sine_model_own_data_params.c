/**
  ******************************************************************************
  * @file    sine_model_own_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-11-04T13:29:04+0100
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

#include "sine_model_own_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_sine_model_own_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_sine_model_own_weights_array_u64[53] = {
  0x7f7f817f817f8181U, 0x81817f81817f8181U, 0x7df5U, 0x0U,
  0xfffff4a1U, 0xa02ffff80c1U, 0x0U, 0x14f22000000f3U,
  0xffffde3b00000000U, 0x0U, 0x8f75774df10459c2U, 0xa8249281dfb63b57U,
  0xdf46c917f1e0a87cU, 0x5b47a2066281ccdcU, 0x91bef17fb0a91571U, 0xbbfa97347b1412fbU,
  0x189def7f0c3ab481U, 0xe52185d9b7a9c5a5U, 0xb0ed0621da4a1025U, 0xfa60c0ae7f5377c1U,
  0x8c678b02e498d25eU, 0xf664bcf4814b7f39U, 0xfc718abeeb3586a5U, 0x727e2c0f81a65177U,
  0x7ffb58322500e005U, 0x95128f95e3354e4U, 0xf7243a3239dbfc7fU, 0xf33697c079f9f437U,
  0x6f03c453816382U, 0xd3dd06e7103d2cf9U, 0xee6fdf62bcf18faU, 0x22c044e681b2aae0U,
  0x859ff14bbbb81436U, 0xdbf0e681d1e6dcb3U, 0x16d7d01107f5c881U, 0x37ff3fc5e0f5f117U,
  0xf8221d24f7ecd581U, 0xe1e750def80be007U, 0x8127136e138ac535U, 0x3ee416333b8f30c7U,
  0x89071e49e990ec7fU, 0x2b1bdada24b3d81dU, 0x89600000000U, 0xfffffdba000015f6U,
  0xfffffc8000001131U, 0xfffffe3afffff35fU, 0x1982U, 0xfffff859fffff876U,
  0xffffecf4fffff435U, 0x124400000dbcU, 0xdc22202821d7d4f7U, 0xdfe87f1d1815125dU,
  0xfffffe63U,
};


ai_handle g_sine_model_own_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_sine_model_own_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

