/*
 * Copyright (c) 2021, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 3-Clause Clear License
 * and the Alliance for Open Media Patent License 1.0. If the BSD 3-Clause Clear
 * License was not distributed with this source code in the LICENSE file, you
 * can obtain it at aomedia.org/license/software-license/bsd-3-c-c/.  If the
 * Alliance for Open Media Patent License 1.0 was not distributed with this
 * source code in the PATENTS file, you can obtain it at
 * aomedia.org/license/patent-license/.
 */

#ifndef AVM_AV2_ENCODER_HASH_H_
#define AVM_AV2_ENCODER_HASH_H_

#include "config/avm_config.h"

#include "avm/avm_integer.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <string.h>

typedef struct _crc_calculator {
  uint32_t remainder;
  uint32_t trunc_poly;
  uint32_t bits;
  uint32_t table[256];
  uint32_t table8[8][256];
  uint32_t final_result_mask;
} CRC_CALCULATOR;

// Initialize the crc calculator. It must be executed at least once before
// calling av2_get_crc_value().
void av2_crc_calculator_init(CRC_CALCULATOR *p_crc_calculator, uint32_t bits,
                             uint32_t truncPoly);
uint32_t av2_get_crc_value(CRC_CALCULATOR *p_crc_calculator, uint8_t *p,
                           int length);

static AVM_INLINE uint32_t av2_crc24_slice8_step(const CRC_CALCULATOR *calc,
                                                 uint32_t rem,
                                                 const void *chunk8) {
  uint64_t w;
  memcpy(&w, chunk8, sizeof(w));
  // Note: assumes little-endian byte order; big-endian targets fall back to
  // av2_get_crc_value().
  w ^= (uint64_t)(((rem >> 16) & 0xFFu) | (rem & 0xFF00u) |
                  ((rem & 0xFFu) << 16));
  return calc->table8[7][w & 0xFFu] ^ calc->table8[6][(w >> 8) & 0xFFu] ^
         calc->table8[5][(w >> 16) & 0xFFu] ^
         calc->table8[4][(w >> 24) & 0xFFu] ^
         calc->table8[3][(w >> 32) & 0xFFu] ^
         calc->table8[2][(w >> 40) & 0xFFu] ^
         calc->table8[1][(w >> 48) & 0xFFu] ^
         calc->table8[0][(w >> 56) & 0xFFu];
}

static AVM_INLINE uint32_t av2_get_crc24_8bytes(CRC_CALCULATOR *calc,
                                                const void *p) {
#if CONFIG_BIG_ENDIAN
  return av2_get_crc_value(calc, (uint8_t *)p, 8);
#else
  assert(calc->bits == 24);
  return av2_crc24_slice8_step(calc, 0, p);
#endif
}

static AVM_INLINE uint32_t av2_get_crc24_16bytes(CRC_CALCULATOR *calc,
                                                 const void *p) {
#if CONFIG_BIG_ENDIAN
  return av2_get_crc_value(calc, (uint8_t *)p, 16);
#else
  assert(calc->bits == 24);
  const uint8_t *bytes = (const uint8_t *)p;
  const uint32_t rem = av2_crc24_slice8_step(calc, 0, bytes);
  return av2_crc24_slice8_step(calc, rem, bytes + 8);
#endif
}

// CRC32C: POLY = 0x82f63b78;
typedef struct _CRC32C {
  /* Table for a quadword-at-a-time software crc. */
  uint32_t table[8][256];
} CRC32C;

// init table for software version crc32c
void av2_crc32c_calculator_init(CRC32C *p_crc32c);

#define AVM_BUFFER_SIZE_FOR_BLOCK_HASH (1 << (2 * (MAX_SB_SIZE_LOG2 - 1)))

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVM_AV2_ENCODER_HASH_H_
