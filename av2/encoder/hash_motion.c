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

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "config/av2_rtcd.h"

#include "avm_ports/mem.h"
#include "av2/encoder/block.h"
#include "av2/encoder/hash.h"
#include "av2/encoder/hash_motion.h"

#define kSrcBits 16
#define kBlockSizeBits 3
#define kMaxAddr (1 << (kSrcBits + kBlockSizeBits))

static void get_pixels_in_1D_short_array_by_block_2x2(const uint16_t *y_src,
                                                      int stride,
                                                      uint16_t *p_pixels_in1D) {
  const uint16_t *p_pel = y_src;
  int index = 0;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      p_pixels_in1D[index++] = p_pel[j];
    }
    p_pel += stride;
  }
}

static int is_block16_2x2_row_same_value(const uint16_t *p) {
  if (p[0] != p[1] || p[2] != p[3]) {
    return 0;
  }
  return 1;
}

static int is_block16_2x2_col_same_value(const uint16_t *p) {
  if ((p[0] != p[2]) || (p[1] != p[3])) {
    return 0;
  }
  return 1;
}

// the hash value (hash_value1 consists two parts, the first 3 bits relate to
// the block size and the remaining 16 bits are the crc values. This function
// is used to get the first 3 bits.
static int hash_block_size_to_index(int block_size) {
  switch (block_size) {
    case 4: return 0;
    case 8: return 1;
    case 16: return 2;
    case 32: return 3;
    case 64: return 4;
    case 128: return 5;
    case 256: return 6;
    default: return -1;
  }
}

void av2_hash_table_init(IntraBCHashInfo *intrabc_hash_info) {
  if (!intrabc_hash_info->g_crc_initialized) {
    av2_crc_calculator_init(&intrabc_hash_info->crc_calculator1, 24, 0x5D6DCB);
    av2_crc_calculator_init(&intrabc_hash_info->crc_calculator2, 24, 0x864CFB);
    intrabc_hash_info->g_crc_initialized = 1;
  }
  intrabc_hash_info->intrabc_hash_table.p_lookup_table = NULL;
}

void av2_hash_table_clear_all(hash_table *p_hash_table) {
  if (p_hash_table->p_lookup_table == NULL) {
    return;
  }
  for (int i = 0; i < kMaxAddr; i++) {
    if (p_hash_table->p_lookup_table[i] != NULL) {
      avm_vector_destroy(p_hash_table->p_lookup_table[i]);
      avm_free(p_hash_table->p_lookup_table[i]);
      p_hash_table->p_lookup_table[i] = NULL;
    }
  }
}

void av2_hash_table_destroy(hash_table *p_hash_table) {
  av2_hash_table_clear_all(p_hash_table);
  avm_free(p_hash_table->p_lookup_table);
  p_hash_table->p_lookup_table = NULL;
}

void av2_hash_table_create(hash_table *p_hash_table) {
  if (p_hash_table->p_lookup_table != NULL) {
    av2_hash_table_clear_all(p_hash_table);
    return;
  }
  p_hash_table->p_lookup_table =
      (Vector **)avm_malloc(sizeof(p_hash_table->p_lookup_table[0]) * kMaxAddr);
  memset(p_hash_table->p_lookup_table, 0,
         sizeof(p_hash_table->p_lookup_table[0]) * kMaxAddr);
}

static AVM_INLINE void hash_table_add_to_table(hash_table *p_hash_table,
                                               uint32_t hash_value,
                                               const block_hash *curr_hash) {
  Vector *vec = p_hash_table->p_lookup_table[hash_value];
  if (vec == NULL) {
    vec = avm_malloc(sizeof(*vec));
    avm_vector_setup(vec, 10, sizeof(*curr_hash));
    p_hash_table->p_lookup_table[hash_value] = vec;
  }
  if (vec->size < vec->capacity) {
    ((block_hash *)vec->data)[vec->size++] = *curr_hash;
  } else {
    block_hash tmp = *curr_hash;
    avm_vector_push_back(vec, &tmp);
  }
}

static AVM_INLINE uint32_t block_hash_xy_key(const block_hash *h) {
  return ((uint32_t)(uint16_t)h->x << 16) | (uint16_t)h->y;
}

static int compare_block_hash_xy(const void *a, const void *b) {
  const uint32_t ka = block_hash_xy_key((const block_hash *)a);
  const uint32_t kb = block_hash_xy_key((const block_hash *)b);
  return (ka > kb) - (ka < kb);
}

int32_t av2_hash_table_count(const hash_table *p_hash_table,
                             uint32_t hash_value) {
  if (p_hash_table->p_lookup_table[hash_value] == NULL) {
    return 0;
  } else {
    return (int32_t)(p_hash_table->p_lookup_table[hash_value]->size);
  }
}

Iterator av2_hash_get_first_iterator(hash_table *p_hash_table,
                                     uint32_t hash_value) {
  assert(av2_hash_table_count(p_hash_table, hash_value) > 0);
  return avm_vector_begin(p_hash_table->p_lookup_table[hash_value]);
}

void av2_generate_block_2x2_hash_value(IntraBCHashInfo *intrabc_hash_info,
                                       const YV12_BUFFER_CONFIG *picture,
                                       uint32_t *pic_block_hash[2],
                                       int8_t *pic_block_same_info[3]) {
  const int width = 2;
  const int height = 2;
  const int x_end = picture->y_crop_width - width + 1;
  const int y_end = picture->y_crop_height - height + 1;
  CRC_CALCULATOR *calc_1 = &intrabc_hash_info->crc_calculator1;
  CRC_CALCULATOR *calc_2 = &intrabc_hash_info->crc_calculator2;

  DECLARE_ALIGNED(8, uint16_t, p[4]);
  int pos = 0;
  for (int y_pos = 0; y_pos < y_end; y_pos++) {
    for (int x_pos = 0; x_pos < x_end; x_pos++) {
      get_pixels_in_1D_short_array_by_block_2x2(
          picture->y_buffer + y_pos * picture->y_stride + x_pos,
          picture->y_stride, p);
      pic_block_same_info[0][pos] = is_block16_2x2_row_same_value(p);
      pic_block_same_info[1][pos] = is_block16_2x2_col_same_value(p);

      pic_block_hash[0][pos] = av2_get_crc24_8bytes(calc_1, p);
      pic_block_hash[1][pos] = av2_get_crc24_8bytes(calc_2, p);
      pos++;
    }
    pos += width - 1;
  }
}

void av2_generate_block_hash_value(IntraBCHashInfo *intrabc_hash_info,
                                   const YV12_BUFFER_CONFIG *picture,
                                   int block_size,
                                   uint32_t *src_pic_block_hash[2],
                                   uint32_t *dst_pic_block_hash[2],
                                   int8_t *src_pic_block_same_info[3],
                                   int8_t *dst_pic_block_same_info[3]) {
  CRC_CALCULATOR *calc_1 = &intrabc_hash_info->crc_calculator1;
  CRC_CALCULATOR *calc_2 = &intrabc_hash_info->crc_calculator2;

  const int pic_width = picture->y_crop_width;
  const int x_end = picture->y_crop_width - block_size + 1;
  const int y_end = picture->y_crop_height - block_size + 1;

  const int src_size = block_size >> 1;
  const int quad_size = block_size >> 2;

  DECLARE_ALIGNED(8, uint32_t, p[4]);

  int pos = 0;
  for (int y_pos = 0; y_pos < y_end; y_pos++) {
    for (int x_pos = 0; x_pos < x_end; x_pos++) {
      p[0] = src_pic_block_hash[0][pos];
      p[1] = src_pic_block_hash[0][pos + src_size];
      p[2] = src_pic_block_hash[0][pos + src_size * pic_width];
      p[3] = src_pic_block_hash[0][pos + src_size * pic_width + src_size];
      dst_pic_block_hash[0][pos] = av2_get_crc24_16bytes(calc_1, p);

      p[0] = src_pic_block_hash[1][pos];
      p[1] = src_pic_block_hash[1][pos + src_size];
      p[2] = src_pic_block_hash[1][pos + src_size * pic_width];
      p[3] = src_pic_block_hash[1][pos + src_size * pic_width + src_size];
      dst_pic_block_hash[1][pos] = av2_get_crc24_16bytes(calc_2, p);

      dst_pic_block_same_info[0][pos] =
          src_pic_block_same_info[0][pos] &&
          src_pic_block_same_info[0][pos + quad_size] &&
          src_pic_block_same_info[0][pos + src_size] &&
          src_pic_block_same_info[0][pos + src_size * pic_width] &&
          src_pic_block_same_info[0][pos + src_size * pic_width + quad_size] &&
          src_pic_block_same_info[0][pos + src_size * pic_width + src_size];

      dst_pic_block_same_info[1][pos] =
          src_pic_block_same_info[1][pos] &&
          src_pic_block_same_info[1][pos + src_size] &&
          src_pic_block_same_info[1][pos + quad_size * pic_width] &&
          src_pic_block_same_info[1][pos + quad_size * pic_width + src_size] &&
          src_pic_block_same_info[1][pos + src_size * pic_width] &&
          src_pic_block_same_info[1][pos + src_size * pic_width + src_size];
      pos++;
    }
    pos += block_size - 1;
  }

  if (block_size >= 4) {
    const int size_minus_1 = block_size - 1;
    pos = 0;
    for (int y_pos = 0; y_pos < y_end; y_pos++) {
      for (int x_pos = 0; x_pos < x_end; x_pos++) {
        dst_pic_block_same_info[2][pos] =
            (!dst_pic_block_same_info[0][pos] &&
             !dst_pic_block_same_info[1][pos]) ||
            (((x_pos & size_minus_1) == 0) && ((y_pos & size_minus_1) == 0));
        pos++;
      }
      pos += block_size - 1;
    }
  }
}

void av2_add_to_hash_map_by_row_with_precal_data(hash_table *p_hash_table,
                                                 uint32_t *pic_hash[2],
                                                 int8_t *pic_is_same,
                                                 int pic_width, int pic_height,
                                                 int block_size) {
  const int x_end = pic_width - block_size + 1;
  const int y_end = pic_height - block_size + 1;

  const int8_t *src_is_added = pic_is_same;
  const uint32_t *src_hash[2] = { pic_hash[0], pic_hash[1] };

  int add_value = hash_block_size_to_index(block_size);
  assert(add_value >= 0);
  add_value <<= kSrcBits;
  const int crc_mask = (1 << kSrcBits) - 1;

  // Scan row-major for sequential L1 cache access, then restore exact (x, y)
  // bucket order within this block_size's disjoint [add_value, add_value +
  // 2^kSrcBits) slice so hash lookups remain 100% bit-identical.
  for (int y_pos = 0; y_pos < y_end; y_pos++) {
    const int row_offset = y_pos * pic_width;
    for (int x_pos = 0; x_pos < x_end; x_pos++) {
      const int pos = row_offset + x_pos;
      if (src_is_added[pos]) {
        block_hash curr_block_hash;
        curr_block_hash.x = x_pos;
        curr_block_hash.y = y_pos;

        const uint32_t hash_value1 = (src_hash[0][pos] & crc_mask) + add_value;
        curr_block_hash.hash_value2 = src_hash[1][pos];

        hash_table_add_to_table(p_hash_table, hash_value1, &curr_block_hash);
      }
    }
  }

  const int bucket_end = add_value + (1 << kSrcBits);
  for (int bucket = add_value; bucket < bucket_end; bucket++) {
    Vector *vec = p_hash_table->p_lookup_table[bucket];
    if (vec == NULL || vec->size <= 1) continue;
    block_hash *arr = (block_hash *)vec->data;
    const size_t n = vec->size;
    if (n <= 16) {
      for (size_t i = 1; i < n; i++) {
        const block_hash cur = arr[i];
        const uint32_t cur_key = block_hash_xy_key(&cur);
        size_t j = i;
        while (j > 0 && block_hash_xy_key(&arr[j - 1]) > cur_key) {
          arr[j] = arr[j - 1];
          j--;
        }
        arr[j] = cur;
      }
    } else {
      qsort(arr, n, sizeof(*arr), compare_block_hash_xy);
    }
  }
}

int av2_hash_is_horizontal_perfect(const YV12_BUFFER_CONFIG *picture,
                                   int block_size, int x_start, int y_start) {
  const int stride = picture->y_stride;
  const uint16_t *p = picture->y_buffer + y_start * stride + x_start;

  for (int i = 0; i < block_size; i++) {
    for (int j = 1; j < block_size; j++) {
      if (p[j] != p[0]) {
        return 0;
      }
    }
    p += stride;
  }

  return 1;
}

int av2_hash_is_vertical_perfect(const YV12_BUFFER_CONFIG *picture,
                                 int block_size, int x_start, int y_start) {
  const int stride = picture->y_stride;
  const uint16_t *p = picture->y_buffer + y_start * stride + x_start;

  for (int i = 0; i < block_size; i++) {
    for (int j = 1; j < block_size; j++) {
      if (p[j * stride + i] != p[i]) {
        return 0;
      }
    }
  }
  return 1;
}

void av2_get_block_hash_value(IntraBCHashInfo *intrabc_hash_info,
                              const uint16_t *y_src, int stride, int block_size,
                              uint32_t *hash_value1, uint32_t *hash_value2) {
  int add_value = hash_block_size_to_index(block_size);
  assert(add_value >= 0);
  add_value <<= kSrcBits;
  const int crc_mask = (1 << kSrcBits) - 1;

  CRC_CALCULATOR *calc_1 = &intrabc_hash_info->crc_calculator1;
  CRC_CALCULATOR *calc_2 = &intrabc_hash_info->crc_calculator2;
  uint32_t **buf_1 = intrabc_hash_info->hash_value_buffer[0];
  uint32_t **buf_2 = intrabc_hash_info->hash_value_buffer[1];

  // 2x2 subblock hash values in current CU
  int sub_block_in_width = (block_size >> 1);
  DECLARE_ALIGNED(8, uint16_t, pixel_to_hash[4]);
  for (int y_pos = 0; y_pos < block_size; y_pos += 2) {
    for (int x_pos = 0; x_pos < block_size; x_pos += 2) {
      int pos = (y_pos >> 1) * sub_block_in_width + (x_pos >> 1);
      get_pixels_in_1D_short_array_by_block_2x2(y_src + y_pos * stride + x_pos,
                                                stride, pixel_to_hash);
      assert(pos < AVM_BUFFER_SIZE_FOR_BLOCK_HASH);
      buf_1[0][pos] = av2_get_crc24_8bytes(calc_1, pixel_to_hash);
      buf_2[0][pos] = av2_get_crc24_8bytes(calc_2, pixel_to_hash);
    }
  }

  int src_sub_block_in_width = sub_block_in_width;
  sub_block_in_width >>= 1;

  int src_idx = 1;
  int dst_idx = 0;

  // 4x4 subblock hash values to current block hash values
  DECLARE_ALIGNED(8, uint32_t, to_hash[4]);
  for (int sub_width = 4; sub_width <= block_size; sub_width *= 2) {
    src_idx = 1 - src_idx;
    dst_idx = 1 - dst_idx;

    int dst_pos = 0;
    for (int y_pos = 0; y_pos < sub_block_in_width; y_pos++) {
      for (int x_pos = 0; x_pos < sub_block_in_width; x_pos++) {
        int srcPos = (y_pos << 1) * src_sub_block_in_width + (x_pos << 1);

        assert(srcPos + 1 < AVM_BUFFER_SIZE_FOR_BLOCK_HASH);
        assert(srcPos + src_sub_block_in_width + 1 <
               AVM_BUFFER_SIZE_FOR_BLOCK_HASH);
        assert(dst_pos < AVM_BUFFER_SIZE_FOR_BLOCK_HASH);
        to_hash[0] = buf_1[src_idx][srcPos];
        to_hash[1] = buf_1[src_idx][srcPos + 1];
        to_hash[2] = buf_1[src_idx][srcPos + src_sub_block_in_width];
        to_hash[3] = buf_1[src_idx][srcPos + src_sub_block_in_width + 1];

        buf_1[dst_idx][dst_pos] = av2_get_crc24_16bytes(calc_1, to_hash);

        to_hash[0] = buf_2[src_idx][srcPos];
        to_hash[1] = buf_2[src_idx][srcPos + 1];
        to_hash[2] = buf_2[src_idx][srcPos + src_sub_block_in_width];
        to_hash[3] = buf_2[src_idx][srcPos + src_sub_block_in_width + 1];
        buf_2[dst_idx][dst_pos] = av2_get_crc24_16bytes(calc_2, to_hash);
        dst_pos++;
      }
    }

    src_sub_block_in_width = sub_block_in_width;
    sub_block_in_width >>= 1;
  }

  *hash_value1 = (buf_1[dst_idx][0] & crc_mask) + add_value;
  *hash_value2 = buf_2[dst_idx][0];
}
