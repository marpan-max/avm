/*
 * Copyright (c) 2019, Alliance for Open Media. All rights reserved.
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "config/avm_config.h"
#include "config/avm_dsp_rtcd.h"
#include "config/av2_rtcd.h"

#include "avm_dsp/avm_dsp_common.h"
#include "avm_dsp/binary_codes_writer.h"
#include "avm_ports/mem.h"
#include "avm_ports/avm_timer.h"

#include "av2/common/reconinter.h"
#include "av2/common/blockd.h"
#include "av2/common/quant_common.h"

#include "av2/encoder/encodeframe.h"
#include "av2/encoder/encodeframe_utils.h"
#include "av2/encoder/var_based_part.h"
#include "av2/encoder/reconinter_enc.h"
#include "av2/encoder/rdopt_utils.h"

// Macros for common video resolutions: width x height
// For example, 720p represents video resolution of 1280x720 pixels.
#define RESOLUTION_288P 352 * 288
#define RESOLUTION_360P 640 * 360
#define RESOLUTION_480P 640 * 480
#define RESOLUTION_720P 1280 * 720
#define RESOLUTION_1080P 1920 * 1080
#define RESOLUTION_1440P 2560 * 1440
#define RESOLUTION_4K 3840 * 2160

// Possible values for the force_split variable while evaluating variance based
// partitioning.
enum {
  // Evaluate all partition types
  PART_EVAL_ALL = 0,
  // Force PARTITION_SPLIT
  PART_EVAL_ONLY_SPLIT = 1,
  // Force PARTITION_NONE
  PART_EVAL_ONLY_NONE = 2
} UENUM1BYTE(PART_EVAL_STATUS);

typedef struct {
  VPVariance *part_variances;
  VPartVar *split[4];
} variance_node;

static inline void tree_to_node(void *data, BLOCK_SIZE bsize,
                                variance_node *node) {
  node->part_variances = NULL;
  switch (bsize) {
    case BLOCK_128X128: {
      VP128x128 *vt = (VP128x128 *)data;
      node->part_variances = &vt->part_variances;
      for (int split_idx = 0; split_idx < 4; split_idx++)
        node->split[split_idx] = &vt->split[split_idx].part_variances.none;
      break;
    }
    case BLOCK_64X64: {
      VP64x64 *vt = (VP64x64 *)data;
      node->part_variances = &vt->part_variances;
      for (int split_idx = 0; split_idx < 4; split_idx++)
        node->split[split_idx] = &vt->split[split_idx].part_variances.none;
      break;
    }
    case BLOCK_32X32: {
      VP32x32 *vt = (VP32x32 *)data;
      node->part_variances = &vt->part_variances;
      for (int split_idx = 0; split_idx < 4; split_idx++)
        node->split[split_idx] = &vt->split[split_idx].part_variances.none;
      break;
    }
    case BLOCK_16X16: {
      VP16x16 *vt = (VP16x16 *)data;
      node->part_variances = &vt->part_variances;
      for (int split_idx = 0; split_idx < 4; split_idx++)
        node->split[split_idx] = &vt->split[split_idx].part_variances.none;
      break;
    }
    case BLOCK_8X8: {
      VP8x8 *vt = (VP8x8 *)data;
      node->part_variances = &vt->part_variances;
      for (int split_idx = 0; split_idx < 4; split_idx++)
        node->split[split_idx] = &vt->split[split_idx].part_variances.none;
      break;
    }
    default: {
      VP4x4 *vt = (VP4x4 *)data;
      assert(bsize == BLOCK_4X4);
      node->part_variances = &vt->part_variances;
      for (int split_idx = 0; split_idx < 4; split_idx++)
        node->split[split_idx] = &vt->split[split_idx];
      break;
    }
  }
}

// Set variance values given sum square error, sum error, count.
static inline void fill_variance(uint32_t s2, int32_t s, int c, VPartVar *v) {
  v->sum_square_error = s2;
  v->sum_error = s;
  v->log2_count = c;
}

static inline void get_variance(VPartVar *v) {
  v->variance =
      (int)(256 * (v->sum_square_error -
                   (uint32_t)(((int64_t)v->sum_error * v->sum_error) >>
                              v->log2_count)) >>
            v->log2_count);
}

static inline void sum_2_variances(const VPartVar *a, const VPartVar *b,
                                   VPartVar *r) {
  assert(a->log2_count == b->log2_count);
  fill_variance(a->sum_square_error + b->sum_square_error,
                a->sum_error + b->sum_error, a->log2_count + 1, r);
}

static inline void fill_variance_tree(void *data, BLOCK_SIZE bsize) {
  variance_node node;
  memset(&node, 0, sizeof(node));
  tree_to_node(data, bsize, &node);
  sum_2_variances(node.split[0], node.split[1], &node.part_variances->horz[0]);
  sum_2_variances(node.split[2], node.split[3], &node.part_variances->horz[1]);
  sum_2_variances(node.split[0], node.split[2], &node.part_variances->vert[0]);
  sum_2_variances(node.split[1], node.split[3], &node.part_variances->vert[1]);
  sum_2_variances(&node.part_variances->vert[0], &node.part_variances->vert[1],
                  &node.part_variances->none);
}

static inline void set_block_size(AV2_COMP *const cpi, int mi_row, int mi_col,
                                  BLOCK_SIZE bsize) {
  if (cpi->common.mi_params.mi_cols > mi_col &&
      cpi->common.mi_params.mi_rows > mi_row) {
    CommonModeInfoParams *mi_params = &cpi->common.mi_params;
    const int mi_grid_idx = get_mi_grid_idx(mi_params, mi_row, mi_col);
    const int mi_alloc_idx = get_alloc_mi_idx(mi_params, mi_row, mi_col);
    MB_MODE_INFO *mi = mi_params->mi_grid_base[mi_grid_idx] =
        &mi_params->mi_alloc[mi_alloc_idx];
    mi->sb_type[PLANE_TYPE_Y] = bsize;
    mi->sb_type[PLANE_TYPE_UV] = bsize;
  }
}

static int set_vt_partitioning(AV2_COMP *cpi, MACROBLOCKD *const xd,
                               const TileInfo *const tile, void *data,
                               BLOCK_SIZE bsize, int mi_row, int mi_col,
                               int64_t threshold, BLOCK_SIZE bsize_min,
                               PART_EVAL_STATUS force_split) {
  AV2_COMMON *const cm = &cpi->common;
  // Use only square for now.
  const int check_horiz_split = false;
  const int check_vert_split = false;
  variance_node vt;
  const int block_width = mi_size_wide[bsize];
  const int block_height = mi_size_high[bsize];
  int bs_width_check = block_width;
  int bs_height_check = block_height;
  int bs_width_vert_check = block_width >> 1;
  int bs_height_horiz_check = block_height >> 1;
  // On the right and bottom boundary we only need to check
  // if half the bsize fits, because boundary is extended
  // up to 64. So do this check only for sb_size = 64X64.
  if (cm->seq_params.sb_size == BLOCK_64X64) {
    if (tile->mi_col_end == cm->mi_params.mi_cols) {
      bs_width_check = (block_width >> 1) + 1;
      bs_width_vert_check = (block_width >> 2) + 1;
    }
    if (tile->mi_row_end == cm->mi_params.mi_rows) {
      bs_height_check = (block_height >> 1) + 1;
      bs_height_horiz_check = (block_height >> 2) + 1;
    }
  }

  assert(block_height == block_width);
  tree_to_node(data, bsize, &vt);

  if (mi_col + bs_width_check <= tile->mi_col_end &&
      mi_row + bs_height_check <= tile->mi_row_end &&
      force_split == PART_EVAL_ONLY_NONE) {
    set_block_size(cpi, mi_row, mi_col, bsize);
    return 1;
  }
  if (force_split == PART_EVAL_ONLY_SPLIT) return 0;

  // For bsize=bsize_min (16x16/8x8 for 8x8/4x4 downsampling), select if
  // variance is below threshold, otherwise split will be selected.
  // No check for vert/horiz split as too few samples for variance.
  if (bsize == bsize_min) {
    // Variance already computed to set the force_split.
    if (frame_is_intra_only(cm)) get_variance(&vt.part_variances->none);
    if (mi_col + bs_width_check <= tile->mi_col_end &&
        mi_row + bs_height_check <= tile->mi_row_end &&
        vt.part_variances->none.variance < threshold) {
      set_block_size(cpi, mi_row, mi_col, bsize);
      return 1;
    }
    return 0;
  } else if (bsize > bsize_min) {
    // Variance already computed to set the force_split.
    if (frame_is_intra_only(cm)) get_variance(&vt.part_variances->none);
    // For key frame: take split for bsize above 32X32 or very high variance.
    if (frame_is_intra_only(cm) &&
        (bsize > BLOCK_32X32 ||
         vt.part_variances->none.variance > (threshold << 4))) {
      return 0;
    }
    // If variance is low, take the bsize (no split).
    if (mi_col + bs_width_check <= tile->mi_col_end &&
        mi_row + bs_height_check <= tile->mi_row_end &&
        vt.part_variances->none.variance < threshold) {
      set_block_size(cpi, mi_row, mi_col, bsize);
      return 1;
    }
    // Check vertical split.
    if (check_vert_split && mi_row + bs_height_check <= tile->mi_row_end &&
        mi_col + bs_width_vert_check <= tile->mi_col_end) {
      BLOCK_SIZE subsize = get_partition_subsize(bsize, PARTITION_VERT);
      BLOCK_SIZE plane_bsize =
          get_plane_block_size(subsize, xd->plane[AVM_PLANE_U].subsampling_x,
                               xd->plane[AVM_PLANE_U].subsampling_y);
      get_variance(&vt.part_variances->vert[0]);
      get_variance(&vt.part_variances->vert[1]);
      if (vt.part_variances->vert[0].variance < threshold &&
          vt.part_variances->vert[1].variance < threshold &&
          plane_bsize < BLOCK_INVALID) {
        set_block_size(cpi, mi_row, mi_col, subsize);
        set_block_size(cpi, mi_row, mi_col + block_width / 2, subsize);
        return 1;
      }
    }
    // Check horizontal split.
    if (check_horiz_split && mi_col + bs_width_check <= tile->mi_col_end &&
        mi_row + bs_height_horiz_check <= tile->mi_row_end) {
      BLOCK_SIZE subsize = get_partition_subsize(bsize, PARTITION_HORZ);
      BLOCK_SIZE plane_bsize =
          get_plane_block_size(subsize, xd->plane[AVM_PLANE_U].subsampling_x,
                               xd->plane[AVM_PLANE_U].subsampling_y);
      get_variance(&vt.part_variances->horz[0]);
      get_variance(&vt.part_variances->horz[1]);
      if (vt.part_variances->horz[0].variance < threshold &&
          vt.part_variances->horz[1].variance < threshold &&
          plane_bsize < BLOCK_INVALID) {
        set_block_size(cpi, mi_row, mi_col, subsize);
        set_block_size(cpi, mi_row + block_height / 2, mi_col, subsize);
        return 1;
      }
    }
    return 0;
  }
  return 0;
}

static inline void fill_variance_8x8avg_highbd(
    const uint16_t *src_buf, int src_stride, const uint16_t *dst_buf,
    int dst_stride, int x16_idx, int y16_idx, VP16x16 *vst, int pixels_wide,
    int pixels_high) {
  for (int idx = 0; idx < 4; idx++) {
    const int x8_idx = x16_idx + GET_BLK_IDX_X(idx, 3);
    const int y8_idx = y16_idx + GET_BLK_IDX_Y(idx, 3);
    unsigned int sse = 0;
    int sum = 0;
    if (x8_idx < pixels_wide && y8_idx < pixels_high) {
      int src_avg = avm_highbd_avg_8x8(src_buf + y8_idx * src_stride + x8_idx,
                                       src_stride);
      int dst_avg = avm_highbd_avg_8x8(dst_buf + y8_idx * dst_stride + x8_idx,
                                       dst_stride);

      sum = src_avg - dst_avg;
      sse = sum * sum;
    }
    fill_variance(sse, sum, 0, &vst->split[idx].part_variances.none);
  }
}

// Obtain parameters required to calculate variance (such as sum, sse, etc,.)
// at 8x8 sub-block level for a given 16x16 block.
// The function can be called only when is_key_frame is false since sum is
// computed between source and reference frames.
static inline void fill_variance_8x8avg(const uint16_t *src_buf, int src_stride,
                                        const uint16_t *dst_buf, int dst_stride,
                                        int x16_idx, int y16_idx, VP16x16 *vst,
                                        int pixels_wide, int pixels_high) {
  fill_variance_8x8avg_highbd(src_buf, src_stride, dst_buf, dst_stride, x16_idx,
                              y16_idx, vst, pixels_wide, pixels_high);
}

// Function to compute average and variance of 4x4 sub-block.
// The function can be called only when is_key_frame is true since sum is
// computed using source frame only.
static inline void fill_variance_4x4avg(const uint16_t *src_buf, int src_stride,
                                        int x8_idx, int y8_idx, VP8x8 *vst,
                                        int pixels_wide, int pixels_high,
                                        int border_offset_4x4) {
  for (int idx = 0; idx < 4; idx++) {
    const int x4_idx = x8_idx + GET_BLK_IDX_X(idx, 2);
    const int y4_idx = y8_idx + GET_BLK_IDX_Y(idx, 2);
    unsigned int sse = 0;
    int sum = 0;
    if (x4_idx < pixels_wide - border_offset_4x4 &&
        y4_idx < pixels_high - border_offset_4x4) {
      int src_avg;
      int dst_avg = 128;
      src_avg = avm_highbd_avg_4x4(src_buf + y4_idx * src_stride + x4_idx,
                                   src_stride);
      sum = src_avg - dst_avg;
      sse = sum * sum;
    }
    fill_variance(sse, sum, 0, &vst->split[idx].part_variances.none);
  }
}

static inline void tune_thresh_based_on_resolution(AV2_COMP *cpi,
                                                   int64_t thresholds[],
                                                   int64_t threshold_base,
                                                   int current_qindex,
                                                   int num_pixels) {
  if (num_pixels >= RESOLUTION_720P) thresholds[3] = thresholds[3] << 1;
  if (num_pixels <= RESOLUTION_288P) {
    const int qindex_thr[5][2] = {
      { 200, 220 }, { 140, 170 }, { 120, 150 }, { 200, 210 }, { 170, 220 },
    };
    int th_idx = 0;
    const int qindex_low_thr = qindex_thr[th_idx][0];
    const int qindex_high_thr = qindex_thr[th_idx][1];
    if (current_qindex >= qindex_high_thr) {
      threshold_base = (5 * threshold_base) >> 1;
      thresholds[1] = threshold_base >> 3;
      thresholds[2] = threshold_base << 2;
      thresholds[3] = threshold_base << 5;
    } else if (current_qindex < qindex_low_thr) {
      thresholds[1] = threshold_base >> 3;
      thresholds[2] = threshold_base >> 1;
      thresholds[3] = threshold_base << 3;
    } else {
      int64_t qi_diff_low = current_qindex - qindex_low_thr;
      int64_t qi_diff_high = qindex_high_thr - current_qindex;
      int64_t threshold_diff = qindex_high_thr - qindex_low_thr;
      int64_t threshold_base_high = (5 * threshold_base) >> 1;

      threshold_diff = threshold_diff > 0 ? threshold_diff : 1;
      threshold_base =
          (qi_diff_low * threshold_base_high + qi_diff_high * threshold_base) /
          threshold_diff;
      thresholds[1] = threshold_base >> 3;
      thresholds[2] = ((qi_diff_low * threshold_base) +
                       qi_diff_high * (threshold_base >> 1)) /
                      threshold_diff;
      thresholds[3] = ((qi_diff_low * (threshold_base << 5)) +
                       qi_diff_high * (threshold_base << 3)) /
                      threshold_diff;
    }
  } else if (num_pixels < RESOLUTION_720P) {
    thresholds[2] = (3 * threshold_base) >> 1;
  } else if (num_pixels < RESOLUTION_1080P) {
    thresholds[2] = threshold_base << 1;
  } else {
    // num_pixels >= RESOLUTION_1080P
    if (cpi->oxcf.tune_cfg.content == AVM_CONTENT_SCREEN) {
      if (num_pixels < RESOLUTION_1440P) {
        thresholds[2] = (5 * threshold_base) >> 1;
      } else {
        thresholds[2] = (7 * threshold_base) >> 1;
      }
    } else {
      if (cpi->oxcf.speed > 7) {
        thresholds[2] = threshold_base << 2;
      }
    }
  }
}

static void set_vbp_thresholds_key_frame(int64_t thresholds[],
                                         int64_t threshold_base,
                                         int num_pixels) {
  thresholds[0] = threshold_base;
  thresholds[1] = threshold_base;
  if (num_pixels < RESOLUTION_720P) {
    thresholds[2] = threshold_base / 3;
    thresholds[3] = threshold_base >> 1;
  } else {
    int shift_val = 2;
    thresholds[2] = threshold_base >> shift_val;
    thresholds[3] = threshold_base >> shift_val;
  }
  thresholds[4] = threshold_base << 2;
}

static inline void set_vbp_thresholds(AV2_COMP *cpi, int64_t thresholds[],
                                      int qindex) {
  AV2_COMMON *const cm = &cpi->common;
  const int is_key_frame = frame_is_intra_only(cm);
  const int threshold_multiplier = is_key_frame ? 120 : 1;
  const int ac_q = av2_ac_quant_QTX(qindex, 0, 0, cm->seq_params.bit_depth);
  int64_t threshold_base = (int64_t)(threshold_multiplier * ac_q) >> 3;
  const int current_qindex = cm->quant_params.base_qindex;
  const int threshold_left_shift = 7;
  const int num_pixels = cm->width * cm->height;

  if (is_key_frame) {
    set_vbp_thresholds_key_frame(thresholds, threshold_base, num_pixels);
    return;
  }

  thresholds[0] = threshold_base >> 1;
  thresholds[1] = threshold_base;
  thresholds[2] = threshold_base << 2;
  thresholds[3] = threshold_base << threshold_left_shift;
  thresholds[4] = INT64_MAX;

  tune_thresh_based_on_resolution(cpi, thresholds, threshold_base,
                                  current_qindex, num_pixels);

  // TODO(any, marpan): add adjustments/tunings based on QP and content.
}

static void fill_variance_tree_leaves(
    MACROBLOCK *x, VP128x128 *vt, PART_EVAL_STATUS *force_split,
    int avg_16x16[][4], int maxvar_16x16[][4], int minvar_16x16[][4],
    int64_t *thresholds, const uint16_t *src_buf, int src_stride,
    const uint16_t *dst_buf, int dst_stride, bool is_key_frame,
    const bool is_small_sb) {
  MACROBLOCKD *xd = &x->e_mbd;
  const int num_64x64_blocks = is_small_sb ? 1 : 4;
  int pixels_wide = 128, pixels_high = 128;
  int border_offset_4x4 = 0;
  // dst_buf pointer is not used for is_key_frame, so it should be NULL.
  assert(IMPLIES(is_key_frame, dst_buf == NULL));
  if (is_small_sb) {
    pixels_wide = 64;
    pixels_high = 64;
  }
  if (xd->mb_to_right_edge < 0) pixels_wide += (xd->mb_to_right_edge >> 3);
  if (xd->mb_to_bottom_edge < 0) pixels_high += (xd->mb_to_bottom_edge >> 3);
  for (int blk64_idx = 0; blk64_idx < num_64x64_blocks; blk64_idx++) {
    const int x64_idx = GET_BLK_IDX_X(blk64_idx, 6);
    const int y64_idx = GET_BLK_IDX_Y(blk64_idx, 6);
    const int blk64_scale_idx = blk64_idx << 2;
    force_split[blk64_idx + 1] = PART_EVAL_ALL;

    for (int lvl1_idx = 0; lvl1_idx < 4; lvl1_idx++) {
      const int x32_idx = x64_idx + GET_BLK_IDX_X(lvl1_idx, 5);
      const int y32_idx = y64_idx + GET_BLK_IDX_Y(lvl1_idx, 5);
      const int lvl1_scale_idx = (blk64_scale_idx + lvl1_idx) << 2;
      force_split[5 + blk64_scale_idx + lvl1_idx] = PART_EVAL_ALL;
      avg_16x16[blk64_idx][lvl1_idx] = 0;
      maxvar_16x16[blk64_idx][lvl1_idx] = 0;
      minvar_16x16[blk64_idx][lvl1_idx] = INT_MAX;
      for (int lvl2_idx = 0; lvl2_idx < 4; lvl2_idx++) {
        const int x16_idx = x32_idx + GET_BLK_IDX_X(lvl2_idx, 4);
        const int y16_idx = y32_idx + GET_BLK_IDX_Y(lvl2_idx, 4);
        const int split_index = 21 + lvl1_scale_idx + lvl2_idx;
        VP16x16 *vst = &vt->split[blk64_idx].split[lvl1_idx].split[lvl2_idx];
        force_split[split_index] = PART_EVAL_ALL;
        if (is_key_frame) {
          // Go down to 4x4 down-sampling for variance.
          for (int lvl3_idx = 0; lvl3_idx < 4; lvl3_idx++) {
            const int x8_idx = x16_idx + GET_BLK_IDX_X(lvl3_idx, 3);
            const int y8_idx = y16_idx + GET_BLK_IDX_Y(lvl3_idx, 3);
            VP8x8 *vst2 = &vst->split[lvl3_idx];
            fill_variance_4x4avg(src_buf, src_stride, x8_idx, y8_idx, vst2,
                                 pixels_wide, pixels_high, border_offset_4x4);
          }
        } else {
          fill_variance_8x8avg(src_buf, src_stride, dst_buf, dst_stride,
                               x16_idx, y16_idx, vst, pixels_wide, pixels_high);

          fill_variance_tree(vst, BLOCK_16X16);
          VPartVar *none_var = &vt->split[blk64_idx]
                                    .split[lvl1_idx]
                                    .split[lvl2_idx]
                                    .part_variances.none;
          get_variance(none_var);
          const int val_none_var = none_var->variance;
          avg_16x16[blk64_idx][lvl1_idx] += val_none_var;
          minvar_16x16[blk64_idx][lvl1_idx] =
              AVMMIN(minvar_16x16[blk64_idx][lvl1_idx], val_none_var);
          maxvar_16x16[blk64_idx][lvl1_idx] =
              AVMMAX(maxvar_16x16[blk64_idx][lvl1_idx], val_none_var);
          if (val_none_var > thresholds[3]) {
            // 16X16 variance is above threshold for split, so force split to
            // 8x8 for this 16x16 block (this also forces splits for upper
            // levels).
            force_split[split_index] = PART_EVAL_ONLY_SPLIT;
            force_split[5 + blk64_scale_idx + lvl1_idx] = PART_EVAL_ONLY_SPLIT;
            force_split[blk64_idx + 1] = PART_EVAL_ONLY_SPLIT;
            force_split[0] = PART_EVAL_ONLY_SPLIT;
          }
        }
      }
    }
  }
}

static void setup_planes(AV2_COMP *cpi, MACROBLOCK *x, unsigned int *y_sad,
                         int mi_row, int mi_col, bool is_small_sb,
                         bool scaled_ref_last) {
  AV2_COMMON *const cm = &cpi->common;
  MACROBLOCKD *xd = &x->e_mbd;
  const int num_planes = av2_num_planes(cm);
  BLOCK_SIZE bsize = is_small_sb ? BLOCK_64X64 : BLOCK_128X128;
  MB_MODE_INFO *mi = xd->mi[0];
  const MV_REFERENCE_FRAME last_frame = get_closest_pastcur_ref_or_ref0(cm);
  const YV12_BUFFER_CONFIG *yv12 = get_ref_frame_yv12_buf(cm, last_frame);
  assert(yv12 != NULL);
  if (yv12 != NULL) {
    av2_setup_pre_planes(
        xd, 0, yv12, mi_row, mi_col,
        scaled_ref_last ? NULL : get_ref_scale_factors(cm, last_frame),
        num_planes, NULL);
    mi->ref_frame[0] = last_frame;
    mi->ref_frame[1] = NONE_FRAME;
    mi->sb_type[PLANE_TYPE_Y] = cm->seq_params.sb_size;
    mi->sb_type[PLANE_TYPE_UV] = cm->seq_params.sb_size;
    mi->mv[0].as_int = 0;

    if (*y_sad == UINT_MAX) {
      *y_sad = cpi->fn_ptr[bsize].sdf(x->plane[AVM_PLANE_Y].src.buf,
                                      x->plane[AVM_PLANE_Y].src.stride,
                                      xd->plane[AVM_PLANE_Y].pre[0].buf,
                                      xd->plane[AVM_PLANE_Y].pre[0].stride);
    }
  }
  // TODO(any, marpan): consider other references.

  // TODO(any, marpan): add superblock motion estimation.

  // Only calculate the predictor for non-zero MV.
  if (mi->mv[0].as_int != 0) {
    set_ref_ptrs(cm, xd, mi->ref_frame[0], mi->ref_frame[1]);
    av2_enc_build_inter_predictor(cm, xd, mi_row, mi_col, NULL,
                                  cm->seq_params.sb_size, AVM_PLANE_Y,
                                  num_planes - 1);
  }
}

int av1_choose_var_based_partitioning(AV2_COMP *cpi, const TileInfo *const tile,
                                      MACROBLOCK *x, int mi_row, int mi_col) {
  AV2_COMMON *const cm = &cpi->common;
  MACROBLOCKD *xd = &x->e_mbd;
  const int64_t *const vbp_thresholds = cpi->vbp_info.thresholds;
  PART_EVAL_STATUS force_split[85];
  int avg_64x64;
  int max_var_32x32[4];
  int min_var_32x32[4];
  int var_32x32;
  int var_64x64;
  int min_var_64x64 = INT_MAX;
  int max_var_64x64 = 0;
  int avg_16x16[4][4];
  int maxvar_16x16[4][4];
  int minvar_16x16[4][4];
  const uint16_t *src_buf;
  const uint16_t *dst_buf;
  int dst_stride;
  bool is_key_frame = frame_is_intra_only(cm);
  bool scaled_ref_last = false;
  const int is_360p_or_smaller = cm->width * cm->height <= RESOLUTION_360P;

  assert(cm->seq_params.sb_size == BLOCK_64X64 ||
         cm->seq_params.sb_size == BLOCK_128X128);
  const bool is_small_sb = (cm->seq_params.sb_size == BLOCK_64X64);
  const int num_64x64_blocks = is_small_sb ? 1 : 4;

  unsigned int y_sad = UINT_MAX;

  int64_t thresholds[5] = { vbp_thresholds[0], vbp_thresholds[1],
                            vbp_thresholds[2], vbp_thresholds[3],
                            vbp_thresholds[4] };

  const int qindex = cm->quant_params.base_qindex;

  set_vbp_thresholds(cpi, thresholds, qindex);

  src_buf = x->plane[AVM_PLANE_Y].src.buf;
  int src_stride = x->plane[AVM_PLANE_Y].src.stride;

  // Index for force_split: 0 for 64x64, 1-4 for 32x32 blocks,
  // 5-20 for the 16x16 blocks.
  force_split[0] = PART_EVAL_ALL;
  memset(x->part_search_info.variance_low, 0,
         sizeof(x->part_search_info.variance_low));

  if (!is_key_frame) {
    setup_planes(cpi, x, &y_sad, mi_row, mi_col, is_small_sb, scaled_ref_last);

    MB_MODE_INFO *mi = xd->mi[0];
    // Use reference SB directly for zero mv.
    if (mi->mv[0].as_int != 0) {
      dst_buf = xd->plane[AVM_PLANE_Y].dst.buf;
      dst_stride = xd->plane[AVM_PLANE_Y].dst.stride;
    } else {
      dst_buf = xd->plane[AVM_PLANE_Y].pre[0].buf;
      dst_stride = xd->plane[AVM_PLANE_Y].pre[0].stride;
    }
  } else {
    dst_buf = NULL;
    dst_stride = 0;
  }

  // TODO(any, marpan): add chroma check/adjustments.

  // TDOD(any, marpan): check for early exits.

  VP128x128 *vt;
  VP64x64 *vt64x64;
  AVM_CHECK_MEM_ERROR(xd->error_info, vt, avm_malloc(sizeof(*vt)));
  AVM_CHECK_MEM_ERROR(xd->error_info, vt64x64,
                      avm_malloc(sizeof(*vt64x64) * num_64x64_blocks));
  vt->split = vt64x64;

  // Fill in the entire tree of 8x8 (for inter frames) or 4x4 (for key frames)
  // variances for splits.
  fill_variance_tree_leaves(x, vt, force_split, avg_16x16, maxvar_16x16,
                            minvar_16x16, thresholds, src_buf, src_stride,
                            dst_buf, dst_stride, is_key_frame, is_small_sb);

  avg_64x64 = 0;
  for (int blk64_idx = 0; blk64_idx < num_64x64_blocks; ++blk64_idx) {
    max_var_32x32[blk64_idx] = 0;
    min_var_32x32[blk64_idx] = INT_MAX;
    const int blk64_scale_idx = blk64_idx << 2;
    for (int lvl1_idx = 0; lvl1_idx < 4; lvl1_idx++) {
      const int lvl1_scale_idx = (blk64_scale_idx + lvl1_idx) << 2;
      for (int lvl2_idx = 0; lvl2_idx < 4; lvl2_idx++) {
        if (!is_key_frame) continue;
        VP16x16 *vtemp = &vt->split[blk64_idx].split[lvl1_idx].split[lvl2_idx];
        for (int lvl3_idx = 0; lvl3_idx < 4; lvl3_idx++)
          fill_variance_tree(&vtemp->split[lvl3_idx], BLOCK_8X8);
        fill_variance_tree(vtemp, BLOCK_16X16);
        // If variance of this 16x16 block is above the threshold, force block
        // to split. This also forces a split on the upper levels.
        get_variance(&vtemp->part_variances.none);
        if (vtemp->part_variances.none.variance > thresholds[3]) {
          const int split_index = 21 + lvl1_scale_idx + lvl2_idx;
          force_split[split_index] = PART_EVAL_ONLY_SPLIT;
          force_split[5 + blk64_scale_idx + lvl1_idx] = PART_EVAL_ONLY_SPLIT;
          force_split[blk64_idx + 1] = PART_EVAL_ONLY_SPLIT;
          force_split[0] = PART_EVAL_ONLY_SPLIT;
        }
      }
      fill_variance_tree(&vt->split[blk64_idx].split[lvl1_idx], BLOCK_32X32);
      // If variance of this 32x32 block is above the threshold, or if its above
      // (some threshold of) the average variance over the sub-16x16 blocks,
      // then force this block to split. This also forces a split on the upper
      // (64x64) level.
      if (force_split[5 + blk64_scale_idx + lvl1_idx] == PART_EVAL_ALL) {
        get_variance(&vt->split[blk64_idx].split[lvl1_idx].part_variances.none);
        var_32x32 =
            vt->split[blk64_idx].split[lvl1_idx].part_variances.none.variance;
        max_var_32x32[blk64_idx] = AVMMAX(var_32x32, max_var_32x32[blk64_idx]);
        min_var_32x32[blk64_idx] = AVMMIN(var_32x32, min_var_32x32[blk64_idx]);
        const int max_min_var_16X16_diff = (maxvar_16x16[blk64_idx][lvl1_idx] -
                                            minvar_16x16[blk64_idx][lvl1_idx]);

        if (var_32x32 > thresholds[2] ||
            (!is_key_frame && var_32x32 > (thresholds[2] >> 1) &&
             var_32x32 > (avg_16x16[blk64_idx][lvl1_idx] >> 1))) {
          force_split[5 + blk64_scale_idx + lvl1_idx] = PART_EVAL_ONLY_SPLIT;
          force_split[blk64_idx + 1] = PART_EVAL_ONLY_SPLIT;
          force_split[0] = PART_EVAL_ONLY_SPLIT;
        } else if (!is_key_frame && is_360p_or_smaller &&
                   ((max_min_var_16X16_diff > (thresholds[2] >> 1) &&
                     maxvar_16x16[blk64_idx][lvl1_idx] > thresholds[2]) ||
                    (maxvar_16x16[blk64_idx][lvl1_idx] > (thresholds[2] >> 4) &&
                     maxvar_16x16[blk64_idx][lvl1_idx] >
                         (minvar_16x16[blk64_idx][lvl1_idx] << 2)))) {
          force_split[5 + blk64_scale_idx + lvl1_idx] = PART_EVAL_ONLY_SPLIT;
          force_split[blk64_idx + 1] = PART_EVAL_ONLY_SPLIT;
          force_split[0] = PART_EVAL_ONLY_SPLIT;
        }
      }
    }
    if (force_split[1 + blk64_idx] == PART_EVAL_ALL) {
      fill_variance_tree(&vt->split[blk64_idx], BLOCK_64X64);
      get_variance(&vt->split[blk64_idx].part_variances.none);
      var_64x64 = vt->split[blk64_idx].part_variances.none.variance;
      max_var_64x64 = AVMMAX(var_64x64, max_var_64x64);
      min_var_64x64 = AVMMIN(var_64x64, min_var_64x64);
      // If the difference of the max-min variances of sub-blocks or max
      // variance of a sub-block is above some threshold of then force this
      // block to split. Only checking this for noise level >= medium, if
      // encoder is in SVC or if we already forced large blocks.
      const int max_min_var_32x32_diff =
          max_var_32x32[blk64_idx] - min_var_32x32[blk64_idx];
      const int64_t set_threshold = 3 * (thresholds[1] >> 3);

      if (!is_key_frame && max_min_var_32x32_diff > set_threshold) {
        force_split[1 + blk64_idx] = PART_EVAL_ONLY_SPLIT;
        force_split[0] = PART_EVAL_ONLY_SPLIT;
      }
      avg_64x64 += var_64x64;
    }
    if (is_small_sb) force_split[0] = PART_EVAL_ONLY_SPLIT;
  }

  if (force_split[0] == PART_EVAL_ALL) {
    fill_variance_tree(vt, BLOCK_128X128);
    get_variance(&vt->part_variances.none);
    const int set_avg_64x64 = (9 * avg_64x64) >> 5;
    if (!is_key_frame && vt->part_variances.none.variance > set_avg_64x64)
      force_split[0] = PART_EVAL_ONLY_SPLIT;

    if (!is_key_frame &&
        (max_var_64x64 - min_var_64x64) > 3 * (thresholds[0] >> 3) &&
        max_var_64x64 > thresholds[0] >> 1)
      force_split[0] = PART_EVAL_ONLY_SPLIT;
  }

  if (mi_col + 32 > tile->mi_col_end || mi_row + 32 > tile->mi_row_end ||
      !set_vt_partitioning(cpi, xd, tile, vt, BLOCK_128X128, mi_row, mi_col,
                           thresholds[0], BLOCK_16X16, force_split[0])) {
    for (int blk64_idx = 0; blk64_idx < num_64x64_blocks; ++blk64_idx) {
      const int x64_idx = GET_BLK_IDX_X(blk64_idx, 4);
      const int y64_idx = GET_BLK_IDX_Y(blk64_idx, 4);
      const int blk64_scale_idx = blk64_idx << 2;

      // Now go through the entire structure, splitting every block size until
      // we get to one that's got a variance lower than our threshold.
      if (set_vt_partitioning(cpi, xd, tile, &vt->split[blk64_idx], BLOCK_64X64,
                              mi_row + y64_idx, mi_col + x64_idx, thresholds[1],
                              BLOCK_16X16, force_split[1 + blk64_idx]))
        continue;
      for (int lvl1_idx = 0; lvl1_idx < 4; ++lvl1_idx) {
        const int x32_idx = GET_BLK_IDX_X(lvl1_idx, 3);
        const int y32_idx = GET_BLK_IDX_Y(lvl1_idx, 3);
        const int lvl1_scale_idx = (blk64_scale_idx + lvl1_idx) << 2;
        if (set_vt_partitioning(
                cpi, xd, tile, &vt->split[blk64_idx].split[lvl1_idx],
                BLOCK_32X32, (mi_row + y64_idx + y32_idx),
                (mi_col + x64_idx + x32_idx), thresholds[2], BLOCK_16X16,
                force_split[5 + blk64_scale_idx + lvl1_idx]))
          continue;
        for (int lvl2_idx = 0; lvl2_idx < 4; ++lvl2_idx) {
          const int x16_idx = GET_BLK_IDX_X(lvl2_idx, 2);
          const int y16_idx = GET_BLK_IDX_Y(lvl2_idx, 2);
          const int split_index = 21 + lvl1_scale_idx + lvl2_idx;
          VP16x16 *vtemp =
              &vt->split[blk64_idx].split[lvl1_idx].split[lvl2_idx];
          if (set_vt_partitioning(cpi, xd, tile, vtemp, BLOCK_16X16,
                                  mi_row + y64_idx + y32_idx + y16_idx,
                                  mi_col + x64_idx + x32_idx + x16_idx,
                                  thresholds[3], BLOCK_8X8,
                                  force_split[split_index]))
            continue;
          for (int lvl3_idx = 0; lvl3_idx < 4; ++lvl3_idx) {
            const int x8_idx = GET_BLK_IDX_X(lvl3_idx, 1);
            const int y8_idx = GET_BLK_IDX_Y(lvl3_idx, 1);
            set_block_size(cpi, (mi_row + y64_idx + y32_idx + y16_idx + y8_idx),
                           (mi_col + x64_idx + x32_idx + x16_idx + x8_idx),
                           BLOCK_8X8);
          }
        }
      }
    }
  }
  avm_free(vt);
  avm_free(vt64x64);
  return 0;
}
