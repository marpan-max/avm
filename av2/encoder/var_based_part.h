/*
 * Copyright (c) 2016, Alliance for Open Media. All rights reserved.
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#ifndef AOM_av2_encODER_VAR_BASED_PART_H_
#define AOM_av2_encODER_VAR_BASED_PART_H_

#include <stdio.h>

#include "config/avm_config.h"
#include "config/avm_dsp_rtcd.h"
#include "config/av2_rtcd.h"

#include "av2/encoder/encoder.h"

// Calculate block index x and y from split level and index
#define GET_BLK_IDX_X(idx, level) (((idx) & (0x01)) << (level))
#define GET_BLK_IDX_Y(idx, level) (((idx) >> (0x01)) << (level))

#ifdef __cplusplus
extern "C" {
#endif

/*!\brief Variance based partition selection.
 *
 * Select the partitioning based on the variance of the residual signal,
 * residual generated as the difference between the source and prediction.
 * The prediction is the reconstructed LAST or reconstructed GOLDEN, whichever
 * has lower y sad. For LAST, option exists (speed feature) to use motion
 * compensation based on superblock motion via int_pro_motion_estimation. For
 * key frames reference is fixed 128 level, so variance is the source variance.
 * The variance is computed for downsampled inputs (8x8 or 4x4 downsampled),
 * and selection is done top-down via as set of partition thresholds. defined
 * for each block level, and set based on Q, resolution, noise level, and
 * content state.
 *
 * \ingroup variance_partition
 * \callgraph
 * \callergraph
 *
 * \param[in]       cpi          Top level encoder structure
 * \param[in]       tile         Pointer to TileInfo
 * \param[in]       td           Pointer to ThreadData
 * \param[in]       x            Pointer to MACROBLOCK
 * \param[in]       mi_row       Row coordinate of the superblock in a step
 size of MI_SIZE
 * \param[in]       mi_col       Column coordinate of the super block in a step
 size of MI_SIZE
 *
 * \return Returns the partition in \c xd->mi[0]->sb_type. Also sets the low
 * temporal variance flag and the color sensitivity flag (both used in
 * nonrd_pickmode).
 */
int av1_choose_var_based_partitioning(AV2_COMP *cpi, const TileInfo *const tile,
                                      MACROBLOCK *x, int mi_row, int mi_col);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AOM_av2_encODER_VAR_BASED_PART_H_
