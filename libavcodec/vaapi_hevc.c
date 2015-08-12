/*
 * H.264 HW decode acceleration through VA API
 *
 * Copyright (C) 2015 XBMC Foundation
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "vaapi_internal.h"
#include "hevc.h"
#include "mpegutils.h"

/**
 * @file
 * This file implements the glue code between FFmpeg's and VA API's
 * structures for HEVC decoding.
 */

/**
 * Initialize an empty VA API picture.
 *
 * VA API requires a fixed-size reference picture array.
 */
static void init_vaapi_pic(VAPictureHEVC *va_pic)
{
    va_pic->picture_id = VA_INVALID_ID;
    va_pic->flags = VA_PICTURE_HEVC_INVALID;
    va_pic->pic_order_cnt = 0;
}

/**
 * Translate an FFmpeg Picture into its VA API form.
 *
 * @param[out] va_pic          A pointer to VA API's own picture struct
 * @param[in]  pic             A pointer to the FFmpeg picture struct to convert
 * @param[in]  pic_structure   The picture field type (as defined in mpegvideo.h),
 *                             supersedes pic's field type if nonzero.
 */
static void fill_vaapi_pic(VAPictureHEVC *va_pic,
                           HEVCFrame     *pic,
                           int            pic_structure)
{
    pic_structure &= PICT_FRAME; /* PICT_TOP_FIELD|PICT_BOTTOM_FIELD */

    va_pic->picture_id = ff_vaapi_get_surface_id(pic->frame);

    va_pic->flags = 0;
    if (pic_structure != PICT_FRAME) {
        va_pic->flags |= VA_PICTURE_HEVC_FIELD_PIC;
        va_pic->flags |= (pic_structure & PICT_TOP_FIELD) ? 0 : VA_PICTURE_HEVC_BOTTOM_FIELD;
    }
    if (pic->flags & HEVC_FRAME_FLAG_LONG_REF)
        va_pic->flags |= VA_PICTURE_HEVC_LONG_TERM_REFERENCE;

    va_pic->pic_order_cnt = pic->poc;
}

static void fill_picture_parameters(const HEVCContext *h,
                                    VAPictureParameterBufferHEVC *pp)
{
    const HEVCFrame *current_picture = h->ref;
    int i, j;

    memset(pp, 0, sizeof(*pp));

    fill_vaapi_pic(&pp->CurrPic, current_picture, h->picture_struct);

    pp->pic_width_in_luma_samples  = h->sps->min_cb_width;
    pp->pic_height_in_luma_samples = h->sps->min_cb_height;

    pp->pic_fields.bits.chroma_format_idc = h->sps->chroma_format_idc;

    pp->sps_max_dec_pic_buffering_minus1         = h->sps->temporal_layer[h->sps->max_sub_layers - 1].max_dec_pic_buffering - 1;
    pp->log2_min_luma_coding_block_size_minus3   = h->sps->log2_min_cb_size - 3;
    pp->log2_diff_max_min_luma_coding_block_size = h->sps->log2_diff_max_min_coding_block_size;
    pp->log2_min_transform_block_size_minus2     = h->sps->log2_min_tb_size - 2;
    pp->log2_diff_max_min_transform_block_size   = h->sps->log2_max_trafo_size  - h->sps->log2_min_tb_size;
    pp->max_transform_hierarchy_depth_inter      = h->sps->max_transform_hierarchy_depth_inter;
    pp->max_transform_hierarchy_depth_intra      = h->sps->max_transform_hierarchy_depth_intra;
    pp->num_short_term_ref_pic_sets              = h->sps->nb_st_rps;
//    pp->num_long_term_ref_pics_sps               = h->sps->num_long_term_ref_pics_sps;

    pp->num_ref_idx_l0_default_active_minus1     = h->pps->num_ref_idx_l0_default_active - 1;
    pp->num_ref_idx_l1_default_active_minus1     = h->pps->num_ref_idx_l1_default_active - 1;
    pp->init_qp_minus26                          = h->pps->pic_init_qp_minus26;

    pp->pps_cb_qp_offset            = h->pps->cb_qp_offset;
    pp->pps_cr_qp_offset            = h->pps->cr_qp_offset;
    if (h->pps->tiles_enabled_flag) {
        pp->num_tile_columns_minus1 = h->pps->num_tile_columns - 1;
        pp->num_tile_rows_minus1    = h->pps->num_tile_rows - 1;

        if (!h->pps->uniform_spacing_flag) {
            for (i = 0; i < h->pps->num_tile_columns; i++)
                pp->column_width_minus1[i] = h->pps->column_width[i] - 1;

            for (i = 0; i < h->pps->num_tile_rows; i++)
                pp->row_height_minus1[i] = h->pps->row_height[i] - 1;
        }
    }

    pp->diff_cu_qp_delta_depth           = h->pps->diff_cu_qp_delta_depth;
    pp->pps_beta_offset_div2             = h->pps->beta_offset / 2;
    pp->pps_tc_offset_div2               = h->pps->tc_offset / 2;
    pp->log2_parallel_merge_level_minus2 = h->pps->log2_parallel_merge_level - 2;
//    pp->CurrPicOrderCntVal               = h->poc;

    // fill RefPicList from the DPB
    for (i = 0, j = 0; i < FF_ARRAY_ELEMS(pp->ReferenceFrames); i++) {
        const HEVCFrame *frame = NULL;
        while (!frame && j < FF_ARRAY_ELEMS(h->DPB)) {
            if (&h->DPB[j] != current_picture && (h->DPB[j].flags & (HEVC_FRAME_FLAG_LONG_REF | HEVC_FRAME_FLAG_SHORT_REF)))
                frame = &h->DPB[j];
            j++;
        }

        if (frame) {
            fill_vaapi_pic(&pp->ReferenceFrames[i], frame, frame->flags & HEVC_FRAME_FLAG_LONG_REF);
        } else {
            pp->ReferenceFrames[i].picture_id = VA_PICTURE_HEVC_INVALID;
            pp->ReferenceFrames[i].pic_order_cnt = 0;
        }
    }
}


/** Initialize and start decoding a frame with VA API. */
static int vaapi_hevc_start_frame(AVCodecContext          *avctx,
                                  av_unused const uint8_t *buffer,
                                  av_unused uint32_t       size)
{
    HEVCContext * const h = avctx->priv_data;
    struct vaapi_context * const vactx = avctx->hwaccel_context;
    VAPictureParameterBufferHEVC *pic_param;
    VAIQMatrixBufferHEVC *iq_matrix;

    ff_dlog(avctx, "vaapi_hevc_start_frame()\n");

    vactx->slice_param_size = sizeof(VASliceParameterBufferHEVC);

    /* Fill in VAPictureParameterBufferHEVC. */
    pic_param = ff_vaapi_alloc_pic_param(vactx, sizeof(VAPictureParameterBufferHEVC));
    if (!pic_param)
        return -1;
    fill_picture_parameters(h, pic_param);

    return 0;
}

/** End a hardware decoding based frame. */
static int vaapi_hevc_end_frame(AVCodecContext *avctx)
{
    struct vaapi_context * const vactx = avctx->hwaccel_context;
    HEVCContext * const h = avctx->priv_data;
    int ret;

    ff_dlog(avctx, "vaapi_hevc_end_frame()\n");
    ret = ff_vaapi_commit_slices(vactx);
    if (ret < 0)
        goto finish;

    ret = ff_vaapi_render_picture(vactx, ff_vaapi_get_surface_id(h->ref->frame));
    if (ret < 0)
        goto finish;

finish:
    ff_vaapi_common_end_frame(avctx);
    return ret;
}

/** Decode the given hevc slice with VA API. */
static int vaapi_hevc_decode_slice(AVCodecContext *avctx,
                                   const uint8_t  *buffer,
                                   uint32_t        size)
{
    HEVCContext * const h = avctx->priv_data;
    VASliceParameterBufferHEVC *slice_param;

    ff_dlog(avctx, "vaapi_hevc_decode_slice(): buffer %p, size %d\n",
            buffer, size);

    /* Fill in VASliceParameterBufferH264. */
    slice_param = (VASliceParameterBufferHEVC *)ff_vaapi_alloc_slice(avctx->hwaccel_context, buffer, size);
    if (!slice_param)
        return -1;

    return 0;
}

AVHWAccel ff_hevc_vaapi_hwaccel = {
    .name           = "hevc_vaapi",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_HEVC,
    .pix_fmt        = AV_PIX_FMT_VAAPI_VLD,
    .start_frame    = vaapi_hevc_start_frame,
    .end_frame      = vaapi_hevc_end_frame,
    .decode_slice   = vaapi_hevc_decode_slice,
};
