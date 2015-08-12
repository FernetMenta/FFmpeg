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
                           HEVCFrame *pic,
                           int pic_structure)
{

}

/** Decoded Picture Buffer (DPB). */
typedef struct DPB {
    int            size;        ///< Current number of reference frames in the DPB
    int            max_size;    ///< Max number of reference frames. This is FF_ARRAY_ELEMS(VAPictureParameterBufferHEVC.ReferenceFrames)
    VAPictureHEVC *va_pics;     ///< Pointer to VAPictureParameterBufferHEVC.ReferenceFrames array
} DPB;

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
    fill_vaapi_pic(&pic_param->CurrPic, h->ref, h->picture_struct);

//    if (fill_vaapi_ReferenceFrames(pic_param, h) < 0)
//        return -1;

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
