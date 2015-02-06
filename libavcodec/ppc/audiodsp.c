/*
 * Copyright (c) 2007 Luca Barbato <lu_zero@gentoo.org>
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

/**
 * @file
 * miscellaneous audio operations
 */

#include "config.h"
#if HAVE_ALTIVEC_H
#include <altivec.h>
#endif

#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/mem.h"
#include "libavutil/ppc/cpu.h"
#include "libavutil/ppc/types_altivec.h"
#include "libavutil/ppc/util_altivec.h"
#include "libavcodec/audiodsp.h"

#if HAVE_ALTIVEC

static void vector_clipf_altivec(float *dst, const float *src,
                           float min, float max, int len)
{
    int i;
    DECLARE_ALIGNED(16, float, tmp)[4];
    tmp[0] = min;
    vector float vmin, vmax, vec[2];
    vector bool cmpmask[2];
    vmin = vec_ld(0, tmp);
    tmp[0] = max;
    vmax = vec_ld(0, tmp);

    vmin = vec_splat(vmin, 0);
    vmax = vec_splat(vmax, 0);

    for (i = 0; i < len; i += 8) {
        vec[0] = vec_ld( 0, &src[i]);
        vec[1] = vec_ld(16, &src[i]);
        cmpmask[0] = vec_cmpgt(vec[0], vmax);
        cmpmask[1] = vec_cmpgt(vec[1], vmax);
        vec[0] = vec_sel(vec[0], vmax, cmpmask[0]);
        vec[1] = vec_sel(vec[1], vmax, cmpmask[1]);
        cmpmask[0] = vec_cmplt(vec[0], vmin);
        cmpmask[1] = vec_cmplt(vec[1], vmin);
        vec[0] = vec_sel(vec[0], vmin, cmpmask[0]);
        vec[1] = vec_sel(vec[1], vmin, cmpmask[1]);
        vec_st(vec[0],  0, &dst[i]);
        vec_st(vec[1], 16, &dst[i]);
    }
}

static int32_t scalarproduct_int16_altivec(const int16_t *v1, const int16_t *v2,
                                           int order)
{
    int i;
    LOAD_ZERO;
    register vec_s16 vec1;
    register vec_s32 res = vec_splat_s32(0), t;
    int32_t ires;

//    printf("scalarproduct_int16_altivec: v1 = %08x, v2 = %08x, order = %d\n", v1, v2, order);
    for (i = 0; i < order; i += 8) {
        vec1 = vec_unaligned_load(v1);
        t    = vec_msum(vec1, vec_ld(0, v2), zero_s32v);
        res  = vec_sums(t, res);
        v1  += 8;
        v2  += 8;
    }
    res = vec_splat(res, 3);
    vec_ste(res, 0, &ires);

    return ires;
}

static void vector_clip_int32_altivec(int32_t *dst, const int32_t *src, int32_t min,
                                int32_t max, unsigned int len)
{
    int i;
    DECLARE_ALIGNED(16, int32_t, tmp)[4];
    tmp[0] = min;
    vector int32_t vmin, vmax, vec[2];
    vector bool cmpmask[2];
    vmin = vec_ld(0, tmp);
    tmp[0] = max;
    vmax = vec_ld(0, tmp);

    vmin = vec_splat(vmin, 0);
    vmax = vec_splat(vmax, 0);

    for (i = 0; i < len; i += 8) {
        vec[0] = vec_ld( 0, &src[i]);
        vec[1] = vec_ld(16, &src[i]);
        cmpmask[0] = vec_cmpgt(vec[0], vmax);
        cmpmask[1] = vec_cmpgt(vec[1], vmax);
        vec[0] = vec_sel(vec[0], vmax, cmpmask[0]);
        vec[1] = vec_sel(vec[1], vmax, cmpmask[1]);
        cmpmask[0] = vec_cmplt(vec[0], vmin);
        cmpmask[1] = vec_cmplt(vec[1], vmin);
        vec[0] = vec_sel(vec[0], vmin, cmpmask[0]);
        vec[1] = vec_sel(vec[1], vmin, cmpmask[1]);
        vec_st(vec[0],  0, &dst[i]);
        vec_st(vec[1], 16, &dst[i]);
    }
}


#endif /* HAVE_ALTIVEC */

av_cold void ff_audiodsp_init_ppc(AudioDSPContext *c)
{
#if HAVE_ALTIVEC
    if (!PPC_ALTIVEC(av_get_cpu_flags()))
        return;

    c->scalarproduct_int16 = scalarproduct_int16_altivec;
    c->vector_clip_int32   = vector_clip_int32_altivec;
    c->vector_clipf        = vector_clipf_altivec;
#endif /* HAVE_ALTIVEC */
}
