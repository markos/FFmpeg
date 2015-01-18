/*
 * H.26L/H.264/AVC/JVT/14496-10/... encoder/decoder
 * Copyright (c) 2003 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (c) 2015 Konstantinos Margaritis <markos@freevec.org>
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
 * H.264 / AVC / MPEG4 part10 prediction functions.
 * @author Michael Niedermayer <michaelni@gmx.at>
 * @author Konstantinos Margaritis <markos@freevec.org>
 */

#include <stdio.h>

#include "libavutil/cpu.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/ppc/cpu.h"
#include "libavutil/ppc/types_altivec.h"
#include "libavutil/ppc/util_altivec.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/mathops.h"
#include "libavcodec/bit_depth_template.c"
#include "libavcodec/h264pred.h"


static void ff_pred16x16_vert_altivec(uint8_t *_src, ptrdiff_t _stride)
{
    int i;
    uint32_t *src = (uint32_t*)_src;
    int stride = _stride>>(sizeof(uint32_t)-1);
    vec_u32 srcv = vec_ld(0, src-stride);

    // completely unroll the loop
    VEC_ST(srcv, 0*stride, src); 
    VEC_ST(srcv, 1*stride, src);
    VEC_ST(srcv, 2*stride, src);
    VEC_ST(srcv, 3*stride, src);
    VEC_ST(srcv, 4*stride, src);
    VEC_ST(srcv, 5*stride, src);
    VEC_ST(srcv, 6*stride, src);
    VEC_ST(srcv, 7*stride, src);
    VEC_ST(srcv, 8*stride, src);
    VEC_ST(srcv, 9*stride, src);
    VEC_ST(srcv,10*stride, src);
    VEC_ST(srcv,11*stride, src);
    VEC_ST(srcv,12*stride, src);
    VEC_ST(srcv,13*stride, src);
    VEC_ST(srcv,14*stride, src);
    VEC_ST(srcv,15*stride, src);
}

static void ff_pred16x16_hor_altivec(uint8_t *_src, ptrdiff_t stride)
{
    DECLARE_ALIGNED(16, uint32_t, a)[4];
    int i;
    pixel *src = (pixel*)_src;
    stride >>= sizeof(pixel)-1;

    for(i=0; i<16; i++){
        a[0] = PIXEL_SPLAT_X4(src[-1+i*stride]);

        vec_u32 srcv = vec_ld(0, a);
        srcv = vec_splat(srcv, 0);

        VEC_ST(srcv, i*stride, (uint32_t *)(src));
    }
}

static void ff_pred16x16_dc_altivec(uint8_t *_src, ptrdiff_t stride)
{
    DECLARE_ALIGNED(16, uint32_t, dcsplat)[4];
    int i, dc=0;
    pixel *src = (pixel*)_src;
    stride >>= sizeof(pixel)-1;

    for(i=0;i<16; i++){
        dc+= src[-1+i*stride];
    }

    for(i=0;i<16; i++){
        dc+= src[i-stride];
    }

    dcsplat[0] = PIXEL_SPLAT_X4((dc+16)>>5);
    vec_u32 dcsplatv = vec_ld(0, dcsplat);
    dcsplatv = vec_splat(dcsplatv, 0);

    VEC_ST(dcsplatv, 0*stride, (uint32_t*) src); 
    VEC_ST(dcsplatv, 1*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 2*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 3*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 4*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 5*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 6*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 7*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 8*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 9*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,10*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,11*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,12*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,13*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,14*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,15*stride, (uint32_t*) src);
}

static void ff_pred16x16_left_dc_altivec(uint8_t *_src, ptrdiff_t stride)
{
    DECLARE_ALIGNED(16, uint32_t, dcsplat)[4];
    int i, dc=0;
    pixel *src = (pixel*)_src;
    stride >>= sizeof(pixel)-1;

    for(i=0;i<16; i++){
        dc+= src[-1+i*stride];
    }

    dcsplat[0] = PIXEL_SPLAT_X4((dc+8)>>4);
    vec_u32 dcsplatv = vec_ld(0, dcsplat);
    dcsplatv = vec_splat(dcsplatv, 0);

    VEC_ST(dcsplatv, 0*stride, (uint32_t*) src); 
    VEC_ST(dcsplatv, 1*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 2*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 3*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 4*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 5*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 6*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 7*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 8*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 9*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,10*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,11*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,12*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,13*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,14*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,15*stride, (uint32_t*) src);
}

static void ff_pred16x16_top_dc_altivec(uint8_t *_src, ptrdiff_t stride)
{
    DECLARE_ALIGNED(16, uint32_t, dcsplat)[4];
    int i, dc=0;
    pixel *src = (pixel*)_src;
    stride >>= sizeof(pixel)-1;

    for(i=0;i<16; i++){
        dc+= src[i-stride];
    }

    dcsplat[0] = PIXEL_SPLAT_X4((dc+8)>>4);
    vec_u32 dcsplatv = vec_ld(0, dcsplat);
    dcsplatv = vec_splat(dcsplatv, 0);

    VEC_ST(dcsplatv, 0*stride, (uint32_t*) src); 
    VEC_ST(dcsplatv, 1*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 2*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 3*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 4*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 5*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 6*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 7*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 8*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 9*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,10*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,11*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,12*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,13*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,14*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,15*stride, (uint32_t*) src);
}

static void ff_pred16x16_128_dc_altivec(uint8_t *_src, ptrdiff_t stride)
{
    DECLARE_ALIGNED(16, uint32_t, dcsplat)[4];
    int i;
    pixel *src = (pixel*)_src;
    stride >>= sizeof(pixel)-1;
    
    dcsplat[0] = PIXEL_SPLAT_X4((1<<(BIT_DEPTH-1))+0);
    vec_u32 dcsplatv = vec_ld(0, dcsplat);
    dcsplatv = vec_splat(dcsplatv, 0);

    VEC_ST(dcsplatv, 0*stride, (uint32_t*) src); 
    VEC_ST(dcsplatv, 1*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 2*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 3*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 4*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 5*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 6*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 7*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 8*stride, (uint32_t*) src);
    VEC_ST(dcsplatv, 9*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,10*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,11*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,12*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,13*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,14*stride, (uint32_t*) src);
    VEC_ST(dcsplatv,15*stride, (uint32_t*) src);
}

// TODO: Prpperly optimize loop, now it's just a dup
static inline void ff_pred16x16_plane_compat_altivec(uint8_t *_src,
                                                 ptrdiff_t _stride,
                                                 const int svq3,
                                                 const int rv40)
{
  int i, j, k;
  int a;
  INIT_CLIP
  pixel *src = (pixel*)_src;
  int stride = _stride>>(sizeof(pixel)-1);
  const pixel * const src0 = src +7-stride;
  const pixel *       src1 = src +8*stride-1;
  const pixel *       src2 = src1-2*stride;    // == src+6*stride-1;
  int H = src0[1] - src0[-1];
  int V = src1[0] - src2[ 0];
  for(k=2; k<=8; ++k) {
    src1 += stride; src2 -= stride;
    H += k*(src0[k] - src0[-k]);
    V += k*(src1[0] - src2[ 0]);
  }
  if(svq3){
    H = ( 5*(H/4) ) / 16;
    V = ( 5*(V/4) ) / 16;

    /* required for 100% accuracy */
    i = H; H = V; V = i;
  }else if(rv40){
    H = ( H + (H>>2) ) >> 4;
    V = ( V + (V>>2) ) >> 4;
  }else{
    H = ( 5*H+32 ) >> 6;
    V = ( 5*V+32 ) >> 6;
  }

  a = 16*(src1[0] + src2[16] + 1) - 7*(V+H);
  for(j=16; j>0; --j) {
    int b = a;
    a += V;
    for(i=-16; i<0; i+=4) {
      src[16+i] = CLIP((b    ) >> 5);
      src[17+i] = CLIP((b+  H) >> 5);
      src[18+i] = CLIP((b+2*H) >> 5);
      src[19+i] = CLIP((b+3*H) >> 5);
      b += 4*H;
    }
    src += stride;
  }
}

static void ff_pred16x16_plane_altivec(uint8_t *src, ptrdiff_t stride)
{
    ff_pred16x16_plane_compat_altivec(src, stride, 0, 0);
}

static av_cold void h264_pred_init_altivec(H264PredContext *h, int codec_id,
                                        const int bit_depth,
                                        const int chroma_format_idc)
{
#if HAVE_ALTIVEC
    const int high_depth = bit_depth > 8;

    if (high_depth)
        return;

    h->pred16x16[DC_PRED8x8     ] = ff_pred16x16_dc_altivec;
    h->pred16x16[VERT_PRED8x8   ] = ff_pred16x16_vert_altivec;
    h->pred16x16[HOR_PRED8x8    ] = ff_pred16x16_hor_altivec;
    h->pred16x16[LEFT_DC_PRED8x8] = ff_pred16x16_left_dc_altivec;
    h->pred16x16[TOP_DC_PRED8x8 ] = ff_pred16x16_top_dc_altivec;
    h->pred16x16[DC_128_PRED8x8 ] = ff_pred16x16_128_dc_altivec;
/*  TODO: Set when function properly implemented
    if (codec_id != AV_CODEC_ID_SVQ3 && codec_id != AV_CODEC_ID_RV40 &&
        codec_id != AV_CODEC_ID_VP7 && codec_id != AV_CODEC_ID_VP8)
        h->pred16x16[PLANE_PRED8x8  ] = ff_pred16x16_plane_altivec;*/
#endif // HAVE_ALTIVEC
}

av_cold void ff_h264_pred_init_ppc(H264PredContext *h, int codec_id,
                                   int bit_depth, const int chroma_format_idc)
{
    int cpu_flags = av_get_cpu_flags();

    if (PPC_ALTIVEC(av_get_cpu_flags()))
        h264_pred_init_altivec(h, codec_id, bit_depth, chroma_format_idc);
}
