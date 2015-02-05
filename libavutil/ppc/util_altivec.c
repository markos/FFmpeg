/*
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
 * Contains misc utility macros and inline functions
 */

#include "util_altivec.h"

/* Need to produce one of these permutation masks, depending on whether addr
   is 8-byte or 16-byte aligned:
   10 11 12 13 14 15 16 17 08 09 0a 0b 0c 0d 0e 0f  (16-byte aligned)
   00 01 02 03 04 05 06 07 10 11 12 13 14 15 16 17  (8-byte aligned)
 */

void rightside_permmask_init(vec_u8 permmasks[2]) {
    int i = 0;
    vec_u8 v, termsub, masksub, result;
    vec_u8 v_0 = vec_splat_u8(0), v_8 = vec_splat_u8(8);
    vec_u8 v_16 = vec_add(v_8, v_8);
    vec_u8 v_24 = vec_add(v_16, v_8);

    do {
        /* Will produce either 16-byte aligned or 8-byte aligned arrays:
           00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f  (16-byte aligned)
           08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17  (8-byte aligned)
         */
        v = vec_lvsl(0, (uint8_t*)(i*8));

        /* Shift 8bytes to the left, and add 8 bytes to the lot, will produce one of the following:
           10 11 12 13 14 15 16 17 08 09 0a 0b 0c 0d 0e 0f  (16-byte aligned)
           18 19 1a 1b 1c 1d 1e 1f 10 11 12 13 14 15 16 17  (8-byte aligned)
         */
        result = vec_sld(v, v, 8);
        result = vec_add(result, v_8);

        /* Get the negated mask after comparing if <24(18 hex), fill the elements with '0x18':
           00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  (16-byte aligned)
           18 18 18 18 18 18 18 18 00 00 00 00 00 00 00 00  (8-byte aligned)
         */
        masksub = (vec_u8) vec_cmplt(result, v_24);
        masksub = vec_nor(masksub, masksub);
        termsub = vec_sel(v_0, v_24, masksub);

        /* Subtract from the result:
           10 11 12 13 14 15 16 17 08 09 0a 0b 0c 0d 0e 0f  (16-byte aligned, no change)
           00 01 02 03 04 05 06 07 10 11 12 13 14 15 16 17  (8-byte aligned)
           QED
         */
        permmasks[i] = vec_sub(result, termsub);
    } while (++i < 2);
}


