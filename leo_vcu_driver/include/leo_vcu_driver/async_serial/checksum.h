/*
 * Library: libcrc
 * File:    include/checksum.h
 * Author:  Lammert Bies
 *
 * This file is licensed under the MIT License as stated below
 *
 * Copyright (c) 1999-2018 Lammert Bies
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Description
 * -----------
 * The headerfile include/checksum.h contains the definitions and prototypes
 * for routines that can be used to calculate several kinds of checksums.
 */

#ifndef DEF_LIBCRC_CHECKSUM_H
#define DEF_LIBCRC_CHECKSUM_H

#include <cstddef>
#include <cstdint>


static void init_crc16_tab();

static bool crc_tab16_init = false;
static uint16_t crc_tab16[256];


inline uint16_t crc_16(const uint8_t * input_str, size_t num_bytes)
{

  static const auto u16_ff {static_cast < uint16_t > (0x00FF)};

  if (!crc_tab16_init) {init_crc16_tab();}

  uint16_t crc {0};
  const uint8_t * ptr {input_str};
  if (ptr != nullptr) {
    for (size_t a = 0; a < num_bytes; a++) {
      crc = static_cast < uint16_t > (crc >> 8) ^
        crc_tab16[(crc ^ static_cast < uint16_t > (*ptr++)) & u16_ff];
    }
  }

  return crc;

}  /* crc_16 */


static void init_crc16_tab()
{

  uint16_t i;
  uint16_t j;
  uint16_t crc;
  uint16_t c;

  for (i = 0; i < 256; i++) {
    static const uint16_t crc_poly_16 = 0xA001;
    crc = 0;
    c = i;

    for (j = 0; j < 8; j++) {

      if ((crc ^ c) & 0x0001) {crc = static_cast < uint16_t > (crc >> 1) ^ crc_poly_16;} else {
        crc = static_cast < uint16_t > (crc >> 1);
      }

      c = static_cast < uint16_t > (c >> 1);
    }

    crc_tab16[i] = crc;
  }

  crc_tab16_init = true;

}  /* init_crc16_tab */


#endif  // DEF_LIBCRC_CHECKSUM_H
