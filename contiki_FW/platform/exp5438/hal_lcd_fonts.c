/*******************************************************************************
 *
 *  hal_lcd_fonts.c
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

const unsigned char fonts_lookup[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    64, 65, 0, 69, 0, 68, 67, 0, 0, 1,      //'0' = 48 = 0x30
    2, 3, 4, 5, 6, 7, 8, 9, 66, 0,          //'9' = 57 = 0x39
    0, 70, 0, 62, 0, 10, 11, 12, 13, 14,    //'A' --> 'Z'
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30, 31, 32, 33, 34,
    35, 0, 0, 0, 71, 0, 0, 36, 37, 38,      //'a' = 97
    39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
    49, 50, 51, 52, 53, 54, 55, 56, 57, 58,
    59, 60, 61, 62, 0, 0, 0, 72, 73, 74,
    75, 76, 77, 78, 79, 80, 81              //'z' = 122
};

const unsigned int fonts[] = {
    0x0000, 0x0ffc, 0x3c0f, 0x3f0f, 0x3fcf, 0x3ccf, 0x3cff, 0x3c3f,
    0x3c0f, 0x0ffc, 0x0000, 0x0000, 0x0000, 0x0000, 0x00c0, 0x00f0,
    0x00ff, 0x00f0, 0x00f0, 0x00f0, 0x00f0, 0x00f0, 0x0fff, 0x0000,
    0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f00, 0x03c0,
    0x00f0, 0x003c, 0x0f0f, 0x0fff, 0x0000, 0x0000, 0x0000, 0x0000,
    0x03fc, 0x0f0f, 0x0f00, 0x0f00, 0x03f0, 0x0f00, 0x0f00, 0x0f0f,
    0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x0f00, 0x0fc0, 0x0ff0,
    0x0f3c, 0x0f0f, 0x3fff, 0x0f00, 0x0f00, 0x3fc0, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0fff, 0x000f, 0x000f, 0x000f, 0x03ff, 0x0f00,
    0x0f00, 0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x03f0,
    0x003c, 0x000f, 0x000f, 0x03ff, 0x0f0f, 0x0f0f, 0x0f0f, 0x03fc,
    0x0000, 0x0000, 0x0000, 0x0000, 0x3fff, 0x3c0f, 0x3c0f, 0x3c00,
    0x0f00, 0x03c0, 0x00f0, 0x00f0, 0x00f0, 0x0000, 0x0000, 0x0000,
    0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f3f, 0x03fc, 0x0fcf, 0x0f0f,
    0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f,
    0x0f0f, 0x0f0f, 0x0ffc, 0x03c0, 0x03c0, 0x00f0, 0x00fc, 0x0000,
    0x0000, 0x0000, 0x0000, 0x00f0, 0x03fc, 0x0f0f, 0x0f0f, 0x0f0f,
    0x0fff, 0x0f0f, 0x0f0f, 0x0f0f, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0fff, 0x3c3c, 0x3c3c, 0x3c3c, 0x0ffc, 0x3c3c, 0x3c3c, 0x3c3c,
    0x0fff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0ff0, 0x3c3c, 0x3c0f,
    0x000f, 0x000f, 0x000f, 0x3c0f, 0x3c3c, 0x0ff0, 0x0000, 0x0000,
    0x0000, 0x0000, 0x03ff, 0x0f3c, 0x3c3c, 0x3c3c, 0x3c3c, 0x3c3c,
    0x3c3c, 0x0f3c, 0x03ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x3fff,
    0x303c, 0x003c, 0x0c3c, 0x0ffc, 0x0c3c, 0x003c, 0x303c, 0x3fff,
    0x0000, 0x0000, 0x0000, 0x0000, 0x3fff, 0x3c3c, 0x303c, 0x0c3c,
    0x0ffc, 0x0c3c, 0x003c, 0x003c, 0x00ff, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0ff0, 0x3c3c, 0x3c0f, 0x000f, 0x000f, 0x3f0f, 0x3c0f,
    0x3c3c, 0x3ff0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0f0f, 0x0f0f,
    0x0f0f, 0x0f0f, 0x0fff, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0000,
    0x0000, 0x0000, 0x0000, 0x03fc, 0x00f0, 0x00f0, 0x00f0, 0x00f0,
    0x00f0, 0x00f0, 0x00f0, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000,
    0x3fc0, 0x0f00, 0x0f00, 0x0f00, 0x0f00, 0x0f0f, 0x0f0f, 0x0f0f,
    0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x3c3f, 0x3c3c, 0x0f3c,
    0x0f3c, 0x03fc, 0x0f3c, 0x0f3c, 0x3c3c, 0x3c3f, 0x0000, 0x0000,
    0x0000, 0x0000, 0x00ff, 0x003c, 0x003c, 0x003c, 0x003c, 0x303c,
    0x3c3c, 0x3c3c, 0x3fff, 0x0000, 0x0000, 0x0000, 0x0000, 0x3c0f,
    0x3f3f, 0x3fff, 0x3fff, 0x3ccf, 0x3c0f, 0x3c0f, 0x3c0f, 0x3c0f,
    0x0000, 0x0000, 0x0000, 0x0000, 0x3c0f, 0x3c0f, 0x3c3f, 0x3cff,
    0x3fff, 0x3fcf, 0x3f0f, 0x3c0f, 0x3c0f, 0x0000, 0x0000, 0x0000,
    0x0000, 0x03f0, 0x0f3c, 0x3c0f, 0x3c0f, 0x3c0f, 0x3c0f, 0x3c0f,
    0x0f3c, 0x03f0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0fff, 0x3c3c,
    0x3c3c, 0x3c3c, 0x0ffc, 0x003c, 0x003c, 0x003c, 0x00ff, 0x0000,
    0x0000, 0x0000, 0x0000, 0x03f0, 0x0f3c, 0x3c0f, 0x3c0f, 0x3c0f,
    0x3f0f, 0x3fcf, 0x0ffc, 0x0f00, 0x3fc0, 0x0000, 0x0000, 0x0000,
    0x0fff, 0x3c3c, 0x3c3c, 0x3c3c, 0x0ffc, 0x0f3c, 0x3c3c, 0x3c3c,
    0x3c3f, 0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f, 0x0f0f,
    0x000f, 0x00fc, 0x03c0, 0x0f0f, 0x0f0f, 0x03fc, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0fff, 0x0cf3, 0x00f0, 0x00f0, 0x00f0, 0x00f0,
    0x00f0, 0x00f0, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x0f0f,
    0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x03fc,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f,
    0x0f0f, 0x0f0f, 0x0f0f, 0x03fc, 0x00f0, 0x0000, 0x0000, 0x0000,
    0x0000, 0x3c0f, 0x3c0f, 0x3c0f, 0x3c0f, 0x3ccf, 0x3ccf, 0x0f3c,
    0x0f3c, 0x0f3c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0f0f, 0x0f0f,
    0x0f0f, 0x03fc, 0x00f0, 0x03fc, 0x0f0f, 0x0f0f, 0x0f0f, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x03fc,
    0x00f0, 0x00f0, 0x00f0, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000,
    0x3fff, 0x3f0f, 0x03c3, 0x03c0, 0x00f0, 0x003c, 0x303c, 0x3c0f,
    0x3fff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x03fc, 0x0f00, 0x0ffc, 0x0f0f, 0x0f0f, 0x3cfc, 0x0000, 0x0000,
    0x0000, 0x0000, 0x003f, 0x003c, 0x003c, 0x0ffc, 0x3c3c, 0x3c3c,
    0x3c3c, 0x3c3c, 0x0fcf, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x03fc, 0x0f0f, 0x000f, 0x000f, 0x0f0f, 0x03fc,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0fc0, 0x0f00, 0x0f00, 0x0ffc,
    0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x3cfc, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f, 0x0fff, 0x000f,
    0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x03f0, 0x0f3c,
    0x003c, 0x003c, 0x03ff, 0x003c, 0x003c, 0x003c, 0x00ff, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3cfc, 0x0f0f,
    0x0f0f, 0x0f0f, 0x0ffc, 0x0f00, 0x0f0f, 0x03fc, 0x0000, 0x0000,
    0x003f, 0x003c, 0x003c, 0x0f3c, 0x3cfc, 0x3c3c, 0x3c3c, 0x3c3c,
    0x3c3f, 0x0000, 0x0000, 0x0000, 0x0000, 0x03c0, 0x03c0, 0x0000,
    0x03fc, 0x03c0, 0x03c0, 0x03c0, 0x03c0, 0x3ffc, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0f00, 0x0f00, 0x0000, 0x0ff0, 0x0f00, 0x0f00,
    0x0f00, 0x0f00, 0x0f0f, 0x0f0f, 0x03fc, 0x0000, 0x0000, 0x003f,
    0x003c, 0x003c, 0x3c3c, 0x0f3c, 0x03fc, 0x0f3c, 0x3c3c, 0x3c3f,
    0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x03c0, 0x03c0, 0x03c0,
    0x03c0, 0x03c0, 0x03c0, 0x03c0, 0x3ffc, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0fff, 0x3ccf, 0x3ccf, 0x3ccf,
    0x3ccf, 0x3c0f, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x03ff, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f,
    0x0f0f, 0x0f0f, 0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0fcf, 0x3c3c, 0x3c3c, 0x3c3c, 0x3c3c,
    0x0ffc, 0x003c, 0x00ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x3cfc, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0ffc, 0x0f00, 0x3fc0,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0f3f, 0x3f3c, 0x3cfc,
    0x003c, 0x003c, 0x00ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x03fc, 0x0f0f, 0x003c, 0x03c0, 0x0f0f, 0x03fc,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0030, 0x003c, 0x0fff,
    0x003c, 0x003c, 0x003c, 0x0f3c, 0x03f0, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f,
    0x0f0f, 0x3cfc, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x03fc, 0x00f0, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3c0f, 0x3c0f,
    0x3ccf, 0x3ccf, 0x0f3c, 0x0f3c, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x3c0f, 0x0f3c, 0x03f0, 0x03f0, 0x0f3c,
    0x3c0f, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x3c3c, 0x3c3c, 0x3c3c, 0x3c3c, 0x0ff0, 0x0f00, 0x03c0, 0x00ff,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0fff, 0x0f03, 0x03c0,
    0x003c, 0x0c0f, 0x0fff, 0x0000, 0x0000, 0x0000, 0x0000, 0x03fc,
    0x0f0f, 0x0f00, 0x03c0, 0x00f0, 0x00f0, 0x0000, 0x00f0, 0x00f0,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0f00, 0x03c0, 0x00f0, 0x003c, 0x003c, 0x003c, 0x00f0,
    0x03c0, 0x0f00, 0x0000, 0x0000, 0x0000, 0x0000, 0x003c, 0x00f0,
    0x03c0, 0x0f00, 0x0f00, 0x0f00, 0x03c0, 0x00f0, 0x003c, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03f0, 0x03f0, 0x0000,
    0x0000, 0x03f0, 0x03f0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03f0,
    0x03f0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x3ffc, 0x3ffc, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x03c0, 0x03c0, 0x3ffc, 0x3ffc,
    0x03c0, 0x03c0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x3ffc, 0x0000, 0x0000, 0x3ffc, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f0f,
    0x03fc, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    //0---------------------------
    0x0000, 0x0ffc, 0x3c0f, 0x3f0f, 0x3fcf, 0x3ccf, 0x3cff, 0x3c3f,
    0x3c0f, 0x0ffc, 0x0000, 0x0000, 0x0000,
    //1---------------------------
    0x0000, 0x00c0, 0x00f0, 0x00ff, 0x00f0, 0x00f0, 0x00f0, 0x00f0,
    0x00f0, 0x0fff, 0x0000, 0x0000, 0x0000,
    //2---------------------------
    0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f00, 0x03c0, 0x00f0, 0x003c,
    0x0f0f, 0x0fff, 0x0000, 0x0000, 0x0000,
    //3---------------------------
    0x0000, 0x03fc, 0x0f0f, 0x0f00, 0x0f00, 0x03f0, 0x0f00, 0x0f00,
    0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000,
    //4---------------------------
    0x0000, 0x0f00, 0x0fc0, 0x0ff0, 0x0f3c, 0x0f0f, 0x3fff, 0x0f00,
    0x0f00, 0x3fc0, 0x0000, 0x0000, 0x0000,
    //5---------------------------
    0x0000, 0x0fff, 0x000f, 0x000f, 0x000f, 0x03ff, 0x0f00, 0x0f00,
    0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000,
    //6---------------------------
    0x0000, 0x03f0, 0x003c, 0x000f, 0x000f, 0x03ff, 0x0f0f, 0x0f0f,
    0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000,
    //7---------------------------
    0x0000, 0x3fff, 0x3c0f, 0x3c0f, 0x3c00, 0x0f00, 0x03c0, 0x00f0,
    0x00f0, 0x00f0, 0x0000, 0x0000, 0x0000,
    //8---------------------------
    0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f3f, 0x03fc, 0x0fcf, 0x0f0f,
    0x0f0f, 0x03fc, 0x0000, 0x0000, 0x0000,
    //9---------------------------
    0x0000, 0x03fc, 0x0f0f, 0x0f0f, 0x0f0f, 0x0ffc, 0x03c0, 0x03c0,
    0x00f0, 0x00fc, 0x0000, 0x0000, 0x0000,
};


const unsigned int GrayScale_fonts[] = {
    0x0000, 0x0aa8, 0x280a, 0x2a0a, 0x2a8a, 0x288a, 0x28aa, 0x282a,
    0x280a, 0x0aa8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0080, 0x00a0,
    0x00aa, 0x00a0, 0x00a0, 0x00a0, 0x00a0, 0x00a0, 0x0aaa, 0x0000,
    0x0000, 0x0000, 0x0000, 0x02a8, 0x0a0a, 0x0a0a, 0x0a00, 0x0280,
    0x00a0, 0x0028, 0x0a0a, 0x0aaa, 0x0000, 0x0000, 0x0000, 0x0000,
    0x02a8, 0x0a0a, 0x0a00, 0x0a00, 0x02a0, 0x0a00, 0x0a00, 0x0a0a,
    0x02a8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0a00, 0x0a80, 0x0aa0,
    0x0a28, 0x0a0a, 0x2aaa, 0x0a00, 0x0a00, 0x2a80, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0aaa, 0x000a, 0x000a, 0x000a, 0x02aa, 0x0a00,
    0x0a00, 0x0a0a, 0x02a8, 0x0000, 0x0000, 0x0000, 0x0000, 0x02a0,
    0x0028, 0x000a, 0x000a, 0x02aa, 0x0a0a, 0x0a0a, 0x0a0a, 0x02a8,
    0x0000, 0x0000, 0x0000, 0x0000, 0x2aaa, 0x280a, 0x280a, 0x2800,
    0x0a00, 0x0280, 0x00a0, 0x00a0, 0x00a0, 0x0000, 0x0000, 0x0000,
    0x0000, 0x02a8, 0x0a0a, 0x0a0a, 0x0a2a, 0x02a8, 0x0a8a, 0x0a0a,
    0x0a0a, 0x02a8, 0x0000, 0x0000, 0x0000, 0x0000, 0x02a8, 0x0a0a,
    0x0a0a, 0x0a0a, 0x0aa8, 0x0280, 0x0280, 0x00a0, 0x00a8, 0x0000,
    0x0000, 0x0000, 0x0000, 0x00a0, 0x02a8, 0x0a0a, 0x0a0a, 0x0a0a,
    0x0aaa, 0x0a0a, 0x0a0a, 0x0a0a, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0aaa, 0x2828, 0x2828, 0x2828, 0x0aa8, 0x2828, 0x2828, 0x2828,
    0x0aaa, 0x0000, 0x0000, 0x0000, 0x0000, 0x0aa0, 0x2828, 0x280a,
    0x000a, 0x000a, 0x000a, 0x280a, 0x2828, 0x0aa0, 0x0000, 0x0000,
    0x0000, 0x0000, 0x02aa, 0x0a28, 0x2828, 0x2828, 0x2828, 0x2828,
    0x2828, 0x0a28, 0x02aa, 0x0000, 0x0000, 0x0000, 0x0000, 0x2aaa,
    0x2028, 0x0028, 0x0828, 0x0aa8, 0x0828, 0x0028, 0x2028, 0x2aaa,
    0x0000, 0x0000, 0x0000, 0x0000, 0x2aaa, 0x2828, 0x2028, 0x0828,
    0x0aa8, 0x0828, 0x0028, 0x0028, 0x00aa, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0aa0, 0x2828, 0x280a, 0x000a, 0x000a, 0x2a0a, 0x280a,
    0x2828, 0x2aa0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0a0a, 0x0a0a,
    0x0a0a, 0x0a0a, 0x0aaa, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x0000,
    0x0000, 0x0000, 0x0000, 0x02a8, 0x00a0, 0x00a0, 0x00a0, 0x00a0,
    0x00a0, 0x00a0, 0x00a0, 0x02a8, 0x0000, 0x0000, 0x0000, 0x0000,
    0x2a80, 0x0a00, 0x0a00, 0x0a00, 0x0a00, 0x0a0a, 0x0a0a, 0x0a0a,
    0x02a8, 0x0000, 0x0000, 0x0000, 0x0000, 0x282a, 0x2828, 0x0a28,
    0x0a28, 0x02a8, 0x0a28, 0x0a28, 0x2828, 0x282a, 0x0000, 0x0000,
    0x0000, 0x0000, 0x00aa, 0x0028, 0x0028, 0x0028, 0x0028, 0x2028,
    0x2828, 0x2828, 0x2aaa, 0x0000, 0x0000, 0x0000, 0x0000, 0x280a,
    0x2a2a, 0x2aaa, 0x2aaa, 0x288a, 0x280a, 0x280a, 0x280a, 0x280a,
    0x0000, 0x0000, 0x0000, 0x0000, 0x280a, 0x280a, 0x282a, 0x28aa,
    0x2aaa, 0x2a8a, 0x2a0a, 0x280a, 0x280a, 0x0000, 0x0000, 0x0000,
    0x0000, 0x02a0, 0x0a28, 0x280a, 0x280a, 0x280a, 0x280a, 0x280a,
    0x0a28, 0x02a0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0aaa, 0x2828,
    0x2828, 0x2828, 0x0aa8, 0x0028, 0x0028, 0x0028, 0x00aa, 0x0000,
    0x0000, 0x0000, 0x0000, 0x02a0, 0x0a28, 0x280a, 0x280a, 0x280a,
    0x2a0a, 0x2a8a, 0x0aa8, 0x0a00, 0x2a80, 0x0000, 0x0000, 0x0000,
    0x0aaa, 0x2828, 0x2828, 0x2828, 0x0aa8, 0x0a28, 0x2828, 0x2828,
    0x282a, 0x0000, 0x0000, 0x0000, 0x0000, 0x02a8, 0x0a0a, 0x0a0a,
    0x000a, 0x00a8, 0x0280, 0x0a0a, 0x0a0a, 0x02a8, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0aaa, 0x08a2, 0x00a0, 0x00a0, 0x00a0, 0x00a0,
    0x00a0, 0x00a0, 0x02a8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0a0a,
    0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x02a8,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a,
    0x0a0a, 0x0a0a, 0x0a0a, 0x02a8, 0x00a0, 0x0000, 0x0000, 0x0000,
    0x0000, 0x280a, 0x280a, 0x280a, 0x280a, 0x288a, 0x288a, 0x0a28,
    0x0a28, 0x0a28, 0x0000, 0x0000, 0x0000, 0x0000, 0x0a0a, 0x0a0a,
    0x0a0a, 0x02a8, 0x00a0, 0x02a8, 0x0a0a, 0x0a0a, 0x0a0a, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x02a8,
    0x00a0, 0x00a0, 0x00a0, 0x02a8, 0x0000, 0x0000, 0x0000, 0x0000,
    0x2aaa, 0x2a0a, 0x0282, 0x0280, 0x00a0, 0x0028, 0x2028, 0x280a,
    0x2aaa, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x02a8, 0x0a00, 0x0aa8, 0x0a0a, 0x0a0a, 0x28a8, 0x0000, 0x0000,
    0x0000, 0x0000, 0x002a, 0x0028, 0x0028, 0x0aa8, 0x2828, 0x2828,
    0x2828, 0x2828, 0x0a8a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x02a8, 0x0a0a, 0x000a, 0x000a, 0x0a0a, 0x02a8,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0a80, 0x0a00, 0x0a00, 0x0aa8,
    0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x28a8, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x02a8, 0x0a0a, 0x0aaa, 0x000a,
    0x0a0a, 0x02a8, 0x0000, 0x0000, 0x0000, 0x0000, 0x02a0, 0x0a28,
    0x0028, 0x0028, 0x02aa, 0x0028, 0x0028, 0x0028, 0x00aa, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x28a8, 0x0a0a,
    0x0a0a, 0x0a0a, 0x0aa8, 0x0a00, 0x0a0a, 0x02a8, 0x0000, 0x0000,
    0x002a, 0x0028, 0x0028, 0x0a28, 0x28a8, 0x2828, 0x2828, 0x2828,
    0x282a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0280, 0x0280, 0x0000,
    0x02a8, 0x0280, 0x0280, 0x0280, 0x0280, 0x2aa8, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0a00, 0x0a00, 0x0000, 0x0aa0, 0x0a00, 0x0a00,
    0x0a00, 0x0a00, 0x0a0a, 0x0a0a, 0x02a8, 0x0000, 0x0000, 0x002a,
    0x0028, 0x0028, 0x2828, 0x0a28, 0x02a8, 0x0a28, 0x2828, 0x282a,
    0x0000, 0x0000, 0x0000, 0x0000, 0x02a8, 0x0280, 0x0280, 0x0280,
    0x0280, 0x0280, 0x0280, 0x0280, 0x2aa8, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0aaa, 0x288a, 0x288a, 0x288a,
    0x288a, 0x280a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x02aa, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x02a8, 0x0a0a,
    0x0a0a, 0x0a0a, 0x0a0a, 0x02a8, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0a8a, 0x2828, 0x2828, 0x2828, 0x2828,
    0x0aa8, 0x0028, 0x00aa, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x28a8, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x0aa8, 0x0a00, 0x2a80,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0a2a, 0x2a28, 0x28a8,
    0x0028, 0x0028, 0x00aa, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x02a8, 0x0a0a, 0x0028, 0x0280, 0x0a0a, 0x02a8,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0028, 0x0aaa,
    0x0028, 0x0028, 0x0028, 0x0a28, 0x02a0, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a,
    0x0a0a, 0x28a8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0a0a, 0x0a0a, 0x0a0a, 0x0a0a, 0x02a8, 0x00a0, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x280a, 0x280a,
    0x288a, 0x288a, 0x0a28, 0x0a28, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x280a, 0x0a28, 0x02a0, 0x02a0, 0x0a28,
    0x280a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x2828, 0x2828, 0x2828, 0x2828, 0x0aa0, 0x0a00, 0x0280, 0x00aa,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0aaa, 0x0a02, 0x0280,
    0x0028, 0x080a, 0x0aaa, 0x0000, 0x0000, 0x0000, 0x0000, 0x02a8,
    0x0a0a, 0x0a00, 0x0280, 0x00a0, 0x00a0, 0x0000, 0x00a0, 0x00a0,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0a00, 0x0280, 0x00a0, 0x0028, 0x0028, 0x0028, 0x00a0,
    0x0280, 0x0a00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0028, 0x00a0,
    0x0280, 0x0a00, 0x0a00, 0x0a00, 0x0280, 0x00a0, 0x0028, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x02a0, 0x02a0, 0x0000,
    0x0000, 0x02a0, 0x02a0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x02a0,
    0x02a0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x2aa8, 0x2aa8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0280, 0x0280, 0x2aa8, 0x2aa8,
    0x0280, 0x0280, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x2aa8, 0x0000, 0x0000, 0x2aa8, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x02a8, 0x0a0a, 0x0a0a, 0x0a0a,
    0x02a8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    //0---------------------------
    0x0000, 0x0aa8, 0x280a, 0x2a0a, 0x2a8a, 0x288a, 0x28aa, 0x282a,
    0x280a, 0x0aa8, 0x0000, 0x0000, 0x0000,
    //1---------------------------
    0x0000, 0x0080, 0x00a0, 0x00aa, 0x00a0, 0x00a0, 0x00a0, 0x00a0,
    0x00a0, 0x0aaa, 0x0000, 0x0000, 0x0000,
    //2---------------------------
    0x0000, 0x02a8, 0x0a0a, 0x0a0a, 0x0a00, 0x0280, 0x00a0, 0x0028,
    0x0a0a, 0x0aaa, 0x0000, 0x0000, 0x0000,
    //2---------------------------
    0x0000, 0x02a8, 0x0a0a, 0x0a00, 0x0a00, 0x02a0, 0x0a00, 0x0a00,
    0x0a0a, 0x02a8, 0x0000, 0x0000, 0x0000,
    //4---------------------------
    0x0000, 0x0a00, 0x0a80, 0x0aa0, 0x0a28, 0x0a0a, 0x2aaa, 0x0a00,
    0x0a00, 0x2a80, 0x0000, 0x0000, 0x0000,
    //5---------------------------
    0x0000, 0x0aaa, 0x000a, 0x000a, 0x000a, 0x02aa, 0x0a00, 0x0a00,
    0x0a0a, 0x02a8, 0x0000, 0x0000, 0x0000,
    //6---------------------------
    0x0000, 0x02a0, 0x0028, 0x000a, 0x000a, 0x02aa, 0x0a0a, 0x0a0a,
    0x0a0a, 0x02a8, 0x0000, 0x0000, 0x0000,
    //7---------------------------
    0x0000, 0x2aaa, 0x280a, 0x280a, 0x2800, 0x0a00, 0x0280, 0x00a0,
    0x00a0, 0x00a0, 0x0000, 0x0000, 0x0000,
    //8---------------------------
    0x0000, 0x02a8, 0x0a0a, 0x0a0a, 0x0a2a, 0x02a8, 0x0a8a, 0x0a0a,
    0x0a0a, 0x02a8, 0x0000, 0x0000, 0x0000,
    //9---------------------------
    0x0000, 0x02a8, 0x0a0a, 0x0a0a, 0x0a0a, 0x0aa8, 0x0280, 0x0280,
    0x00a0, 0x00a8, 0x0000, 0x0000, 0x0000,
};
