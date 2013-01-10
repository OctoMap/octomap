/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, R. Schmitt, K.M. Wurm and A. Hornung,
 * University of Freiburg. All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <list>
#include <queue>
#include <octomap/OcTreeLUT.h>

namespace octomap {


  OcTreeLUT::OcTreeLUT(unsigned int _max_depth) :
    max_depth(_max_depth) {
    initLUT();
  }

  OcTreeLUT::~OcTreeLUT() {
  }

  void OcTreeLUT::initLUT() {

    // begin Face-Neighbors
    // LUT_N
    nf_values[0][LUT_S] = nf_values[0][LUT_N] = 2;
    nf_values[1][LUT_S] = nf_values[1][LUT_N] = 3;
    nf_values[2][LUT_S] = nf_values[2][LUT_N] = 0;
    nf_values[3][LUT_S] = nf_values[3][LUT_N] = 1;
    nf_values[4][LUT_S] = nf_values[4][LUT_N] = 6;
    nf_values[5][LUT_S] = nf_values[5][LUT_N] = 7;
    nf_values[6][LUT_S] = nf_values[6][LUT_N] = 4;
    nf_values[7][LUT_S] = nf_values[7][LUT_N] = 5;

    nf_rec_values[0][LUT_N] = LUT_NO_REC;
    nf_rec_values[1][LUT_N] = LUT_NO_REC;
    nf_rec_values[2][LUT_N] = LUT_SELF;
    nf_rec_values[3][LUT_N] = LUT_SELF;
    nf_rec_values[4][LUT_N] = LUT_NO_REC;
    nf_rec_values[5][LUT_N] = LUT_NO_REC;
    nf_rec_values[6][LUT_N] = LUT_SELF;
    nf_rec_values[7][LUT_N] = LUT_SELF;

    // LUT_S
    nf_rec_values[0][LUT_S] = LUT_SELF;
    nf_rec_values[1][LUT_S] = LUT_SELF;
    nf_rec_values[2][LUT_S] = LUT_NO_REC;
    nf_rec_values[3][LUT_S] = LUT_NO_REC;
    nf_rec_values[4][LUT_S] = LUT_SELF;
    nf_rec_values[5][LUT_S] = LUT_SELF;
    nf_rec_values[6][LUT_S] = LUT_NO_REC;
    nf_rec_values[7][LUT_S] = LUT_NO_REC;

    // LUT_E
    nf_values[0][LUT_W] = nf_values[0][LUT_E] = 1;
    nf_values[1][LUT_W] = nf_values[1][LUT_E] = 0;
    nf_values[2][LUT_W] = nf_values[2][LUT_E] = 3;
    nf_values[3][LUT_W] = nf_values[3][LUT_E] = 2;
    nf_values[4][LUT_W] = nf_values[4][LUT_E] = 5;
    nf_values[5][LUT_W] = nf_values[5][LUT_E] = 4;
    nf_values[6][LUT_W] = nf_values[6][LUT_E] = 7;
    nf_values[7][LUT_W] = nf_values[7][LUT_E] = 6;

    nf_rec_values[0][LUT_E] = LUT_NO_REC;
    nf_rec_values[1][LUT_E] = LUT_SELF;
    nf_rec_values[2][LUT_E] = LUT_NO_REC;
    nf_rec_values[3][LUT_E] = LUT_SELF;
    nf_rec_values[4][LUT_E] = LUT_NO_REC;
    nf_rec_values[5][LUT_E] = LUT_SELF;
    nf_rec_values[6][LUT_E] = LUT_NO_REC;
    nf_rec_values[7][LUT_E] = LUT_SELF;

    // LUT_W
    nf_rec_values[0][LUT_W] = LUT_SELF;
    nf_rec_values[1][LUT_W] = LUT_NO_REC;
    nf_rec_values[2][LUT_W] = LUT_SELF;
    nf_rec_values[3][LUT_W] = LUT_NO_REC;
    nf_rec_values[4][LUT_W] = LUT_SELF;
    nf_rec_values[5][LUT_W] = LUT_NO_REC;
    nf_rec_values[6][LUT_W] = LUT_SELF;
    nf_rec_values[7][LUT_W] = LUT_NO_REC;

    // LUT_F
    nf_values[0][LUT_R] = nf_values[0][LUT_F] = 4;
    nf_values[1][LUT_R] = nf_values[1][LUT_F] = 5;
    nf_values[2][LUT_R] = nf_values[2][LUT_F] = 6;
    nf_values[3][LUT_R] = nf_values[3][LUT_F] = 7;
    nf_values[4][LUT_R] = nf_values[4][LUT_F] = 0;
    nf_values[5][LUT_R] = nf_values[5][LUT_F] = 1;
    nf_values[6][LUT_R] = nf_values[6][LUT_F] = 2;
    nf_values[7][LUT_R] = nf_values[7][LUT_F] = 3;

    nf_rec_values[0][LUT_F] = LUT_NO_REC;
    nf_rec_values[1][LUT_F] = LUT_NO_REC;
    nf_rec_values[2][LUT_F] = LUT_NO_REC;
    nf_rec_values[3][LUT_F] = LUT_NO_REC;
    nf_rec_values[4][LUT_F] = LUT_SELF;
    nf_rec_values[5][LUT_F] = LUT_SELF;
    nf_rec_values[6][LUT_F] = LUT_SELF;
    nf_rec_values[7][LUT_F] = LUT_SELF;

    // LUT_R
    nf_rec_values[0][LUT_R] = LUT_SELF;
    nf_rec_values[1][LUT_R] = LUT_SELF;
    nf_rec_values[2][LUT_R] = LUT_SELF;
    nf_rec_values[3][LUT_R] = LUT_SELF;
    nf_rec_values[4][LUT_R] = LUT_NO_REC;
    nf_rec_values[5][LUT_R] = LUT_NO_REC;
    nf_rec_values[6][LUT_R] = LUT_NO_REC;
    nf_rec_values[7][LUT_R] = LUT_NO_REC;
    // end Face-Neighbors


    // begin Edge-Neighbors
    // LUT_NW
    for (int i = LUT_NW; i < LUT_SE+1; ++i) {
      nf_values[0][i] = 3;
      nf_values[1][i] = 2;
      nf_values[2][i] = 1;
      nf_values[3][i] = 0;
      nf_values[4][i] = 7;
      nf_values[5][i] = 6;
      nf_values[6][i] = 5;
      nf_values[7][i] = 4;
    }

    nf_rec_values[0][LUT_NW] = LUT_NW_TO_W;
    nf_rec_values[1][LUT_NW] = LUT_NO_REC;
    nf_rec_values[2][LUT_NW] = LUT_SELF;
    nf_rec_values[3][LUT_NW] = LUT_NW_TO_N;
    nf_rec_values[4][LUT_NW] = LUT_NW_TO_W;
    nf_rec_values[5][LUT_NW] = LUT_NO_REC;
    nf_rec_values[6][LUT_NW] = LUT_SELF;
    nf_rec_values[7][LUT_NW] = LUT_NW_TO_N;

    // LUT_NE
    nf_rec_values[0][LUT_NE] = LUT_NO_REC;
    nf_rec_values[1][LUT_NE] = LUT_NE_TO_E;
    nf_rec_values[2][LUT_NE] = LUT_NE_TO_N;
    nf_rec_values[3][LUT_NE] = LUT_SELF;
    nf_rec_values[4][LUT_NE] = LUT_NO_REC;
    nf_rec_values[5][LUT_NE] = LUT_NE_TO_E;
    nf_rec_values[6][LUT_NE] = LUT_NE_TO_N;
    nf_rec_values[7][LUT_NE] = LUT_SELF;

    // LUT_SW
    nf_rec_values[0][LUT_SW] = LUT_SELF;
    nf_rec_values[1][LUT_SW] = LUT_SW_TO_S;
    nf_rec_values[2][LUT_SW] = LUT_SW_TO_W;
    nf_rec_values[3][LUT_SW] = LUT_NO_REC;
    nf_rec_values[4][LUT_SW] = LUT_SELF;
    nf_rec_values[5][LUT_SW] = LUT_SW_TO_S;
    nf_rec_values[6][LUT_SW] = LUT_SW_TO_W;
    nf_rec_values[7][LUT_SW] = LUT_NO_REC;

    // LUT_SE
    nf_rec_values[0][LUT_SE] = LUT_SE_TO_S;
    nf_rec_values[1][LUT_SE] = LUT_SELF;
    nf_rec_values[2][LUT_SE] = LUT_NO_REC;
    nf_rec_values[3][LUT_SE] = LUT_SE_TO_E;
    nf_rec_values[4][LUT_SE] = LUT_SE_TO_S;
    nf_rec_values[5][LUT_SE] = LUT_SELF;
    nf_rec_values[6][LUT_SE] = LUT_NO_REC;
    nf_rec_values[7][LUT_SE] = LUT_SE_TO_E;

    // LUT_FN
    for (int i = LUT_FN; i < LUT_RS+1; ++i) {
      nf_values[0][i] = 6;
      nf_values[1][i] = 7;
      nf_values[2][i] = 4;
      nf_values[3][i] = 5;
      nf_values[4][i] = 2;
      nf_values[5][i] = 3;
      nf_values[6][i] = 0;
      nf_values[7][i] = 1;
    }

    nf_rec_values[0][LUT_FN] = LUT_NO_REC;
    nf_rec_values[1][LUT_FN] = LUT_NO_REC;
    nf_rec_values[2][LUT_FN] = LUT_FN_TO_N;
    nf_rec_values[3][LUT_FN] = LUT_FN_TO_N;
    nf_rec_values[4][LUT_FN] = LUT_FN_TO_F;
    nf_rec_values[5][LUT_FN] = LUT_FN_TO_F;
    nf_rec_values[6][LUT_FN] = LUT_SELF;
    nf_rec_values[7][LUT_FN] = LUT_SELF;

    // LUT_RN
    nf_rec_values[0][LUT_RN] = LUT_RN_TO_R;
    nf_rec_values[1][LUT_RN] = LUT_RN_TO_R;
    nf_rec_values[2][LUT_RN] = LUT_SELF;
    nf_rec_values[3][LUT_RN] = LUT_SELF;
    nf_rec_values[4][LUT_RN] = LUT_NO_REC;
    nf_rec_values[5][LUT_RN] = LUT_NO_REC;
    nf_rec_values[6][LUT_RN] = LUT_RN_TO_N;
    nf_rec_values[7][LUT_RN] = LUT_RN_TO_N;

    // LUT_FS
    nf_rec_values[0][LUT_FS] = LUT_FS_TO_S;
    nf_rec_values[1][LUT_FS] = LUT_FS_TO_S;
    nf_rec_values[2][LUT_FS] = LUT_NO_REC;
    nf_rec_values[3][LUT_FS] = LUT_NO_REC;
    nf_rec_values[4][LUT_FS] = LUT_SELF;
    nf_rec_values[5][LUT_FS] = LUT_SELF;
    nf_rec_values[6][LUT_FS] = LUT_FS_TO_F;
    nf_rec_values[7][LUT_FS] = LUT_FS_TO_F;

    // LUT_RS
    nf_rec_values[0][LUT_RS] = LUT_SELF;
    nf_rec_values[1][LUT_RS] = LUT_SELF;
    nf_rec_values[2][LUT_RS] = LUT_RS_TO_R;
    nf_rec_values[3][LUT_RS] = LUT_RS_TO_R;
    nf_rec_values[4][LUT_RS] = LUT_RS_TO_S;
    nf_rec_values[5][LUT_RS] = LUT_RS_TO_S;
    nf_rec_values[6][LUT_RS] = LUT_NO_REC;
    nf_rec_values[7][LUT_RS] = LUT_NO_REC;

    // LUT_FE
    for (int i = LUT_FE; i < LUT_RW+1; ++i) {
      nf_values[0][i] = 5;
      nf_values[1][i] = 4;
      nf_values[2][i] = 7;
      nf_values[3][i] = 6;
      nf_values[4][i] = 1;
      nf_values[5][i] = 0;
      nf_values[6][i] = 3;
      nf_values[7][i] = 2;
    }

    nf_rec_values[0][LUT_FE] = LUT_NO_REC;
    nf_rec_values[1][LUT_FE] = LUT_FE_TO_E;
    nf_rec_values[2][LUT_FE] = LUT_NO_REC;
    nf_rec_values[3][LUT_FE] = LUT_FE_TO_E;
    nf_rec_values[4][LUT_FE] = LUT_FE_TO_F;
    nf_rec_values[5][LUT_FE] = LUT_SELF;
    nf_rec_values[6][LUT_FE] = LUT_FE_TO_F;
    nf_rec_values[7][LUT_FE] = LUT_SELF;

    // LUT_FW
    nf_rec_values[0][LUT_FW] = LUT_FW_TO_W;
    nf_rec_values[1][LUT_FW] = LUT_NO_REC;
    nf_rec_values[2][LUT_FW] = LUT_FW_TO_W;
    nf_rec_values[3][LUT_FW] = LUT_NO_REC;
    nf_rec_values[4][LUT_FW] = LUT_SELF;
    nf_rec_values[5][LUT_FW] = LUT_FW_TO_F;
    nf_rec_values[6][LUT_FW] = LUT_SELF;
    nf_rec_values[7][LUT_FW] = LUT_FW_TO_F;

    // LUT_RE
    nf_rec_values[0][LUT_RE] = LUT_RE_TO_R;
    nf_rec_values[1][LUT_RE] = LUT_SELF;
    nf_rec_values[2][LUT_RE] = LUT_RE_TO_R;
    nf_rec_values[3][LUT_RE] = LUT_SELF;
    nf_rec_values[4][LUT_RE] = LUT_NO_REC;
    nf_rec_values[5][LUT_RE] = LUT_RE_TO_E;
    nf_rec_values[6][LUT_RE] = LUT_NO_REC;
    nf_rec_values[7][LUT_RE] = LUT_RE_TO_E;

    // LUT_RW
    nf_rec_values[0][LUT_RW] = LUT_SELF;
    nf_rec_values[1][LUT_RW] = LUT_RW_TO_R;
    nf_rec_values[2][LUT_RW] = LUT_SELF;
    nf_rec_values[3][LUT_RW] = LUT_RW_TO_R;
    nf_rec_values[4][LUT_RW] = LUT_RW_TO_W;
    nf_rec_values[5][LUT_RW] = LUT_NO_REC;
    nf_rec_values[6][LUT_RW] = LUT_RW_TO_W;
    nf_rec_values[7][LUT_RW] = LUT_NO_REC;

    // end Edge-Neighbors


    // begin Vertex-Neighbors
    // LUT_FNE
    for (int i = LUT_FNE; i < LUT_RSW+1; ++i) {
      nf_values[0][i] = 7;
      nf_values[1][i] = 6;
      nf_values[2][i] = 5;
      nf_values[3][i] = 4;
      nf_values[4][i] = 3;
      nf_values[5][i] = 2;
      nf_values[6][i] = 1;
      nf_values[7][i] = 0;
    }

    nf_rec_values[0][LUT_FNE] = LUT_NO_REC;
    nf_rec_values[1][LUT_FNE] = LUT_FNE_TO_E;
    nf_rec_values[2][LUT_FNE] = LUT_FNE_TO_N;
    nf_rec_values[3][LUT_FNE] = LUT_FNE_TO_NE;
    nf_rec_values[4][LUT_FNE] = LUT_FNE_TO_F;
    nf_rec_values[5][LUT_FNE] = LUT_FNE_TO_FE;
    nf_rec_values[6][LUT_FNE] = LUT_FNE_TO_FN;
    nf_rec_values[7][LUT_FNE] = LUT_SELF;

    // LUT_FNW
    nf_rec_values[0][LUT_FNW] = LUT_FNW_TO_W;
    nf_rec_values[1][LUT_FNW] = LUT_NO_REC;
    nf_rec_values[2][LUT_FNW] = LUT_FNW_TO_NW;
    nf_rec_values[3][LUT_FNW] = LUT_FNW_TO_N;
    nf_rec_values[4][LUT_FNW] = LUT_FNW_TO_FW;
    nf_rec_values[5][LUT_FNW] = LUT_FNW_TO_F;
    nf_rec_values[6][LUT_FNW] = LUT_SELF;
    nf_rec_values[7][LUT_FNW] = LUT_FNW_TO_FN;

    // LUT_FSE
    nf_rec_values[0][LUT_FSE] = LUT_FSE_TO_S;
    nf_rec_values[1][LUT_FSE] = LUT_FSE_TO_SE;
    nf_rec_values[2][LUT_FSE] = LUT_NO_REC;
    nf_rec_values[3][LUT_FSE] = LUT_FSE_TO_E;
    nf_rec_values[4][LUT_FSE] = LUT_FSE_TO_FS;
    nf_rec_values[5][LUT_FSE] = LUT_SELF;
    nf_rec_values[6][LUT_FSE] = LUT_FSE_TO_F;
    nf_rec_values[7][LUT_FSE] = LUT_FSE_TO_FE;

    // LUT_FSW
    nf_rec_values[0][LUT_FSW] = LUT_FSW_TO_SW;
    nf_rec_values[1][LUT_FSW] = LUT_FSW_TO_S;
    nf_rec_values[2][LUT_FSW] = LUT_FSW_TO_W;
    nf_rec_values[3][LUT_FSW] = LUT_NO_REC;
    nf_rec_values[4][LUT_FSW] = LUT_SELF;
    nf_rec_values[5][LUT_FSW] = LUT_FSW_TO_FS;
    nf_rec_values[6][LUT_FSW] = LUT_FSW_TO_FW;
    nf_rec_values[7][LUT_FSW] = LUT_FSW_TO_F;

    // LUT_RNE
    nf_rec_values[0][LUT_RNE] = LUT_RNE_TO_R;
    nf_rec_values[1][LUT_RNE] = LUT_RNE_TO_RE;
    nf_rec_values[2][LUT_RNE] = LUT_RNE_TO_RN;
    nf_rec_values[3][LUT_RNE] = LUT_SELF;
    nf_rec_values[4][LUT_RNE] = LUT_NO_REC;
    nf_rec_values[5][LUT_RNE] = LUT_RNE_TO_E;
    nf_rec_values[6][LUT_RNE] = LUT_RNE_TO_N;
    nf_rec_values[7][LUT_RNE] = LUT_RNE_TO_NE;

    // LUT_RNW
    nf_rec_values[0][LUT_RNW] = LUT_RNW_TO_RW;
    nf_rec_values[1][LUT_RNW] = LUT_RNW_TO_R;
    nf_rec_values[2][LUT_RNW] = LUT_SELF;
    nf_rec_values[3][LUT_RNW] = LUT_RNW_TO_RN;
    nf_rec_values[4][LUT_RNW] = LUT_RNW_TO_W;
    nf_rec_values[5][LUT_RNW] = LUT_NO_REC;
    nf_rec_values[6][LUT_RNW] = LUT_RNW_TO_NW;
    nf_rec_values[7][LUT_RNW] = LUT_RNW_TO_N;

    // LUT_RSE
    nf_rec_values[0][LUT_RSE] = LUT_RSE_TO_RS;
    nf_rec_values[1][LUT_RSE] = LUT_SELF;
    nf_rec_values[2][LUT_RSE] = LUT_RSE_TO_R;
    nf_rec_values[3][LUT_RSE] = LUT_RSE_TO_RE;
    nf_rec_values[4][LUT_RSE] = LUT_RSE_TO_S;
    nf_rec_values[5][LUT_RSE] = LUT_RSE_TO_SE;
    nf_rec_values[6][LUT_RSE] = LUT_NO_REC;
    nf_rec_values[7][LUT_RSE] = LUT_RSE_TO_E;

    // LUT_RSW
    nf_rec_values[0][LUT_RSW] = LUT_SELF;
    nf_rec_values[1][LUT_RSW] = LUT_RSW_TO_RS;
    nf_rec_values[2][LUT_RSW] = LUT_RSW_TO_RW;
    nf_rec_values[3][LUT_RSW] = LUT_RSW_TO_R;
    nf_rec_values[4][LUT_RSW] = LUT_RSW_TO_SW;
    nf_rec_values[5][LUT_RSW] = LUT_RSW_TO_S;
    nf_rec_values[6][LUT_RSW] = LUT_RSW_TO_W;
    nf_rec_values[7][LUT_RSW] = LUT_NO_REC;


    nf_multiple_values[LUT_N][0] =  0;
    nf_multiple_values[LUT_N][1] =  1;
    nf_multiple_values[LUT_N][2] =  4;
    nf_multiple_values[LUT_N][3] =  5;

    nf_multiple_values[LUT_S][0] =  2;
    nf_multiple_values[LUT_S][1] =  3;
    nf_multiple_values[LUT_S][2] =  6;
    nf_multiple_values[LUT_S][3] =  7;

    nf_multiple_values[LUT_E][0] =  0;
    nf_multiple_values[LUT_E][1] =  2;
    nf_multiple_values[LUT_E][2] =  4;
    nf_multiple_values[LUT_E][3] =  6;

    nf_multiple_values[LUT_W][0] =  1;
    nf_multiple_values[LUT_W][1] =  3;
    nf_multiple_values[LUT_W][2] =  5;
    nf_multiple_values[LUT_W][3] =  7;

    nf_multiple_values[LUT_F][0] =  0;
    nf_multiple_values[LUT_F][1] =  1;
    nf_multiple_values[LUT_F][2] =  2;
    nf_multiple_values[LUT_F][3] =  3;

    nf_multiple_values[LUT_R][0] =  4;
    nf_multiple_values[LUT_R][1] =  5;
    nf_multiple_values[LUT_R][2] =  6;
    nf_multiple_values[LUT_R][3] =  7;

    nf_multiple_values[LUT_NW][0] =  1;
    nf_multiple_values[LUT_NW][1] =  5;
    nf_multiple_values[LUT_NW][2] = LUT_NO_REC;
    nf_multiple_values[LUT_NW][3] = LUT_NO_REC;

    nf_multiple_values[LUT_NE][0] =  0;
    nf_multiple_values[LUT_NE][1] =  4;
    nf_multiple_values[LUT_NE][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_NE][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_SW][0] =  3;
    nf_multiple_values[LUT_SW][1] =  7;
    nf_multiple_values[LUT_SW][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_SW][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_SE][0] =  2;
    nf_multiple_values[LUT_SE][1] =  6;
    nf_multiple_values[LUT_SE][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_SE][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_FN][0] =  1;
    nf_multiple_values[LUT_FN][1] =  3;
    nf_multiple_values[LUT_FN][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_FN][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_RN][0] =  4;
    nf_multiple_values[LUT_RN][1] =  5;
    nf_multiple_values[LUT_RN][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_RN][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_FS][0] =  2;
    nf_multiple_values[LUT_FS][1] =  3;
    nf_multiple_values[LUT_FS][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_FS][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_RS][0] =  6;
    nf_multiple_values[LUT_RS][1] =  7;
    nf_multiple_values[LUT_RS][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_RS][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_FE][0] =  0;
    nf_multiple_values[LUT_FE][1] =  2;
    nf_multiple_values[LUT_FE][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_FE][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_FW][0] =  1;
    nf_multiple_values[LUT_FW][1] =  3;
    nf_multiple_values[LUT_FW][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_FW][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_RE][0] =  4;
    nf_multiple_values[LUT_RE][1] =  6;
    nf_multiple_values[LUT_RE][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_RE][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_RW][0] =  5;
    nf_multiple_values[LUT_RW][1] =  7;
    nf_multiple_values[LUT_RW][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_RW][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_FNE][0] =  0;
    nf_multiple_values[LUT_FNE][1] =  LUT_NO_REC;
    nf_multiple_values[LUT_FNE][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_FNE][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_FNW][0] =  1;
    nf_multiple_values[LUT_FNW][1] =  LUT_NO_REC;
    nf_multiple_values[LUT_FNW][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_FNW][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_FSE][0] =  2;
    nf_multiple_values[LUT_FSE][1] =  LUT_NO_REC;
    nf_multiple_values[LUT_FSE][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_FSE][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_FSW][0] =  3;
    nf_multiple_values[LUT_FSW][1] =  LUT_NO_REC;
    nf_multiple_values[LUT_FSW][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_FSW][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_RNE][0] =  4;
    nf_multiple_values[LUT_RNE][1] =  LUT_NO_REC;
    nf_multiple_values[LUT_RNE][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_RNE][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_RNW][0] =  5;
    nf_multiple_values[LUT_RNW][1] =  LUT_NO_REC;
    nf_multiple_values[LUT_RNW][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_RNW][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_RSE][0] =  6;
    nf_multiple_values[LUT_RSE][1] =  LUT_NO_REC;
    nf_multiple_values[LUT_RSE][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_RSE][3] =  LUT_NO_REC;

    nf_multiple_values[LUT_RSW][0] =  7;
    nf_multiple_values[LUT_RSW][1] =  LUT_NO_REC;
    nf_multiple_values[LUT_RSW][2] =  LUT_NO_REC;
    nf_multiple_values[LUT_RSW][3] =  LUT_NO_REC;

  };


  unsigned int OcTreeLUT::genPos(const OcTreeKey& key, const int& i) const {
    unsigned int retval = 0;
    if (key.k[0] & (1 << i)) retval += 1;
    if (key.k[1] & (1 << i)) retval += 2;
    if (key.k[2] & (1 << i)) retval += 4;
    return retval;
  }


  /*
   * used internally to generate neighbor key from a given key
   */
  void OcTreeLUT::changeKey(const int& val, OcTreeKey& key, const unsigned short int& i) const {
    switch (val) {
    case 0:
      key.k[0] &= ~(1 << i);
      key.k[1] &= ~(1 << i);
      key.k[2] &= ~(1 << i);
      break;
    case 1:
      key.k[0] |= (1 << i);
      key.k[1] &= ~(1 << i);
      key.k[2] &= ~(1 << i);
      break;
    case 2:
      key.k[0] &= ~(1 << i);
      key.k[1] |= (1 << i);
      key.k[2] &= ~(1 << i);
      break;
    case 3:
      key.k[0] |= (1 << i);
      key.k[1] |= (1 << i);
      key.k[2] &= ~(1 << i);
      break;
    case 4:
      key.k[0] &= ~(1 << i);
      key.k[1] &= ~(1 << i);
      key.k[2] |= (1 << i);
      break;
    case 5:
      key.k[0] |= (1 << i);
      key.k[1] &= ~(1 << i);
      key.k[2] |= (1 << i);
      break;
    case 6:
      key.k[0] &= ~(1 << i);
      key.k[1] |= (1 << i);
      key.k[2] |= (1 << i);
      break;
    case 7:
      key.k[0] |= (1 << i);
      key.k[1] |= (1 << i);
      key.k[2] |= (1 << i);
      break;
    }
  }


  bool OcTreeLUT::genNeighborKey(const OcTreeKey& node_key, const signed char& dir,
                                 OcTreeKey& neighbor_key) const {
    
    neighbor_key.k[0] = node_key.k[0];
    neighbor_key.k[1] = node_key.k[1];
    neighbor_key.k[2] = node_key.k[2];

    unsigned int depth = 0;
    signed char curDir = dir;
    
    signed char pos;
    while (depth < max_depth) {
      pos = static_cast<signed char>(genPos(neighbor_key, depth));
      changeKey(nf_values[pos][curDir], neighbor_key, depth);

      if (nf_rec_values[pos][curDir] != LUT_NO_REC) { // recurs
        curDir -= nf_rec_values[pos][curDir];
        depth++;
      } 
      else { 
        return true;
      }
    }

    return false;
  };



} // namespace

