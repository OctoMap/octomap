// $Id$

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009, K. M. Wurm, A. Hornung, University of Freiburg
 * All rights reserved.
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

#include <bitset>
#include <cassert>
#include <fstream> // REMOVE

#include <octomap/OcTreeNodeLabeled.h>

namespace octomap {


  OcTreeNodeLabeled::OcTreeNodeLabeled()
    : OcTreeNode() {
    setLabel(UNKNOWN);
    setAuxFlag(false);
  }

  OcTreeNodeLabeled::~OcTreeNodeLabeled(){
    // children are deleted in parent destructor
  }


  // ============================================================
  // =  children          =======================================
  // ============================================================

  bool OcTreeNodeLabeled::createChild(unsigned int i) {
    if (itsChildren == NULL) {
      allocChildren();
    }
    itsChildren[i] = new OcTreeNodeLabeled();
    return true;
  }


  OcTreeNodeLabeled* OcTreeNodeLabeled::getChild(unsigned int i) {
    assert((i < 8) && (itsChildren != NULL));
    assert(itsChildren[i] != NULL);
    return (OcTreeNodeLabeled*) itsChildren[i];
  }

  const OcTreeNodeLabeled* OcTreeNodeLabeled::getChild(unsigned int i) const{
    assert((i < 8) && (itsChildren != NULL));
    assert(itsChildren[i] != NULL);
    return (const OcTreeNodeLabeled*) itsChildren[i];
  }

  
  // ============================================================
  // =  data              =======================================
  // ============================================================

  void OcTreeNodeLabeled::setLabel(OcTreeNodeLabeled::Label label) {
    char l = (char) label;
    if (l & 1) data |=  1; // set
    else       data &= ~1; // clear
    if (l & 2) data |=  2; // set
    else       data &= ~2; // clear
  }


  OcTreeNodeLabeled::Label OcTreeNodeLabeled::getLabel() const {
    char retval = 0;
    if (data & 1) retval += 1;
    if (data & (1<<1)) retval += 2;
    return (Label) retval;
  }


  OcTreeNodeLabeled::Label OcTreeNodeLabeled::commonChildLabel() const{
    Label common_label = UNKNOWN;
    for (unsigned int i=0; i<8; i++) {

      if (childExists(i)) {

        // REMOVE FOR RELEASE
        if (!getChild(i)->atThreshold()) {
          printf("commonChildLabel:: node has delta children (no %d). This should not happen.\n", i);
        }

        if (common_label == UNKNOWN) common_label = getChild(i)->getLabel();
        else if (this->getChild(i)->getLabel() != common_label) return MIXED;
      }
      else { // "unknown" child
        if (UNKOWN_AS_OBSTACLE && (common_label != UNKNOWN)) return MIXED;
      }
    }
    return common_label;
  }


  void OcTreeNodeLabeled::toMaxLikelihood() {

    // set maximum likelihood label

    if (hasChildren()) {  // update inner node

      setLabel(commonChildLabel());

      // REMOVE FOR RELEASE
      if (getLabel() == UNKNOWN) {
	printf("toMaxLikelihood:: label is set to UNKOWN. This should not happen.\n");
	return;
      }

      //      printf("{%d}", getLabel());
    }

    // update leaf node
    else {
      if (isOccupied()) setLabel(OCCUPIED);
      else              setLabel(FREE);
    }

    OcTreeNode::toMaxLikelihood();  // update occupancy probability

  }


  bool OcTreeNodeLabeled::getAuxFlag() const {
    return (data & 4);
  }


  void OcTreeNodeLabeled::setAuxFlag(bool a) {
    if (a) data |=  4; // set
    else   data &= ~4; // clear
  }

  // ============================================================
  // =  file I/O          =======================================
  // ============================================================

  std::istream& OcTreeNodeLabeled::readBinary(std::istream &s) {

    char child1to4_char;
    char child5to8_char;
    s.read((char*)&child1to4_char, sizeof(char));
    s.read((char*)&child5to8_char, sizeof(char));

    std::bitset<8> child1to4 ((unsigned long) child1to4_char);
    std::bitset<8> child5to8 ((unsigned long) child5to8_char);

    //     std::cout << "read:  "
    // 	      << child1to4.to_string<char,std::char_traits<char>,std::allocator<char> >() << " "
    // 	      << child5to8.to_string<char,std::char_traits<char>,std::allocator<char> >() << std::endl;

    for (unsigned int i=0; i<4; i++) {
      if ((child1to4[i*2] == 1) && (child1to4[i*2+1] == 0)) {
        // child is free leaf
        createChild(i);
        getChild(i)->setLabel(FREE);
        getChild(i)->setLogOdds(clampingThresMin);
      }
      else if ((child1to4[i*2] == 0) && (child1to4[i*2+1] == 1)) {
        // child is occupied leaf
        createChild(i);
        getChild(i)->setLabel(OCCUPIED);
        getChild(i)->setLogOdds(clampingThresMax);
      }
      else if ((child1to4[i*2] == 1) && (child1to4[i*2+1] == 1)) {
        // child has children
        createChild(i);
        getChild(i)->setLabel(UNKNOWN);
      }
      // child is unkown, we leave it uninitialized
    }
    for (unsigned int i=0; i<4; i++) {
      if ((child5to8[i*2] == 1) && (child5to8[i*2+1] == 0)) {
        // child is free leaf
        createChild(i+4);
        getChild(i+4)->setLabel(FREE);
        getChild(i+4)->setLogOdds(clampingThresMin);
      }
      else if ((child5to8[i*2] == 0) && (child5to8[i*2+1] == 1)) {
        // child is occupied leaf
        createChild(i+4);
        getChild(i+4)->setLabel(OCCUPIED);
        getChild(i+4)->setLogOdds(clampingThresMax);
      }
      else if ((child5to8[i*2] == 1) && (child5to8[i*2+1] == 1)) {
        // child has children
        createChild(i+4);
        getChild(i+4)->setLabel(UNKNOWN); // set occupancy when all children have been read
      }
      // child is unkown, we leave it uninitialized
    }


    // read children's children and set the label
    for (unsigned int i=0; i<8; i++) {
      if (this->childExists(i)) {
        OcTreeNodeLabeled* child = this->getChild(i);
        if (child->getLabel() == UNKNOWN) {

          child->readBinary(s);

          Label child_label = UNKNOWN;
          for (unsigned int j=0; j<8; j++) {
            if (child->childExists(j)) {
              OcTreeNodeLabeled* grand_child = child->getChild(j);
              Label label = grand_child->getLabel();
              if      (child_label == UNKNOWN) child_label = label;
              else if (child_label != label)   child_label = MIXED;
	    }
	  } // end for grand children

          child->setLabel(child_label);
          child->setLogOdds(child->getMaxChildLogOdds());
        }
      } // end if child exists
    } // end for children

    return s;
  }

} // end namespace


