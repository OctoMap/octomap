// $Id: OcTreeNode.cpp 26 2009-09-28 15:35:22Z kai_wurm $

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: GNU GPL v2, http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*/

/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <bitset>
#include <cassert>
#include <math.h>
#include <fstream>
#include <stdlib.h>

#include "OcTreeNodeLabeled.h"

namespace octomap {


  OcTreeNodeLabeled::OcTreeNodeLabeled()
    : OcTreeNode() {

    setLabel(UNKNOWN);
  }

  OcTreeNodeLabeled::~OcTreeNodeLabeled(){
    if (itsChildren != NULL) {
      for (unsigned int i=0;i<8;i++) {
        if (itsChildren[i] != NULL) delete itsChildren[i];
      }
      delete[] itsChildren;
    }
  }


  // ============================================================
  // =  children          =======================================
  // ============================================================


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

  
  char OcTreeNodeLabeled::commonChildLabel() const{
    char common_label = UNKNOWN;
    for (unsigned int i=0; i<8; i++) {

      if (childExists(i)) {

        // REMOVE FOR RELEASE
        if (getChild(i)->isDelta()) {
          printf("commonChildLabel:: node has delta children (no %d). This should not happen.\n", i);
          exit(0);
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

  bool OcTreeNodeLabeled::createChild(unsigned int i) {
    if (itsChildren == NULL) {
      allocChildren();
    }
    itsChildren[i] = new OcTreeNodeLabeled();
    return true;
  }

  // ============================================================
  // =  data              =======================================
  // ============================================================

  bool OcTreeNodeLabeled::getAuxFlag() const {
    return (data & 4);
  }

  void OcTreeNodeLabeled::setAuxFlag(bool a) {
    if (a) data |=  4; // set
    else   data &= ~4; // clear
  }

  char OcTreeNodeLabeled::getLabel() const {
    char retval = 0;
    if (data & 1) retval += 1;
    if (data & (1<<1)) retval += 2;
    return retval;
  }

  void OcTreeNodeLabeled::setLabel(char l) {
    if (l & 1) data |=  1; // set
    else       data &= ~1; // clear
    if (l & 2) data |=  2; // set
    else       data &= ~2; // clear
  }


  void OcTreeNodeLabeled::convertToBinary() {

    // converting node
    if (hasChildren()) {
      setLabel(commonChildLabel());

      // REMOVE FOR RELEASE
      if (getLabel() == UNKNOWN) {
	printf("convertToBinary:: label is set to UNKOWN. This should not happen.\n");
	exit(0);
	return;
      }

      //      printf("{%d}", getLabel());
    }

    // converting leaf
    else {
      if (isOccupied()) setLabel(OCCUPIED);
      else              setLabel(FREE);
    }

    OcTreeNode::convertToBinary();

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
        getChild(i)->setLogOdds(CLAMPING_THRES_MIN);
      }
      else if ((child1to4[i*2] == 0) && (child1to4[i*2+1] == 1)) {
        // child is occupied leaf
        createChild(i);
        getChild(i)->setLabel(OCCUPIED);
        getChild(i)->setLogOdds(CLAMPING_THRES_MAX);
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
        getChild(i+4)->setLogOdds(CLAMPING_THRES_MIN);
      }
      else if ((child5to8[i*2] == 0) && (child5to8[i*2+1] == 1)) {
        // child is occupied leaf
        createChild(i+4);
        getChild(i+4)->setLabel(OCCUPIED);
        getChild(i+4)->setLogOdds(CLAMPING_THRES_MAX);
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

          char child_label = UNKNOWN;
          for (unsigned int j=0; j<8; j++) {
            if (child->childExists(j)) {
              OcTreeNodeLabeled* grand_child = child->getChild(j);
              char label = grand_child->getLabel();
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


