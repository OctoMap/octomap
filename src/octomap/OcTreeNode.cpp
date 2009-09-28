// $Id$

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

#include "OcTreeNode.h"

namespace octomap {


  OcTreeNode::OcTreeNode()
    : log_odds_occupancy(0),  itsChildren(NULL) {

    setLabel(UNKNOWN);
    setDelta(true);
  }

  OcTreeNode::~OcTreeNode(){
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

  void OcTreeNode::allocChildren() {
    itsChildren = new OcTreeNode*[8];
    for (unsigned int i=0; i<8; i++) {
      itsChildren[i] = NULL;
    }
  }

  bool OcTreeNode::childExists(unsigned int i) const {
    assert(i < 8);
    if ((itsChildren != NULL) && (itsChildren[i] != NULL)) return true;
    else return false;
  }

  OcTreeNode* OcTreeNode::getChild(unsigned int i) {
    assert((i < 8) && (itsChildren != NULL));
    assert(itsChildren[i] != NULL);
    return itsChildren[i];
  }

  const OcTreeNode* OcTreeNode::getChild(unsigned int i) const{
    assert((i < 8) && (itsChildren != NULL));
    assert(itsChildren[i] != NULL);
    return itsChildren[i];
  }

  bool OcTreeNode::hasChildren() const {
    if (itsChildren == NULL) return false;
    for (unsigned int i = 0; i<8; i++)
      if (childExists(i)) return true;
    return false;
  }

  bool OcTreeNode::collapsible() const{
    // all children must exist and they
    // must not have children of their own
    // children may not contain delta information
    for (unsigned int i = 0; i<8; i++) {
      if (!childExists(i)) return false;
      else if (getChild(i)->hasChildren()) return false;
      else if (getChild(i)->isDelta()) return false;
    }
    return true;
  }

  char OcTreeNode::commonChildLabel() const{
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

  bool OcTreeNode::createChild(unsigned int i) {
    if (itsChildren == NULL) {
      allocChildren();
    }
    itsChildren[i] = new OcTreeNode();
    return true;
  }

  // ============================================================
  // =  data              =======================================
  // ============================================================

  bool OcTreeNode::isDelta() const {
    return (data & 4);
  }

  void OcTreeNode::setDelta(bool a) {
    if (a) data |=  4; // set
    else   data &= ~4; // clear
  }

  char OcTreeNode::getLabel() const {
    char retval = 0;
    if (data & 1) retval += 1;
    if (data & (1<<1)) retval += 2;
    return retval;
  }

  void OcTreeNode::setLabel(char l) {
    if (l & 1) data |=  1; // set
    else       data &= ~1; // clear
    if (l & 2) data |=  2; // set
    else       data &= ~2; // clear
  }

  void OcTreeNode::integrateHit() {
    updateLogOdds(PROB_HIT);
  }

  void OcTreeNode::integrateMiss() {
    updateLogOdds(PROB_MISS);
  }

  double OcTreeNode::logodds(double p) const {
    return log(p/(1-p));
  }


  void OcTreeNode::updateLogOdds(double p) {

    if (!isDelta()) printf("WARNING: updating passive node!\n"); // REMOVE FOR RELEASE

//     printf("logodds before: %f\n", log_odds_occupancy);
    log_odds_occupancy += logodds(p);
//     printf("logodds after : %f\n\n", log_odds_occupancy);

    if (!hasChildren() &&
        ((log_odds_occupancy > CLAMPING_THRES_MAX) || (log_odds_occupancy < CLAMPING_THRES_MIN))) {
      this->convertToBinary();
    }

  }

  double OcTreeNode::getOccupancy() const {
    if (isDelta()) {
      return 1. - ( 1. / (1. + exp(log_odds_occupancy)) );
    }
    else {
      std::cerr << "Warning: getOccupancy on pure binary node\n";
      exit(0);
    }
  }

  double OcTreeNode::getMeanChildLogOdds() const{
    double mean = 0;
    char c = 0;
    for (unsigned int i=0; i<8; i++) {
      if (childExists(i)) {
        mean += getChild(i)->getOccupancy();
        c++;
      }
    }
    if (c) mean /= (double) c;
    return log(mean/(1-mean));
  }


  double OcTreeNode::getMaxChildLogOdds() const{
    double max = -1e6;
    for (unsigned int i=0; i<8; i++) {
      if (childExists(i)) {
        double l = getChild(i)->getLogOdds();
        if (l > max) max = l;
      }
    }
    return max;
  }


  bool OcTreeNode::isOccupied() const {
    if (isDelta()) return (this->getOccupancy() >= ML_OCC_PROB_THRES); // TODO speedup logodds >= 0 ?
    else return (getLabel() == OCCUPIED);
  }


  void OcTreeNode::convertToDelta() {
    assert(!isDelta());

    // converting leaf
    if (!hasChildren()) {
      if (isOccupied()) setLogOdds(CLAMPING_THRES_MAX);
      else              setLogOdds(CLAMPING_THRES_MIN);
    }

    setDelta(true);
  }

  void OcTreeNode::convertToBinary() {
    assert(isDelta());

    // converting node
    if (hasChildren()) {


      // REMOVE FOR RELEASE
      for (unsigned int i=0; i<8; i++) {
	if (childExists(i) && (getChild(i)->isDelta())) {
	  printf("convertToBinary:: node has delta children. This should not happen.\n");
	  exit(0);
	  return;
	}
      }


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

    setDelta(false);
  }


  // TODO use label (char) instead of bool?
  bool OcTreeNode::labelMatches(bool occupied) const{
    assert(!isDelta());

    return ((occupied && (getLabel() == OCCUPIED)) ||
	    (!occupied && getLabel() == FREE));
  }

  // ============================================================
  // =  pruning           =======================================
  // ============================================================


  bool OcTreeNode::pruneNode() {

    if (!this->collapsible()) return false;

    // this node is binary itself, children can not be delta
    return pruneBinary();
  }

  // pre-condition: node has binary children who have no children
  // post-condition:
  // children deleted, node's label is common label of children
  bool OcTreeNode::pruneBinary() {

    char common_label = this->getChild(0)->getLabel();
    for (unsigned int i=1; i<8; i++) {
      if (this->getChild(i)->getLabel() != common_label) return false;
    }

    // prune children
    for (unsigned int i=0;i<8;i++) {
      if (itsChildren[i] != NULL) delete itsChildren[i];
    }
    delete[] itsChildren;
    itsChildren = NULL;

    // convert pruned node to binary leaf
    this->setLabel(common_label);
    //     printf("%d)", common_label);
    if (isDelta()) convertToBinary();

    return true;

  }


  // ============================================================
  // =  file I/O          =======================================
  // ============================================================

  std::istream& OcTreeNode::readBinary(std::istream &s) {

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
        getChild(i)->setDelta(false);
      }
      else if ((child1to4[i*2] == 0) && (child1to4[i*2+1] == 1)) {
        // child is occupied leaf
        createChild(i);
        getChild(i)->setLabel(OCCUPIED);
        getChild(i)->setDelta(false);
      }
      else if ((child1to4[i*2] == 1) && (child1to4[i*2+1] == 1)) {
        // child has children
        createChild(i);
        getChild(i)->setLabel(UNKNOWN);
        getChild(i)->setDelta(false);
      }
      // child is unkown, we leave it uninitialized
    }
    for (unsigned int i=0; i<4; i++) {
      if ((child5to8[i*2] == 1) && (child5to8[i*2+1] == 0)) {
        // child is free leaf
        createChild(i+4);
        getChild(i+4)->setLabel(FREE);
        getChild(i+4)->setDelta(false);
      }
      else if ((child5to8[i*2] == 0) && (child5to8[i*2+1] == 1)) {
        // child is occupied leaf
        createChild(i+4);
        getChild(i+4)->setLabel(OCCUPIED);
        getChild(i+4)->setDelta(false);
      }
      else if ((child5to8[i*2] == 1) && (child5to8[i*2+1] == 1)) {
        // child has children
        createChild(i+4);
        getChild(i+4)->setLabel(UNKNOWN); // set occupancy when all children have been read
        getChild(i+4)->setDelta(false);
      }
      // child is unkown, we leave it uninitialized
    }


    // read children's children and set the label
    for (unsigned int i=0; i<8; i++) {
      if (this->childExists(i)) {
        OcTreeNode* child = this->getChild(i);
        if (child->getLabel() == UNKNOWN) {

          child->readBinary(s);

          char child_label = UNKNOWN;
          for (unsigned int j=0; j<8; j++) {
            if (child->childExists(j)) {
              OcTreeNode* grand_child = child->getChild(j);
              char label = grand_child->getLabel();
              if      (child_label == UNKNOWN) child_label = label;
              else if (child_label != label)   child_label = MIXED;
	    }
	  } // end for grand children

          child->setLabel(child_label);
        }
      } // end if child exists
    } // end for children

    return s;
  }

  std::ostream& OcTreeNode::writeBinary(std::ostream &s) {

    // 2 bits for each children, 8 children per node -> 16 bits
    std::bitset<8> child1to4;
    std::bitset<8> child5to8;

    // 10 : child is free node
    // 01 : child is occupied node
    // 00 : child is unkown node
    // 11 : child has children


    // speedup: only set bits to 1, rest is init with 0 anyway,
    //          can be one logic expression per bit

    for (unsigned int i=0; i<4; i++) {
      if (childExists(i)) {
        OcTreeNode* child = this->getChild(i);
        if      (child->hasChildren())        { child1to4[i*2] = 1; child1to4[i*2+1] = 1; }
        else if (child->getLabel()==OCCUPIED) { child1to4[i*2] = 0; child1to4[i*2+1] = 1; }
        else                                  { child1to4[i*2] = 1; child1to4[i*2+1] = 0; }
      }
      else {
        child1to4[i*2] = 0; child1to4[i*2+1] = 0;
      }
    }

    for (unsigned int i=0; i<4; i++) {
      if (childExists(i+4)) {
        OcTreeNode* child = this->getChild(i+4);
        if      (child->hasChildren())        { child5to8[i*2] = 1; child5to8[i*2+1] = 1; }
        else if (child->getLabel()==OCCUPIED) { child5to8[i*2] = 0; child5to8[i*2+1] = 1; }
        else                                  { child5to8[i*2] = 1; child5to8[i*2+1] = 0; }
      }
      else {
        child5to8[i*2] = 0; child5to8[i*2+1] = 0;
      }
    }
    //     std::cout << "wrote: "
    // 	      << child1to4.to_string<char,std::char_traits<char>,std::allocator<char> >() << " "
    // 	      << child5to8.to_string<char,std::char_traits<char>,std::allocator<char> >() << std::endl;

    char child1to4_char = (char) child1to4.to_ulong();
    char child5to8_char = (char) child5to8.to_ulong();

    s.write((char*)&child1to4_char, sizeof(char));
    s.write((char*)&child5to8_char, sizeof(char));

    // write children's children
    for (unsigned int i=0; i<8; i++) {
      if (childExists(i)) {
        OcTreeNode* child = this->getChild(i);
        if (child->hasChildren()) {
          child->writeBinary(s);
        }
      }
    }

    return s;
  }

} // end namespace


