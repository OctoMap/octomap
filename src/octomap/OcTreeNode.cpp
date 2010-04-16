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
    : OcTreeDataNode<float>(0.0)
  {
  }

  OcTreeNode::~OcTreeNode(){

  }


  OcTreeNode* OcTreeNode::getChild(unsigned i){
    return static_cast<OcTreeNode*> (OcTreeDataNode<float>::getChild(i));
  }

  const OcTreeNode* OcTreeNode::getChild(unsigned i) const{
    return static_cast<const OcTreeNode*> (OcTreeDataNode<float>::getChild(i));
  }


  // TODO: Use Curiously Recurring Template Pattern instead of copying full function
  // (same for getChild)
  bool OcTreeNode::createChild(unsigned int i) {
    if (itsChildren == NULL) {
      allocChildren();
    }
    assert (itsChildren[i] == NULL);
    itsChildren[i] = new OcTreeNode();
    return true;
  }


  // ============================================================
  // =  occupancy probability  ==================================
  // ============================================================

  void OcTreeNode::integrateHit() {
    updateLogOdds(PROB_HIT);
  }

  void OcTreeNode::integrateMiss() {
    updateLogOdds(PROB_MISS);
  }

  bool OcTreeNode::isOccupied() const {
    return (this->getOccupancy() >= OCC_PROB_THRES);
  }

  bool OcTreeNode::atThreshold() const {
    return ((value <= CLAMPING_THRES_MIN) ||
              (value >= CLAMPING_THRES_MAX));
  }

  void OcTreeNode::toMaxLikelihood() {
      if (isOccupied()) setLogOdds(CLAMPING_THRES_MAX);
      else              setLogOdds(CLAMPING_THRES_MIN);
  }

  double OcTreeNode::getOccupancy() const {
    return 1. - ( 1. / (1. + exp(value)) );
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

  void OcTreeNode::updateOccupancyChildren() {
    //      node->setLogOdds(node->getMeanChildLogOdds());
    this->setLogOdds(this->getMaxChildLogOdds());  // conservative
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


    // inner nodes default to occupied
    this->setLogOdds(CLAMPING_THRES_MAX);

    for (unsigned int i=0; i<4; i++) {
      if ((child1to4[i*2] == 1) && (child1to4[i*2+1] == 0)) {
        // child is free leaf
        createChild(i);
        getChild(i)->setLogOdds(CLAMPING_THRES_MIN);
      }
      else if ((child1to4[i*2] == 0) && (child1to4[i*2+1] == 1)) {
        // child is occupied leaf
        createChild(i);
        getChild(i)->setLogOdds(CLAMPING_THRES_MAX);
      }
      else if ((child1to4[i*2] == 1) && (child1to4[i*2+1] == 1)) {
        // child has children
        createChild(i);
        getChild(i)->setLogOdds(-200.); // child is unkown, we leave it uninitialized
      }
    }
    for (unsigned int i=0; i<4; i++) {
      if ((child5to8[i*2] == 1) && (child5to8[i*2+1] == 0)) {
        // child is free leaf
        createChild(i+4);
        getChild(i+4)->setLogOdds(CLAMPING_THRES_MIN);
      }
      else if ((child5to8[i*2] == 0) && (child5to8[i*2+1] == 1)) {
        // child is occupied leaf
        createChild(i+4);
        getChild(i+4)->setLogOdds(CLAMPING_THRES_MAX);
      }
      else if ((child5to8[i*2] == 1) && (child5to8[i*2+1] == 1)) {
        // child has children
        createChild(i+4);
        getChild(i+4)->setLogOdds(-200.); // set occupancy when all children have been read
      }
      // child is unkown, we leave it uninitialized
    }

    // read children's children and set the label
    for (unsigned int i=0; i<8; i++) {
      if (this->childExists(i)) {
        OcTreeNode* child = this->getChild(i);
        if (fabs(child->getLogOdds()+200.)<1e-3) {
          child->readBinary(s);
          child->setLogOdds(child->getMaxChildLogOdds());
        }
      } // end if child exists
    } // end for children

    return s;
  }

  std::ostream& OcTreeNode::writeBinary(std::ostream &s) const{

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
        const OcTreeNode* child = this->getChild(i);
        if      (child->hasChildren())  { child1to4[i*2] = 1; child1to4[i*2+1] = 1; }
        else if (child->isOccupied())   { child1to4[i*2] = 0; child1to4[i*2+1] = 1; }
        else                            { child1to4[i*2] = 1; child1to4[i*2+1] = 0; }
      }
      else {
        child1to4[i*2] = 0; child1to4[i*2+1] = 0;
      }
    }

    for (unsigned int i=0; i<4; i++) {
      if (childExists(i+4)) {
        const OcTreeNode* child = this->getChild(i+4);
        if      (child->hasChildren())  { child5to8[i*2] = 1; child5to8[i*2+1] = 1; }
        else if (child->isOccupied())   { child5to8[i*2] = 0; child5to8[i*2+1] = 1; }
        else                            { child5to8[i*2] = 1; child5to8[i*2+1] = 0; }
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
        const OcTreeNode* child = this->getChild(i);
        if (child->hasChildren()) {
          child->writeBinary(s);
        }
      }
    }

    return s;
  }




  // ============================================================
  // =  private methodes  =======================================
  // ============================================================


  double OcTreeNode::logodds(double p) const {
    return log(p/(1-p));
  }


  void OcTreeNode::updateLogOdds(double p) {

//     printf("logodds before: %f\n", log_odds_occupancy);
    value += logodds(p);
//     printf("logodds after : %f\n\n", log_odds_occupancy);

    if (!hasChildren() &&
        ((value > CLAMPING_THRES_MAX) ||
            (value < CLAMPING_THRES_MIN))) {
      this->toMaxLikelihood();
    }
  }



} // end namespace


