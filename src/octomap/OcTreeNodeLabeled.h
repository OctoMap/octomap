#ifndef OCTOMAP_OCTREE_NODE_LABELED_H
#define OCTOMAP_OCTREE_NODE_LABELED_H

// $Id: OcTreeNode.h 26 2009-09-28 15:35:22Z kai_wurm $

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

#include "OcTreeNode.h"

namespace octomap {


  /**
   *   Node class storing additional information such as a label 
   */
  class OcTreeNodeLabeled : public OcTreeNode {

  public:

    enum Labels {FREE=0, OCCUPIED=1, MIXED=2, UNKNOWN=3};

  public:

    OcTreeNodeLabeled();
    virtual ~OcTreeNodeLabeled();

    virtual OcTreeNodeLabeled* getChild(unsigned int i);
    virtual const OcTreeNodeLabeled* getChild(unsigned int i) const;
    virtual bool createChild(unsigned int i);


    //! set maximum likelihood clamped logodds value and label
    virtual void convertToBinary();
 
    // data
    /**
     * set a label out of those defined in OcTreeNode::Labels (0..3)
     */
    void setLabel(char l);
    /**
     * @return label of node
     */
    char getLabel() const;

    //! example implementation of additional information on a binary level
    void setAuxFlag(bool a);
    bool getAuxFlag() const;

    //! file reading method which set maximum likelihood labels
    std::istream& readBinary(std::istream &s);


  protected:

    char commonChildLabel() const;

    char data; // store label and state

  };



} // end namespace



#endif
