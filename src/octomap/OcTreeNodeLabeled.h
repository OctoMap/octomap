#ifndef OCTOMAP_OCTREE_NODE_LABELED_H
#define OCTOMAP_OCTREE_NODE_LABELED_H

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

#include "OcTreeNode.h"

namespace octomap {


  /**
   *   Node class storing a label as additional information
   */
  class OcTreeNodeLabeled : public OcTreeNode {

  public:

    enum Label {FREE=0, OCCUPIED=1, MIXED=2, UNKNOWN=3};

  public:

    OcTreeNodeLabeled();
    virtual ~OcTreeNodeLabeled();


    // -- children  ----------------------------------

    virtual bool createChild(unsigned int i);
    virtual OcTreeNodeLabeled* getChild(unsigned int i);
    virtual const OcTreeNodeLabeled* getChild(unsigned int i) const;


    //! set maximum likelihood label and call OcTreeNode::toMaxLikelihood
    virtual void toMaxLikelihood();
 
    // data  -----------------------------------------

    /**
     * set a label
     */
    void setLabel(Label l);
    /**
     * @return Label of node
     */
    Label getLabel() const;

    //! example implementation of additional information on a binary level
    void setAuxFlag(bool a);
    bool getAuxFlag() const;

    //! file reading method which set maximum likelihood labels
    virtual std::istream& readBinary(std::istream &s);


  protected:

    Label commonChildLabel() const;

    char data; // store label and state

  };



} // end namespace



#endif
