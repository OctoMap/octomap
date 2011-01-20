#ifndef OCTOMAP_OCTREE_NODE_LABELED_H
#define OCTOMAP_OCTREE_NODE_LABELED_H

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
