#ifndef OCTOMAP_COLOR_OCTREE_H
#define OCTOMAP_COLOR_OCTREE_H

// $Id:  $

/**
 * OctoMap:
 * A probabilistic, flexible, and compact 3D mapping library for robotic systems.
 * @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009-2011
 * @see http://octomap.sourceforge.net/
 * License: New BSD License
 */

/*
 * Copyright (c) 2009-2011, K. M. Wurm, A. Hornung, University of Freiburg
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

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap {
  
  // node definition
  class ColorOcTreeNode : public OcTreeNode {    
  public:
    
    class Color {
    public:
    Color() : r(255), g(255), b(255) {}
    Color(unsigned char _r, unsigned char _g, unsigned char _b) 
      : r(_r), g(_g), b(_b) {}
      unsigned char r, g, b;
    };

  public:
    ColorOcTreeNode() : OcTreeNode() {}     
    
    // children
    inline ColorOcTreeNode* getChild(unsigned int i) {
      return static_cast<ColorOcTreeNode*> (OcTreeNode::getChild(i));
    }
    inline const ColorOcTreeNode* getChild(unsigned int i) const {
      return static_cast<const ColorOcTreeNode*> (OcTreeNode::getChild(i));
    }
    bool createChild(unsigned int i) {
      if (itsChildren == NULL) allocChildren();      
      itsChildren[i] = new ColorOcTreeNode();
      return true;
    }
    
    inline Color getColor() const { return color; }
    inline void  setColor(Color c) {this->color = color; }
    inline void  setColor(unsigned char r, unsigned char g, unsigned char b) {
      this->color = Color(r,g,b); 
    }

    // update occupancy and color of inner nodes 
    inline void updateOccupancyChildren() {      
      this->setLogOdds(this->getMaxChildLogOdds());  // conservative
      // set color to average color?
    }

    // file I/O

    // TODO: UNTESTED
    std::istream& readValue (std::istream &s);
    std::ostream& writeValue(std::ostream &s) const;
    
  protected:
    Color color;
  };


  // tree definition
  class ColorOcTree : public OccupancyOcTreeBase <ColorOcTreeNode> {

  public:
    ColorOcTree(double _resolution);
    
    // set node color at given key or coordinate. Replaces previous color.
    ColorOcTreeNode* updateColor(const OcTreeKey& key, const unsigned char& r, 
                                 const unsigned char& g, const unsigned char& b);

    ColorOcTreeNode* updateColor(const float& x, const float& y, 
                                 const float& z, const unsigned char& r, 
                                 const unsigned char& g, const unsigned char& b) {
      OcTreeKey key;
      if (!this->genKey(point3d(x,y,z), key)) return NULL;
      return updateColor(key,r,g,b);
    }

  };

} // end namespace

#endif
