/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
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

#ifndef OCTOMAP_COLOR_OCTREE_H
#define OCTOMAP_COLOR_OCTREE_H


#include <iostream>
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
      inline bool operator== (const Color &other) const {
        return (r==other.r && g==other.g && b==other.b);
      }
      inline bool operator!= (const Color &other) const {
        return (r!=other.r || g!=other.g || b!=other.b);
      }
      unsigned char r, g, b;
    };

  public:
    ColorOcTreeNode() : OcTreeNode() {}

    ColorOcTreeNode(const ColorOcTreeNode& rhs) : OcTreeNode(rhs), color(rhs.color) {}

    bool operator==(const ColorOcTreeNode& rhs) const{
      return (rhs.value == value && rhs.color == color);
    }
    
    // children
    inline ColorOcTreeNode* getChild(unsigned int i) {
      return static_cast<ColorOcTreeNode*> (OcTreeNode::getChild(i));
    }
    inline const ColorOcTreeNode* getChild(unsigned int i) const {
      return static_cast<const ColorOcTreeNode*> (OcTreeNode::getChild(i));
    }

    bool createChild(unsigned int i) {
      if (children == NULL) allocChildren();
      children[i] = new ColorOcTreeNode();
      return true;
    }

    bool pruneNode();
    void expandNode();
    
    inline Color getColor() const { return color; }
    inline void  setColor(Color c) {this->color = c; }
    inline void  setColor(unsigned char r, unsigned char g, unsigned char b) {
      this->color = Color(r,g,b); 
    }

    Color& getColor() { return color; }

    // has any color been integrated? (pure white is very unlikely...)
    inline bool isColorSet() const { 
      return ((color.r != 255) || (color.g != 255) || (color.b != 255)); 
    }

    void updateColorChildren();


    ColorOcTreeNode::Color getAverageChildColor() const;
  
    // file I/O
    std::istream& readValue (std::istream &s);
    std::ostream& writeValue(std::ostream &s) const;
    
  protected:
    Color color;
  };


  // tree definition
  class ColorOcTree : public OccupancyOcTreeBase <ColorOcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    ColorOcTree(double resolution) : OccupancyOcTreeBase<ColorOcTreeNode>(resolution) {};  
      
    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    ColorOcTree* create() const {return new ColorOcTree(resolution); }

    std::string getTreeType() const {return "ColorOcTree";}
   
    // set node color at given key or coordinate. Replaces previous color.
    ColorOcTreeNode* setNodeColor(const OcTreeKey& key, const unsigned char& r, 
                                 const unsigned char& g, const unsigned char& b);

    ColorOcTreeNode* setNodeColor(const float& x, const float& y, 
                                 const float& z, const unsigned char& r, 
                                 const unsigned char& g, const unsigned char& b) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeColor(key,r,g,b);
    }

    // integrate color measurement at given key or coordinate. Average with previous color
    ColorOcTreeNode* averageNodeColor(const OcTreeKey& key, const unsigned char& r, 
                                  const unsigned char& g, const unsigned char& b);
    
    ColorOcTreeNode* averageNodeColor(const float& x, const float& y, 
                                      const float& z, const unsigned char& r, 
                                      const unsigned char& g, const unsigned char& b) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return averageNodeColor(key,r,g,b);
    }

    // integrate color measurement at given key or coordinate. Average with previous color
    ColorOcTreeNode* integrateNodeColor(const OcTreeKey& key, const unsigned char& r, 
                                  const unsigned char& g, const unsigned char& b);
    
    ColorOcTreeNode* integrateNodeColor(const float& x, const float& y, 
                                      const float& z, const unsigned char& r, 
                                      const unsigned char& g, const unsigned char& b) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return integrateNodeColor(key,r,g,b);
    }

    // update inner nodes, sets color to average child color
    void updateInnerOccupancy();

    // uses gnuplot to plot a RGB histogram in EPS format
    void writeColorHistogram(std::string filename);
    
  protected:
    void updateInnerOccupancyRecurs(ColorOcTreeNode* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer{
       public:
         StaticMemberInitializer() {
           ColorOcTree* tree = new ColorOcTree(0.1);
           AbstractOcTree::registerTreeType(tree);
         }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer colorOcTreeMemberInit;

  };

  //! user friendly output in format (r g b)
  std::ostream& operator<<(std::ostream& out, ColorOcTreeNode::Color const& c);

} // end namespace

#endif
