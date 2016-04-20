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

#include <octomap/ColorOcTree.h>

namespace octomap {


  // node implementation  --------------------------------------
  std::ostream& ColorOcTreeNode::writeData(std::ostream &s) const {
    s.write((const char*) &value, sizeof(value)); // occupancy
    s.write((const char*) &color, sizeof(Color)); // color
    
    return s;
  }

  std::istream& ColorOcTreeNode::readData(std::istream &s) {        
    s.read((char*) &value, sizeof(value)); // occupancy
    s.read((char*) &color, sizeof(Color)); // color
    
    return s;
  }

  ColorOcTreeNode::Color ColorOcTreeNode::getAverageChildColor() const {
    int mr = 0;
    int mg = 0;
    int mb = 0;
    int c = 0;
    
    if (children != NULL){
      for (int i=0; i<8; i++) {
        ColorOcTreeNode* child = static_cast<ColorOcTreeNode*>(children[i]);
        
        if (child != NULL && child->isColorSet()) {
          mr += child->getColor().r;
          mg += child->getColor().g;
          mb += child->getColor().b;
          ++c;
        }
      }
    }
    
    if (c > 0) {
      mr /= c;
      mg /= c;
      mb /= c;
      return Color((uint8_t) mr, (uint8_t) mg, (uint8_t) mb);
    }
    else { // no child had a color other than white
      return Color(255, 255, 255);
    }
  }


  void ColorOcTreeNode::updateColorChildren() {      
    color = getAverageChildColor();
  }


  // tree implementation  --------------------------------------
  ColorOcTree::ColorOcTree(double resolution)
  : OccupancyOcTreeBase<ColorOcTreeNode>(resolution) {
    colorOcTreeMemberInit.ensureLinking();
  };

  ColorOcTreeNode* ColorOcTree::setNodeColor(const OcTreeKey& key, 
                                             uint8_t r, 
                                             uint8_t g, 
                                             uint8_t b) {
    ColorOcTreeNode* n = search (key);
    if (n != 0) {
      n->setColor(r, g, b); 
    }
    return n;
  }
  
  bool ColorOcTree::pruneNode(ColorOcTreeNode* node) {
    if (!isNodeCollapsible(node)) 
      return false;

    // set value to children's values (all assumed equal)
    node->copyData(*(getNodeChild(node, 0)));
    
    if (node->isColorSet()) // TODO check
      node->setColor(node->getAverageChildColor());

    // delete children
    for (unsigned int i=0;i<8;i++) {
      deleteNodeChild(node, i);
    }
    delete[] node->children;
    node->children = NULL;

    return true;
  }
  
  bool ColorOcTree::isNodeCollapsible(const ColorOcTreeNode* node) const{
    // all children must exist, must not have children of
    // their own and have the same occupancy probability
    if (!nodeChildExists(node, 0))
      return false;
    
    const ColorOcTreeNode* firstChild = getNodeChild(node, 0);
    if (nodeHasChildren(firstChild))
      return false;

    for (unsigned int i = 1; i<8; i++) {
      // compare nodes only using their occupancy, ignoring color for pruning
      if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
        return false;
    }
    
    return true;
  }

  ColorOcTreeNode* ColorOcTree::averageNodeColor(const OcTreeKey& key, 
                                                 uint8_t r, 
                                                 uint8_t g, 
                                                 uint8_t b) {
    ColorOcTreeNode* n = search(key);
    if (n != 0) {
      if (n->isColorSet()) {
        ColorOcTreeNode::Color prev_color = n->getColor();
        n->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2); 
      }
      else {
        n->setColor(r, g, b);
      }
    }
    return n;
  }

  ColorOcTreeNode* ColorOcTree::integrateNodeColor(const OcTreeKey& key, 
                                                   uint8_t r, 
                                                   uint8_t g, 
                                                   uint8_t b) {
    ColorOcTreeNode* n = search (key);
    if (n != 0) {
      if (n->isColorSet()) {
        ColorOcTreeNode::Color prev_color = n->getColor();
        double node_prob = n->getOccupancy();
        uint8_t new_r = (uint8_t) ((double) prev_color.r * node_prob 
                                               +  (double) r * (0.99-node_prob));
        uint8_t new_g = (uint8_t) ((double) prev_color.g * node_prob 
                                               +  (double) g * (0.99-node_prob));
        uint8_t new_b = (uint8_t) ((double) prev_color.b * node_prob 
                                               +  (double) b * (0.99-node_prob));
        n->setColor(new_r, new_g, new_b); 
      }
      else {
        n->setColor(r, g, b);
      }
    }
    return n;
  }
  
  
  void ColorOcTree::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  void ColorOcTree::updateInnerOccupancyRecurs(ColorOcTreeNode* node, unsigned int depth) {
    // only recurse and update for inner nodes:
    if (nodeHasChildren(node)){
      // return early for last level:
      if (depth < this->tree_depth){
        for (unsigned int i=0; i<8; i++) {
          if (nodeChildExists(node, i)) {
            updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
          }
        }
      }
      node->updateOccupancyChildren();
      node->updateColorChildren();
    }
  }

  void ColorOcTree::writeColorHistogram(std::string filename) {

#ifdef _MSC_VER
    fprintf(stderr, "The color histogram uses gnuplot, this is not supported under windows.\n");
#else
    // build RGB histogram
    std::vector<int> histogram_r (256,0);
    std::vector<int> histogram_g (256,0);
    std::vector<int> histogram_b (256,0);
    for(ColorOcTree::tree_iterator it = this->begin_tree(),
          end=this->end_tree(); it!= end; ++it) {
      if (!it.isLeaf() || !this->isNodeOccupied(*it)) continue;
      ColorOcTreeNode::Color& c = it->getColor();
      ++histogram_r[c.r];
      ++histogram_g[c.g];
      ++histogram_b[c.b];
    }
    // plot data
    FILE *gui = popen("gnuplot ", "w");
    fprintf(gui, "set term postscript eps enhanced color\n");
    fprintf(gui, "set output \"%s\"\n", filename.c_str());
    fprintf(gui, "plot [-1:256] ");
    fprintf(gui,"'-' w filledcurve lt 1 lc 1 tit \"r\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 2 tit \"g\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 3 tit \"b\",");
    fprintf(gui, "'-' w l lt 1 lc 1 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 2 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 3 tit \"\"\n");

    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);    
    fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);    
    fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);    
    fprintf(gui, "e\n");
    fflush(gui);
#endif
  }

  std::ostream& operator<<(std::ostream& out, ColorOcTreeNode::Color const& c) {
    return out << '(' << (unsigned int)c.r << ' ' << (unsigned int)c.g << ' ' << (unsigned int)c.b << ')';
  }


  ColorOcTree::StaticMemberInitializer ColorOcTree::colorOcTreeMemberInit;

} // end namespace

