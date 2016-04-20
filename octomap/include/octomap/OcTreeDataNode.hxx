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

namespace octomap {

  template <typename T>
  OcTreeDataNode<T>::OcTreeDataNode()
   : children(NULL)
  {

  }

  template <typename T>
  OcTreeDataNode<T>::OcTreeDataNode(T initVal)
   : children(NULL), value(initVal)
  {

  }

  template <typename T>
  OcTreeDataNode<T>::OcTreeDataNode(const OcTreeDataNode<T>& rhs)
   : children(NULL), value(rhs.value)
  {
    if (rhs.children != NULL){
      allocChildren();
      for (unsigned i = 0; i<8; ++i){
        if (rhs.children[i] != NULL)
          children[i] = new OcTreeDataNode<T>(*(static_cast<OcTreeDataNode<T>*>(rhs.children[i])));

      }
    }
  }
  
  template <typename T>
  OcTreeDataNode<T>::~OcTreeDataNode()
  {
    // Delete only own members. OcTree maintains tree structure and must have deleted 
    // children already
    assert(children == NULL);
  }
  
  template <typename T>
  void OcTreeDataNode<T>::copyData(const OcTreeDataNode<T>& from){
    value = from.value;     
  }

  template <typename T>
  bool OcTreeDataNode<T>::operator== (const OcTreeDataNode<T>& rhs) const{
    return rhs.value == value;
  }

  // ============================================================
  // =  children          =======================================
  // ============================================================


  template <typename T>
  bool OcTreeDataNode<T>::childExists(unsigned int i) const {
    assert(i < 8);
    if ((children != NULL) && (children[i] != NULL))
      return true;
    else
      return false;
  }
  
  template <typename T>
  bool OcTreeDataNode<T>::hasChildren() const {
    if (children == NULL)
      return false;
    for (unsigned int i = 0; i<8; i++){
      // fast check, we know children != NULL
      if (children[i] != NULL)
        return true;
    }
    return false;
  }


  // ============================================================
  // =  File IO           =======================================
  // ============================================================

  template <typename T>
  std::istream& OcTreeDataNode<T>::readData(std::istream &s) {
    s.read((char*) &value, sizeof(value));
    return s;
  }


  template <typename T>
  std::ostream& OcTreeDataNode<T>::writeData(std::ostream &s) const{
    s.write((const char*) &value, sizeof(value));
    return s;
  }


  // ============================================================
  // =  private methodes  =======================================
  // ============================================================
  template <typename T>
  void OcTreeDataNode<T>::allocChildren() {
    children = new AbstractOcTreeNode*[8];
    for (unsigned int i=0; i<8; i++) {
      children[i] = NULL;
    }
  }


} // end namespace

