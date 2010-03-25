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

namespace octomap {

  template <typename T>
  OcTreeDataNode<T>::OcTreeDataNode()
   : itsChildren(NULL)
  {

  }

  template <typename T>
  OcTreeDataNode<T>::OcTreeDataNode(T initVal)
   : itsChildren(NULL), value(initVal)
  {

  }

  template <typename T>
  OcTreeDataNode<T>::~OcTreeDataNode()
  {
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

  template <typename T>
  OcTreeDataNode<T>* OcTreeDataNode<T>::newNode() const{
    return new OcTreeDataNode<T>();
  }

  template <typename T>
  bool OcTreeDataNode<T>::createChild(unsigned int i) {
    if (itsChildren == NULL) {
      allocChildren();
    }
    assert (itsChildren[i] == NULL);
    itsChildren[i] = newNode();
    return true;
  }

  template <typename T>
  bool OcTreeDataNode<T>::childExists(unsigned int i) const {
    assert(i < 8);
    if ((itsChildren != NULL) && (itsChildren[i] != NULL))
      return true;
    else
      return false;
  }

  template <typename T>
  OcTreeDataNode<T>* OcTreeDataNode<T>::getChild(unsigned int i) {
    assert((i < 8) && (itsChildren != NULL));
    assert(itsChildren[i] != NULL);
    return itsChildren[i];
  }

  template <typename T>
  const OcTreeDataNode<T>* OcTreeDataNode<T>::getChild(unsigned int i) const {
    assert((i < 8) && (itsChildren != NULL));
    assert(itsChildren[i] != NULL);
    return itsChildren[i];
  }

  template <typename T>
  bool OcTreeDataNode<T>::hasChildren() const {
    if (itsChildren == NULL) return false;
    for (unsigned int i = 0; i<8; i++)
      if (childExists(i)) return true;
    return false;
  }

  // ============================================================
  // =  pruning           =======================================
  // ============================================================


  template <typename T>
  bool OcTreeDataNode<T>::collapsible() const {
    // all children must exist, must not have children of
    // their own and have the same occupancy probability
    if (!childExists(0) || getChild(0)->hasChildren())
      return false;

    T childValue = getChild(0)->getValue();

    for (unsigned int i = 1; i<8; i++) {
      if (!childExists(i)) return false;
      else if (getChild(i)->hasChildren()) return false;
      else if (getChild(i)->getValue() != childValue) return false;
    }
    return true;
  }

  template <typename T>
  bool OcTreeDataNode<T>::pruneNode() {

    if (!this->collapsible())
      return false;

    // set value to children's values (all assumed equal)
    setValue(getChild(0)->getValue());

    // delete children
    for (unsigned int i=0;i<8;i++) {
      delete itsChildren[i];
    }
    delete[] itsChildren;
    itsChildren = NULL;

    return true;
  }

  template <typename T>
  void OcTreeDataNode<T>::expandNode() {
    assert(!hasChildren());

    for (unsigned int k=0; k<8; k++) {
      createChild(k);
      itsChildren[k]->setValue(value);
    }
  }

  // ============================================================
  // =  File IO           =======================================
  // ============================================================

  template <typename T>
  std::istream& OcTreeDataNode<T>::readValue(std::istream &s) {

    char children_char;

    // read data:
    s.read((char*) &value, sizeof(value));

    s.read((char*)&children_char, sizeof(char));


    std::bitset<8> children ((unsigned long) children_char);

//    std::cout << "read: " << log_odds_occupancy << " "
//                << children.to_string<char,std::char_traits<char>,std::allocator<char> >() << std::endl;


    for (unsigned int i=0; i<8; i++) {
      if (children[i] == 1){
        createChild(i);
        getChild(i)->readValue(s);
      }
    }

    return s;
  }


  template <typename T>
  std::ostream& OcTreeDataNode<T>::writeValue(std::ostream &s) const{

    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;

    for (unsigned int i=0; i<8; i++) {
      if (childExists(i))
        children[i] = 1;
      else
        children[i] = 0;
    }


    char children_char = (char) children.to_ulong();


    s.write((const char*) &value, sizeof(value));
    s.write((char*)&children_char, sizeof(char));

//    std::cout << "wrote: " << log_odds_occupancy << " "
//            << children.to_string<char,std::char_traits<char>,std::allocator<char> >() << std::endl;

    // write children's children
    for (unsigned int i=0; i<8; i++) {
      if (children[i] == 1) {
        this->getChild(i)->writeValue(s);
      }
    }

    return s;
  }


  // ============================================================
  // =  private methodes  =======================================
  // ============================================================
  template <typename T>
  void OcTreeDataNode<T>::allocChildren() {
    itsChildren = new OcTreeDataNode<T>*[8];
    for (unsigned int i=0; i<8; i++) {
      itsChildren[i] = NULL;
    }
  }


} // end namespace

