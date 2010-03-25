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

  template <class NODE>
  bool OcTreeFileIO::write(OcTreeBase<NODE>* tree, const std::string& filename){
    std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);

    if (!file.is_open()){
      std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing written.\n";
      return false;
    } else {
      // TODO: check is_good of finished stream, return
      write(tree, file);
      file.close();
    }

    return true;
  }

  template <class NODE>
  std::ostream& OcTreeFileIO::write(OcTreeBase<NODE>* tree, std::ostream& s){
    // write header:
    s << "# Octomap OcTree file\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
    s << "id " << getTreeID(tree) << std::endl;
    s << "size "<< tree->size() << std::endl;
    s << "res " << tree->getResolution() << std::endl;
    s << "data" << std::endl;
    tree->write(s);

   return s;
  }

  template <class NODE>
  OcTreeBase<NODE>* OcTreeFileIO::read(const std::string& filename){
    std::ifstream file(filename.c_str(), std::ios_base::in |std::ios_base::binary);

    if (!file.is_open()){
      std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing written.\n";
      return NULL;
    } else {
      // TODO: check is_good of finished stream, warn?
      OcTreeBase<NODE>* tree = NULL;
      read(file,tree);
      return tree;
    }

  }

  template <class NODE>
  std::istream& OcTreeFileIO::read(std::istream& s, OcTreeBase<NODE>*& tree){
    unsigned id = 0;
    unsigned size = 0;
    double res = 0.0;
    tree = NULL;

    // check if first line valid:
    std::string line;
    std::getline(s, line);
    if (line.compare(0,9, "# Octomap OcTree file", 0, 9) !=0){
      std::cerr << "Error: First line of OcTree file header should start with \"# Octomap\", but reads:"<< std::endl;
      std::cerr << line << "\n\n";
      return s;
    }

    std::string token;
    bool headerRead = false;
    while(s.good() && !headerRead) {
      s >> token;
      if (token == "data"){
        headerRead = true;
        // skip forward until end of line:
        char c;
        do {
          c = s.get();
        } while(s.good() && (c != '\n'));
      }
      else if (token.compare(0,1,"#") == 0){
        // comment line, skip forward until end of line:
        char c;
        do {
          c = s.get();
        } while(s.good() && (c != '\n'));
      }
      else if (token == "id")
        s >> id;
      else if (token == "res")
        s >> res;
      else if (token == "size")
        s >> size;
      else{
        std::cout << "Unknown keyword in OcTree header, skipping: "<<token << std::endl;
        char c;
        do {
          c = s.get();
        } while(s.good() && (c != '\n'));

      }

    }

    if (!headerRead) {
      std::cerr << "Error reading OcTree header\n";
      return s;
    }

    if (id == 0) {
      std::cerr << "Error reading OcTree header, ID 0\n";
      return s;
    }

    if (res <= 0.0) {
      std::cerr << "Error reading OcTree header, res <= 0.0\n";
      return s;
    }

    // otherwise: values are valid, stream is now at binary data!
    std::cout << "Reading OcTree type "<< id << std::endl;

    tree = createTree<NODE>(id, res);

    if (tree){
      tree->read(s);
    }

    std::cout << "Done ("<< tree->size() << " nodes)\n";

    return s;
  }


  template <class NODE>
  OcTreeBase<NODE>* OcTreeFileIO::createTree(unsigned id, double res){
    OcTreeBase<NODE>* tree = NULL;
    switch(id) {
      case 1: tree = new OcTree(res); break;
      // needs to be fixed (only works on OcTreeNodes right now)
      //case 2: tree = new OcTreeBase<OcTreeDataNode<float> >(res); break;
      // ...
      default: std::cerr << __PRETTY_FUNCTION__ << ": Unknown Octree id "<< id<<".\n"; break;
    }

    return tree;
  }

  template <class NODE>
  unsigned OcTreeFileIO::getTreeID(OcTreeBase<NODE>* tree){
    // check actual type of tree:
    if (dynamic_cast<OcTree*>(tree)){
      return 1;
    } else if (dynamic_cast<OcTreeBase<OcTreeDataNode<float> >*>(tree)){
      return 2;
    } else {
      std::cerr << __PRETTY_FUNCTION__ << ": Unknown Octree type "<< typeid(tree).name()<<".\n";
    }
    return 0;
  }




}
