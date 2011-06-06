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


namespace octomap {

  template <class NODE>
  bool OcTreeFileIO::write(const OcTreeBase<NODE>* tree, const std::string& filename){
    std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);

    if (!file.is_open()){
      OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing written.");
      return false;
    } else {
      // TODO: check is_good of finished stream, return
      write(tree, file);
      file.close();
    }

    return true;
  }

  template <class NODE>
  std::ostream& OcTreeFileIO::write(const OcTreeBase<NODE>* tree, std::ostream& s){
    // write header:
    s << "# Octomap OcTree file\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
    s << "id " << getTreeID(tree) << std::endl;
    s << "size "<< tree->size() << std::endl;
    s << "res " << tree->getResolution() << std::endl;
    s << "data" << std::endl;
    tree->writeConst(s);

   return s;
  }

  template <class NODE>
  OcTreeBase<NODE>* OcTreeFileIO::read(const std::string& filename){
    std::ifstream file(filename.c_str(), std::ios_base::in |std::ios_base::binary);

    if (!file.is_open()){
      OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing read.");
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
      OCTOMAP_ERROR_STR("First line of OcTree file header should start with \"# Octomap\", but reads:");
      OCTOMAP_ERROR_STR(line << "\n");
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
        OCTOMAP_WARNING_STR("Unknown keyword in OcTree header, skipping: "<<token);
        char c;
        do {
          c = s.get();
        } while(s.good() && (c != '\n'));

      }

    }

    if (!headerRead) {
      OCTOMAP_ERROR_STR("Error reading OcTree header");
      return s;
    }

    if (id == 0) {
      OCTOMAP_ERROR_STR("Error reading OcTree header, ID 0");
      return s;
    }

    if (res <= 0.0) {
      OCTOMAP_ERROR_STR("Error reading OcTree header, res <= 0.0");
      return s;
    }

    // otherwise: values are valid, stream is now at binary data!
    OCTOMAP_DEBUG_STR("Reading OcTree type "<< id);

    tree = createTree<NODE>(id, res);

    if (tree){
      tree->read(s);
    }

    OCTOMAP_DEBUG_STR("Done ("<< tree->size() << " nodes)");

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
      default: OCTOMAP_ERROR_STR( __PRETTY_FUNCTION__ << ": Unknown Octree id "<< id<<"."); break;
    }

    return tree;
  }

  template <class NODE>
  unsigned OcTreeFileIO::getTreeID(const OcTreeBase<NODE>* tree){
    // check actual type of tree:
    if (dynamic_cast<const OcTree*>(tree)){
      return 1;
    } else if (dynamic_cast<const OcTreeBase<OcTreeDataNode<float> >*>(tree)){
      return 2;
    } else {
      OCTOMAP_ERROR_STR( __PRETTY_FUNCTION__ << ": Unknown Octree type "<< typeid(tree).name()<<".");
    }
    return 0;
  }




}
