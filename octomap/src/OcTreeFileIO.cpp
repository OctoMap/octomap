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

#include <octomap/OcTreeFileIO.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/CountingOcTree.h>
#include <octomap/OcTreeStamped.h>
namespace octomap {

  OcTreeFileIO::OcTreeFileIO(){
    AbstractOcTree* tree;

    tree = new ColorOcTree(0.1);
    classIDMapping[tree->getTreeType()] = tree;
    tree = new CountingOcTree(0.1);
    classIDMapping[tree->getTreeType()] = tree;
    tree = new OcTree(0.1);
    classIDMapping[tree->getTreeType()] = tree;
    tree = new OcTreeStamped(0.1);
    classIDMapping[tree->getTreeType()] = tree;
  }

  OcTreeFileIO::~OcTreeFileIO(){
    std::map<std::string, AbstractOcTree*>::iterator it;
    for (it = classIDMapping.begin(); it != classIDMapping.end(); it++){
      delete it->second;
    }

    classIDMapping.clear();
  }

  bool OcTreeFileIO::write(const AbstractOcTree* tree, const std::string& filename){
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

  std::ostream& OcTreeFileIO::write(const AbstractOcTree* tree, std::ostream& s){
    // write header:
    s << "# Octomap OcTree file\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
    s << "id " << tree->getTreeType() << std::endl;
    s << "size "<< tree->size() << std::endl;
    s << "res " << tree->getResolution() << std::endl;
    s << "data" << std::endl;
    tree->writeConst(s);

   return s;
  }

  AbstractOcTree* OcTreeFileIO::read(const std::string& filename){
    std::ifstream file(filename.c_str(), std::ios_base::in |std::ios_base::binary);

    if (!file.is_open()){
      OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing read.");
      return NULL;
    } else {
      // TODO: check is_good of finished stream, warn?
      AbstractOcTree* tree = NULL;
      read(file,tree);
      return tree;
    }

  }

  std::istream& OcTreeFileIO::read(std::istream& s, AbstractOcTree*& tree){
    std::string id = "";
    unsigned size = 0;
    double res = 0.0;
    tree = NULL;

    // check if first line valid:
    std::string line;
    int streampos = s.tellg();
    std::getline(s, line);
    if (line.compare(0,9, "# Octomap OcTree file", 0, 9) !=0){
      OCTOMAP_ERROR_STR("First line of OcTree file header should start with \"# Octomap\", but reads:");
      OCTOMAP_ERROR_STR(line << "\n");

      OCTOMAP_WARNING_STR("Could not detect OcTree in file, trying legacy formats.");
      // try reading old .bt and .cot directly, reset stream
      s.seekg(streampos);

      OcTree* binaryTree = new OcTree(0.1);
      binaryTree->readBinary(s);
      if (binaryTree->size() > 1 && s.good()){
        tree = binaryTree;
        OCTOMAP_WARNING_STR("Detected Binary OcTree. Please update your files to the new format.");
      } else{
        // reset and try again, this time ColorOcTree:
        delete binaryTree;
        s.seekg(streampos);
        ColorOcTree* colorTree = new ColorOcTree(0.1);
        colorTree->read(s);
        if (colorTree->size() > 1 && s.good()){
          tree = colorTree;
          OCTOMAP_WARNING_STR("Detected Binary OcTree. Please update your files to the new format.");
        }
        // else: failure, tree will be NULL
      }


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

    if (id == "") {
      OCTOMAP_ERROR_STR("Error reading OcTree header, ID not set");
      return s;
    }
    // fix deprecated id value:
    if (id == "1"){
      OCTOMAP_WARNING("You are using a deprecated id \"%s\", changing to \"OcTree\" (you should update your file header)\n", id.c_str());
      id = "OcTree";
    }

    if (res <= 0.0) {
      OCTOMAP_ERROR_STR("Error reading OcTree header, res <= 0.0");
      return s;
    }

    // otherwise: values are valid, stream is now at binary data!
    OCTOMAP_DEBUG_STR("Reading octree type "<< id);

    tree = createTree(id, res);

    if (tree){
      tree->read(s);
      OCTOMAP_DEBUG_STR("Done ("<< tree->size() << " nodes)");
    }


    return s;
  }


  AbstractOcTree* OcTreeFileIO::createTree(const std::string class_name, double res){
    std::map<std::string, AbstractOcTree*>::iterator it =classIDMapping.find(class_name);
    if (it == classIDMapping.end()){
      OCTOMAP_ERROR("Could not create octree of type %s, not in store in OcTreeFileIO\n", class_name.c_str());
      return NULL;
    } else {
      AbstractOcTree* tree = it->second->create();

      tree->setResolution(res);
      return tree;
    }
  }

}
