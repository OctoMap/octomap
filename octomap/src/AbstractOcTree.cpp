/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
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


#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/CountingOcTree.h>


namespace octomap {
  AbstractOcTree::AbstractOcTree(){

  }

  bool AbstractOcTree::write(const std::string& filename) const{
     std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);

     if (!file.is_open()){
       OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing written.");
       return false;
     } else {
       // TODO: check is_good of finished stream, return
       write(file);
       file.close();
     }

     return true;
   }


  bool AbstractOcTree::write(std::ostream &s) const{
    s << fileHeader <<"\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
    s << "id " << getTreeType() << std::endl;
    s << "size "<< size() << std::endl;
    s << "res " << getResolution() << std::endl;
    s << "data" << std::endl;

    // write the actual data:
    writeData(s);
    // TODO: ret.val, checks stream?
    return true;
  }

  AbstractOcTree* AbstractOcTree::read(const std::string& filename){
    std::ifstream file(filename.c_str(), std::ios_base::in |std::ios_base::binary);

    if (!file.is_open()){
      OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing read.");
      return NULL;
    } else {
      // TODO: check is_good of finished stream, warn?
      return read(file);
    }
  }


  AbstractOcTree* AbstractOcTree::read(std::istream &s){

    // check if first line valid:
    std::string line;
    std::getline(s, line);
    if (line.compare(0,fileHeader.length(), fileHeader) !=0){
      OCTOMAP_ERROR_STR("First line of OcTree file header does not start with \""<< fileHeader);
      return NULL;
    }

    std::string id;
    unsigned size;
    double res;
    if (!AbstractOcTree::readHeader(s, id, size, res))
      return NULL;


    // otherwise: values are valid, stream is now at binary data!
    OCTOMAP_DEBUG_STR("Reading octree type "<< id);

    AbstractOcTree* tree = createTree(id, res);

    if (tree){
      if (size > 0)
        tree->readData(s);

      OCTOMAP_DEBUG_STR("Done ("<< tree->size() << " nodes)");
    }

    return tree;
  }

  bool AbstractOcTree::readHeader(std::istream& s, std::string& id, unsigned& size, double& res){
    id = "";
    size = 0;
    res = 0.0;

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
      return false;
    }

    if (id == "") {
      OCTOMAP_ERROR_STR("Error reading OcTree header, ID not set");
      return false;
    }

    if (res <= 0.0) {
      OCTOMAP_ERROR_STR("Error reading OcTree header, res <= 0.0");
      return false;
    }
    // fix deprecated id value:
    if (id == "1"){
      OCTOMAP_WARNING("You are using a deprecated id \"%s\", changing to \"OcTree\" (you should update your file header)\n", id.c_str());
      id = "OcTree";
    }

    return true;

  }

  AbstractOcTree* AbstractOcTree::createTree(const std::string class_name, double res){
    std::map<std::string, AbstractOcTree*>::iterator it = classIDMapping().find(class_name);
    if (it == classIDMapping().end()){
      OCTOMAP_ERROR("Could not create octree of type %s, not in store in classIDMapping\n", class_name.c_str());
      return NULL;
    } else {
      AbstractOcTree* tree = it->second->create();

      tree->setResolution(res);
      return tree;
    }
  }

  std::map<std::string, AbstractOcTree*>& AbstractOcTree::classIDMapping(){
    // we will "leak" the memory of the map and all trees until program exits,
    // but this ensures all static objects are there as long as needed
    // http://www.parashift.com/c++-faq-lite/ctors.html#faq-10.15
    static std::map<std::string, AbstractOcTree*>* map = new std::map<std::string, AbstractOcTree*>();
    return *map;
  }

  void AbstractOcTree::registerTreeType(AbstractOcTree* tree){
    classIDMapping()[tree->getTreeType()] = tree;
  }


  const std::string AbstractOcTree::fileHeader = "# Octomap OcTree file";
}
