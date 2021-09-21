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
#include <octomap/ColorOcTree.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <list>

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << " input.(ot|bt|cot) [output.ot]\n\n";

  std::cerr << "This tool converts between OctoMap octree file formats, \n"
      "e.g. to convert old legacy files to the new .ot format or to convert \n"
      "between .bt and .ot files. The default output format is .ot.\n\n";

  exit(0);
}

int main(int argc, char** argv) {
  string inputFilename = "";
  string outputFilename = "";

  if (argc < 2 || argc > 3 || (argc > 1 && strcmp(argv[1], "-h") == 0)){
    printUsage(argv[0]);
  }

  inputFilename = std::string(argv[1]);
  if (argc == 3)
    outputFilename = std::string(argv[2]);
  else{
    outputFilename = inputFilename + ".ot";
  }


  cout << "\nReading OcTree file\n===========================\n";
  std::ifstream file(inputFilename.c_str(), std::ios_base::in |std::ios_base::binary);

  if (!file.is_open()){
    OCTOMAP_ERROR_STR("Filestream to "<< inputFilename << " not open, nothing read.");
    exit(-1);
  }

  std::istream::pos_type streampos = file.tellg();
  AbstractOcTree* tree;

  // reading binary:
  if (inputFilename.length() > 3 && (inputFilename.compare(inputFilename.length()-3, 3, ".bt") == 0)){
    OcTree* binaryTree = new OcTree(0.1);

    if (binaryTree->readBinary(file) && binaryTree->size() > 1)
      tree = binaryTree;
    else {
      OCTOMAP_ERROR_STR("Could not detect binary OcTree format in file.");
      exit(-1);

    }
  } else {
    tree = AbstractOcTree::read(file);
    if (!tree){
      OCTOMAP_WARNING_STR("Could not detect OcTree in file, trying legacy formats.");
      // TODO: check if .cot extension, try old format only then
      // reset and try old ColorOcTree format:
      file.clear(); // clear eofbit of istream
      file.seekg(streampos);
      ColorOcTree* colorTree = new ColorOcTree(0.1);
      colorTree->readData(file);
      if (colorTree->size() > 1 && file.good()){
        OCTOMAP_WARNING_STR("Detected Binary ColorOcTree to convert. \nPlease check and update the new file header (resolution will likely be wrong).");
        tree = colorTree;
      } else{
        delete colorTree;
        std::cerr << "Error reading from file " << inputFilename << std::endl;
        exit(-1);
      }
    }


  }

  // close filestream
  file.close();


  if (outputFilename.length() > 3 && (outputFilename.compare(outputFilename.length()-3, 3, ".bt") == 0)){
    std::cerr << "Writing binary (BonsaiTree) file" << std::endl;
    AbstractOccupancyOcTree* octree = dynamic_cast<AbstractOccupancyOcTree*>(tree);
    if (octree){
      if (!octree->writeBinary(outputFilename)){
        std::cerr << "Error writing to " << outputFilename << std::endl;
        exit(-2);
      }
    } else {
      std::cerr << "Error: Writing to .bt is not supported for this tree type: " << tree->getTreeType() << std::endl;
      exit(-2);
    }
  } else{
    std::cerr << "Writing general OcTree file" << std::endl;
    if (!tree->write(outputFilename)){
      std::cerr << "Error writing to " << outputFilename << std::endl;
      exit(-2);
    }
  }





  std::cout << "Finished writing to " << outputFilename << std::endl;
  
  return 0;
}
