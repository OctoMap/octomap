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

#include <string>
#include <fstream>
#include <iostream>
#include <octomap/octomap.h>
#include <cstdlib>
#include <cstring>

using namespace std;
using namespace octomap;


int main(int argc, char **argv)
{

    //bool rotate = false;       // Fix orientation of webots-exported files
    bool show_help = false;
    string outputFilename("");
    string inputFilename("");
//    double minX = 0.0;
//    double minY = 0.0;
//    double minZ = 0.0;
//    double maxX = 0.0;
//    double maxY = 0.0;
//    double maxZ = 0.0;
//    bool applyBBX = false;
//    bool applyOffset = false;
    octomap::point3d offset(0.0, 0.0, 0.0);

    double scale = 1.0;
    double res = -1.0;

    if(argc == 1) show_help = true;
    for(int i = 1; i < argc && !show_help; i++) {
        if(strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ||
           strcmp(argv[i], "--usage") == 0 || strcmp(argv[i], "-usage") == 0 ||
           strcmp(argv[i], "-h") == 0
          )
               show_help = true;
    }
    
    if(show_help) {
        cout << "Usage: "<<argv[0]<<" [OPTIONS] <filename.bt>" << endl;
        cout << "\tOPTIONS:" << endl;
        cout << "\t -o <file>        Output filename (default: first input filename + .bt)" << endl;
//        cout << "\t --mark-free      Mark not occupied cells as 'free' (default: unknown)" << endl;
//        cout << "\t --rotate         Rotate left by 90 deg. to fix the coordinate system when exported from Webots" << endl;
        cout << "\t --offset <x> <y> <z>: add an offset to the octree coordinates (translation)\n";
//        cout << "\t --bb <minx> <miny> <minz> <maxx> <maxy> <maxz>: force bounding box for OcTree" << endl;
        cout << "\t --res <resolution>: set ressolution of OcTree to new value\n";
        cout << "\t --scale <scale>: scale  octree resolution by a value\n";
        exit(0);
    }
    
    for(int i = 1; i < argc; i++) {
      // Parse command line arguments
      if(strcmp(argv[i], "--rotate") == 0) {
        OCTOMAP_WARNING_STR(argv[i] << " not yet implemented!\n");
        //rotate = true;
        continue;
      } else if(strcmp(argv[i], "-o") == 0 && i < argc - 1) {
        i++;
        outputFilename = argv[i];
        continue;
      } else if (strcmp(argv[i], "--bb") == 0 && i < argc - 7) {
        OCTOMAP_WARNING_STR(argv[i] << " not yet implemented!\n");
        i++;
        //minX = atof(argv[i]);
        i++;
        //minY = atof(argv[i]);
        i++;
        //minZ = atof(argv[i]);
        i++;
        //maxX = atof(argv[i]);
        i++;
        //maxY = atof(argv[i]);
        i++;
        //maxZ = atof(argv[i]);

        //applyBBX = true;

        continue;
      } else if (strcmp(argv[i], "--res") == 0 && i < argc - 1) {
        i++;
        res = atof(argv[i]);

        continue;
      } else if (strcmp(argv[i], "--scale") == 0 && i < argc - 1) {
        i++;
        scale = atof(argv[i]);

        continue;
      } else if (strcmp(argv[i], "--offset") == 0 && i < argc - 4) {
        OCTOMAP_WARNING_STR(argv[i] << " not yet implemented!\n");
        i++;
        offset(0) = (float) atof(argv[i]);
        i++;
        offset(1) = (float) atof(argv[i]);
        i++;
        offset(2) = (float) atof(argv[i]);

        //applyOffset = true;

        continue;
      } else if (i == argc-1){
        inputFilename = string(argv[i]);
      }
    }

    if (outputFilename == ""){
      outputFilename = inputFilename + ".edit.bt";
    }

    OcTree* tree = new OcTree(0.1);
    if (!tree->readBinary(inputFilename)){
      OCTOMAP_ERROR("Could not open file, exiting.\n");
      exit(1);
    }

    // apply scale / resolution setting:

    if (scale != 1.0){
      res = tree->getResolution() * scale;
    }

    if (res > 0.0){
      std::cout << "Setting new tree resolution: " << res << std::endl;
      tree->setResolution(res);
    }

    // TODO: implement rest (move, bounding box, rotation...)



//      size = width * height * depth;
//      double res = double(scale)/double(depth);
//
//      if(!tree) {
//        cout << "Generate labeled octree with leaf size " << res << endl << endl;
//        tree = new OcTree(res);
//      }
//
//      if (applyBBX){
//        cout << "Bounding box for Octree: [" << minX << ","<< minY << "," << minZ << " - "
//            << maxX << ","<< maxY << "," << maxZ << "]\n";
//
//      }
//      if (applyOffset){
//        std::cout << "Offset on final map: "<< offset << std::endl;
//
//      }
//
//

//            if(rotate) {
//              endpoint.rotate_IP(M_PI_2, 0.0, 0.0);
//            }
//            if (applyOffset)
//              endpoint += offset;
//
//            if (!applyBBX  || (endpoint(0) <= maxX && endpoint(0) >= minX
//                && endpoint(1) <= maxY && endpoint(1) >= minY
//                && endpoint(2) <= maxZ && endpoint(2) >= minZ)){
//
//              // mark cell in octree as free or occupied
//              if(mark_free || value == 1) {
//                tree->updateNode(endpoint, value == 1);
//              }
//            } else{
//              nr_voxels_out ++;
//            }
//          }


//
//    // prune octree
//    cout << "Prune octree" << endl << endl;
//    tree->prune();
 
    // write octree to file  

    cout << "Writing octree to " << outputFilename << endl;
   
    if (!tree->writeBinary(outputFilename)){
      OCTOMAP_ERROR("Error writing tree to %s\n", outputFilename.c_str());
      exit(1);
    }

    cout << "done" << endl << endl;
    delete tree;
    return 0;
}
