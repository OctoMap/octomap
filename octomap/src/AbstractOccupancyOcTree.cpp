// $Id$

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009-2012
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009-2012, K. M. Wurm, A. Hornung, University of Freiburg
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


#include <octomap/AbstractOccupancyOcTree.h>
#include <octomap/octomap_types.h>


namespace octomap {
  AbstractOccupancyOcTree::AbstractOccupancyOcTree(){
    // some sane default values:
    setOccupancyThres(0.5);   // = 0.0 in logodds
    setProbHit(0.7);          // = 0.85 in logodds
    setProbMiss(0.4);         // = -0.4 in logodds

    setClampingThresMin(0.1192); // = -2 in log odds
    setClampingThresMax(0.971); // = 3.5 in log odds
  }

  bool AbstractOccupancyOcTree::writeBinary(const std::string& filename){
    std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

    if (!binary_outfile.is_open()){
      OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing written.");
      return false;
    }
    return writeBinary(binary_outfile);
  }

  bool AbstractOccupancyOcTree::writeBinaryConst(const std::string& filename) const{
    std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

    if (!binary_outfile.is_open()){
      OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing written.");
      return false;
    }
    writeBinaryConst(binary_outfile);
    binary_outfile.close();
    return true;
  }

  bool AbstractOccupancyOcTree::writeBinary(std::ostream &s){
    // convert to max likelihood first, this makes efficient pruning on binary data possible
    this->toMaxLikelihood();
    this->prune();
    return writeBinaryConst(s);
  }

  bool AbstractOccupancyOcTree::writeBinaryConst(std::ostream &s) const{
    // write new header first:
    s << binaryFileHeader <<"\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
    s << "id " << this->getTreeType() << std::endl;
    s << "size "<< this->size() << std::endl;
    s << "res " << this->getResolution() << std::endl;
    s << "data" << std::endl;

    writeBinaryData(s);

    if (s.good()){
      OCTOMAP_DEBUG(" done.\n");
      return true;
    } else {
      OCTOMAP_WARNING_STR("Output stream not \"good\" after writing tree");
      return false;
    }
  }

  const std::string AbstractOccupancyOcTree::binaryFileHeader = "# Octomap OcTree binary file";
}
