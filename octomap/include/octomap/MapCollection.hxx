// $Id: OcTree.cpp 252 2011-08-15 13:10:00Z ahornung $

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

#include <stdio.h>
#include <sstream>
#include <fstream>

namespace octomap {
  
  template <class MAPNODE>
  MapCollection<MAPNODE>::MapCollection() {
  }

  template <class MAPNODE>
  MapCollection<MAPNODE>::MapCollection(std::string filename) {
    this->read(filename);
  }

  template <class MAPNODE>
  MapCollection<MAPNODE>::~MapCollection() {
    this->clear();
  }

  template <class MAPNODE>
  void MapCollection<MAPNODE>::clear() {
		for(typename std::vector<MAPNODE*>::iterator it= nodes.begin(); it != nodes.end(); ++it)
			delete *it;
		nodes.clear();
  }

  template <class MAPNODE>
  bool MapCollection<MAPNODE>::read(std::string filenamefullpath) {

    std::string path;
    std::string filename;
    splitPathAndFilename(filenamefullpath, &path, &filename);

    std::ifstream infile;
    infile.open(filenamefullpath.c_str(), std::ifstream::in);
    if(!infile.is_open()){
    		OCTOMAP_ERROR_STR("Could not open "<< filenamefullpath << ". MapCollection not loaded.");
    		return false;
    }

    bool ok = true;
    while(ok){
    		std::string nodeID;
    		ok = readTagValue("MAPNODEID", infile, &nodeID);
    		if(!ok){
    			//do not throw error, you could be at the end of the file
    			break;
    		}

    		std::string mapNodeFilename;
    		ok = readTagValue("MAPNODEFILENAME", infile, &mapNodeFilename);
    		if(!ok){
    			OCTOMAP_ERROR_STR("Could not read MAPNODEFILENAME.");
    			break;
    		}

    		std::string poseStr;
    		ok = readTagValue("MAPNODEPOSE", infile, &poseStr);
    		std::istringstream poseStream(poseStr);
    		float x,y,z;
    		poseStream >> x >> y >> z;
    		double roll,pitch,yaw;
    		poseStream >> roll >> pitch >> yaw;
    		ok = ok && !poseStream.fail();
    		if(!ok){
    			OCTOMAP_ERROR_STR("Could not read MAPNODEPOSE.");
    			break;
    		}
    		octomap::pose6d origin(x, y, z, roll, pitch, yaw);

    		MAPNODE* node = new MAPNODE(combinePathAndFilename(path,mapNodeFilename), origin);
    		node->setId(nodeID);

    		if(!ok){
    			for(unsigned int i=0; i<nodes.size(); i++){
    				delete nodes[i];
    			}
    			infile.close();
    			return false;
    		} else {
    			nodes.push_back(node);
    		}
    	}
    	infile.close();
    	return true;
  }

  template <class MAPNODE>
  void MapCollection<MAPNODE>::addNode( MAPNODE* node){
  	nodes.push_back(node);
  }

  template <class MAPNODE>
  MAPNODE* MapCollection<MAPNODE>::addNode() {
    return 0;
  }

  template <class MAPNODE>
  MAPNODE* MapCollection<MAPNODE>::addNode(const Pointcloud& cloud, point3d sensor_origin) {
    return 0;
  }

  template <class MAPNODE>
  bool MapCollection<MAPNODE>::removeNode(const MAPNODE* n) {
    return false;
  }

  template <class MAPNODE>
  MAPNODE* MapCollection<MAPNODE>::queryNode(const point3d& p) {
    return 0;
  }

  template <class MAPNODE>
  bool MapCollection<MAPNODE>::isOccupied(const point3d& p) const {
    return false;
  }

  template <class MAPNODE>
  bool MapCollection<MAPNODE>::isOccupied(float x, float y, float z) const {
    return false;
  }

  template <class MAPNODE>
  bool MapCollection<MAPNODE>::castRay(const point3d& origin, const point3d& direction, point3d& end,
                              bool ignoreUnknownCells, double maxRange) const {
    return false;
  }
  
  template <class MAPNODE>
  bool MapCollection<MAPNODE>::writePointcloud(std::string filename) {
    Pointcloud pc;
    for(typename std::vector<MAPNODE* >::iterator it = nodes.begin(); it != nodes.end(); ++it){
    	Pointcloud tmp = (*it)->generatePointcloud();
    	pc.push_back(tmp);
    }

    pc.writeVrml(filename);

    return true;
  }

  template <class MAPNODE>
  bool MapCollection<MAPNODE>::write(std::string filename) {
    bool ok = true;

    std::ofstream outfile(filename.c_str());
    outfile << "#This file was generated by the write-method of MapCollection\n";

    for(typename std::vector<MAPNODE* >::iterator it = nodes.begin(); it != nodes.end(); ++it){
    	std::string id = (*it)->getId();
    	pose6d origin = (*it)->getOrigin();
    	std::string nodemapFilename = "nodemap_";
    	nodemapFilename.append(id);
    	nodemapFilename.append(".bt");


    	outfile << "MAPNODEID " << id << "\n";
    	outfile << "MAPNODEFILENAME "<< nodemapFilename << "\n";
    	outfile << "MAPNODEPOSE "<<origin.x()<<" "<<origin.y()<<" "<<origin.z()<<" "<<origin.roll()<<" "<<origin.pitch()<<" "<<origin.yaw()<<std::endl;

    	ok = ok && (*it)->writeMap(nodemapFilename);
    }
    outfile.close();

    return ok;
  }

  // TODO
  template <class MAPNODE>
  void MapCollection<MAPNODE>::insertScan(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                                 double maxrange, bool pruning, bool lazy_eval) {
    fprintf(stderr, "ERROR: MapCollection::insertScan is not implemented yet.\n");
  }

  // TODO
  template <class MAPNODE>
  MAPNODE* MapCollection<MAPNODE>::queryNode(std::string id) {
    fprintf(stderr, "ERROR: MapCollection::queryNode(string) is not implemented yet.\n");
    return 0;
  }
        
  // TODO
  template <class MAPNODE>
  std::vector<Pointcloud*> MapCollection<MAPNODE>::segment(const Pointcloud& scan) const {
    std::vector<Pointcloud*> result;
    fprintf(stderr, "ERROR: MapCollection::segment is not implemented yet.\n");
    return result;
  }

  // TODO
  template <class MAPNODE>
  MAPNODE* MapCollection<MAPNODE>::associate(const Pointcloud& scan) {
    fprintf(stderr, "ERROR: MapCollection::associate is not implemented yet.\n");
    return 0;
  }

  template <class MAPNODE>
  void MapCollection<MAPNODE>::splitPathAndFilename(std::string &filenamefullpath, std::string* path, std::string *filename){
  	#ifdef WIN32
  		std::string::size_type lastSlash = filenamefullpath.find_last_of('\\');
  	#else
  		std::string::size_type lastSlash = filenamefullpath.find_last_of('/');
  	#endif
  		if (lastSlash != std::string::npos){
  			*filename =  filenamefullpath.substr(lastSlash + 1);
  			*path = filenamefullpath.substr(0, lastSlash);
  		} else {
  			*filename = filenamefullpath;
  			*path = "";
  		}
  }

  template <class MAPNODE>
  std::string MapCollection<MAPNODE>::combinePathAndFilename(std::string path, std::string filename){
  	std::string result = path;

  	if(path != ""){
  		#ifdef WIN32
  		result.append("\\");
  		#else
  		result.append("/");
  		#endif
  	}

  	result.append(filename);
  	return result;
  }

  template <class MAPNODE>
  bool MapCollection<MAPNODE>::readTagValue(std::string tag, std::ifstream& infile, std::string* value){
  	std::string line;
    while( getline(infile, line) ){
    	if(line.length() != 0 && line[0] != '#')
   			 break;
    }

    *value = "";
    std::string::size_type firstSpace = line.find(' ');
    if(firstSpace != std::string::npos && firstSpace != line.size()-1){
    	*value = line.substr(firstSpace + 1);
    	return true;
    } else
    	return false;
  }

} // namespace
