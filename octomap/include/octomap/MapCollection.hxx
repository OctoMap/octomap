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
    // FIXME: memory leak, else we run into double frees in, e.g., the viewer...

    // for(typename std::vector<MAPNODE*>::iterator it= nodes.begin(); it != nodes.end(); ++it)
    //   delete *it;
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
  MAPNODE* MapCollection<MAPNODE>::addNode(const Pointcloud& cloud, point3d sensor_origin) {
    // TODO...
    return 0;
  }

  template <class MAPNODE>
  bool MapCollection<MAPNODE>::removeNode(const MAPNODE* n) {
    // TODO...
    return false;
  }

  template <class MAPNODE>
  MAPNODE* MapCollection<MAPNODE>::queryNode(const point3d& p) {
    for (const_iterator it = this->begin(); it != this->end(); ++it) {
      point3d ptrans = (*it)->getOrigin().inv().transform(p);
      typename MAPNODE::TreeType::NodeType* n = (*it)->getMap()->search(ptrans);
      if (!n) continue;
      if ((*it)->getMap()->isNodeOccupied(n)) return (*it);
    }
    return 0;
  }

  template <class MAPNODE>
  bool MapCollection<MAPNODE>::isOccupied(const point3d& p) const {
    for (const_iterator it = this->begin(); it != this->end(); ++it) {
      point3d ptrans = (*it)->getOrigin().inv().transform(p);
      typename MAPNODE::TreeType::NodeType* n = (*it)->getMap()->search(ptrans);
      if (!n) continue;
      if ((*it)->getMap()->isNodeOccupied(n)) return true;
    }
    return false;
  }

  template <class MAPNODE>
  bool MapCollection<MAPNODE>::isOccupied(float x, float y, float z) const {
    point3d q(x,y,z);
    return this->isOccupied(q);
  }


  template <class MAPNODE>
  float MapCollection<MAPNODE>::getOccupancy(const point3d& p) {
    float max_occ_val = 0;
    bool is_unknown = true;
    for (const_iterator it = this->begin(); it != this->end(); ++it) {
      point3d ptrans = (*it)->getOrigin().inv().transform(p);
      typename MAPNODE::TreeType::NodeType* n = (*it)->getMap()->search(ptrans);
      if (n) {
        float occ = n->getOccupancy();
        if (occ > max_occ_val) max_occ_val = occ;
        is_unknown = false;
      }
    }
    if (is_unknown) return 0.5;
    return max_occ_val;
  }


  template <class MAPNODE>
  bool MapCollection<MAPNODE>::castRay(const point3d& origin, const point3d& direction, point3d& end,
                                       bool ignoreUnknownCells, double maxRange) const {
    bool hit_obstacle = false;
    double min_dist = 1e6;
    // SPEEDUP: use openMP to do raycasting in parallel
    // SPEEDUP: use bounding boxes to determine submaps 
    for (const_iterator it = this->begin(); it != this->end(); ++it) {
      point3d origin_trans = (*it)->getOrigin().inv().transform(origin);
      point3d direction_trans = (*it)->getOrigin().inv().rot().rotate(direction);
      printf("ray from %.2f,%.2f,%.2f in dir %.2f,%.2f,%.2f in node %s\n",
             origin_trans.x(), origin_trans.y(), origin_trans.z(),
             direction_trans.x(), direction_trans.y(), direction_trans.z(),
             (*it)->getId().c_str());
      point3d temp_endpoint;
      if ((*it)->getMap()->castRay(origin_trans, direction_trans, temp_endpoint, ignoreUnknownCells, maxRange)) {
        printf("hit obstacle in node %s\n", (*it)->getId().c_str());
        double current_dist =  origin_trans.distance(temp_endpoint);
        if (current_dist < min_dist) {
          min_dist = current_dist;
          end = (*it)->getOrigin().transform(temp_endpoint);
        }
        hit_obstacle = true;
      } // end if hit obst
    } // end for
    return hit_obstacle;
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
      outfile << "MAPNODEPOSE " << origin.x() << " " << origin.y() << " " << origin.z() << " "
              << origin.roll() << " " << origin.pitch() << " " << origin.yaw() << std::endl;
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

  template <class MAPNODE>
  MAPNODE* MapCollection<MAPNODE>::queryNode(std::string id) {
    for (const_iterator it = this->begin(); it != this->end(); ++it) {
      if ((*it)->getId() == id) return *(it);
    }
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
  void MapCollection<MAPNODE>::splitPathAndFilename(std::string &filenamefullpath, 
                                                    std::string* path, std::string *filename) {
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
  std::string MapCollection<MAPNODE>::combinePathAndFilename(std::string path, std::string filename) {
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
  bool MapCollection<MAPNODE>::readTagValue(std::string tag, std::ifstream& infile, std::string* value) {
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
    } 
    else return false;
  }

} // namespace
