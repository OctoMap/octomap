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

namespace octomap {

  template <class TREETYPE>
  MapNode<TREETYPE>::MapNode(): node_map(0) {
  }

  template <class TREETYPE>
  MapNode<TREETYPE>::MapNode(TREETYPE* in_node_map, pose6d in_origin) {
  	this->node_map = in_node_map;
  	this->origin = in_origin;
  }

  template <class TREETYPE>
  MapNode<TREETYPE>::MapNode(const Pointcloud& in_cloud, pose6d in_origin): node_map(0) {
  }

  template <class TREETYPE>
  MapNode<TREETYPE>::MapNode(std::string filename, pose6d in_origin): node_map(0){
  	readMap(filename);
  	this->origin = in_origin;
  	id = filename;
  }

  template <class TREETYPE>
  MapNode<TREETYPE>::~MapNode() {
  	clear();
  }

  template <class TREETYPE>
  void MapNode<TREETYPE>::updateMap(const Pointcloud& cloud, point3d sensor_origin) {
  }

  template <class TREETYPE>
  Pointcloud MapNode<TREETYPE>::generatePointcloud() {
    Pointcloud pc;
    point3d_list occs;
    node_map->getOccupied(occs);
    for(point3d_list::iterator it = occs.begin(); it != occs.end(); ++it){
    	pc.push_back(*it);
    }
    return pc;
  }

  template <class TREETYPE>
  void MapNode<TREETYPE>::clear(){
  	if(node_map != 0){
  		delete node_map;
  		node_map = 0;
  		id = "";
  		origin = pose6d(0.0,0.0,0.0,0.0,0.0,0.0);
  	}
  }

  template <class TREETYPE>
  bool MapNode<TREETYPE>::readMap(std::string filename){
  	if(node_map != 0)
  		delete node_map;

    node_map = new TREETYPE(0.05);
    return node_map->readBinary(filename);
  }

  template <class TREETYPE>
  bool MapNode<TREETYPE>::writeMap(std::string filename){
  	return node_map->writeBinary(filename);
  }

} // namespace
