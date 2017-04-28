/**
* dynamicEDT3D:
* A library for incrementally updatable Euclidean distance transforms in 3D.
* @author C. Sprunk, B. Lau, W. Burgard, University of Freiburg, Copyright (C) 2011.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2011-2012, C. Sprunk, B. Lau, W. Burgard, University of Freiburg
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

template <class TREE>
float DynamicEDTOctomapBase<TREE>::distanceValue_Error = -1.0;

template <class TREE>
int DynamicEDTOctomapBase<TREE>::distanceInCellsValue_Error = -1;

template <class TREE>
DynamicEDTOctomapBase<TREE>::DynamicEDTOctomapBase(float maxdist, TREE* _octree, octomap::point3d bbxMin, octomap::point3d bbxMax, bool treatUnknownAsOccupied)
: DynamicEDT3D(((int) (maxdist/_octree->getResolution()+1)*((int) (maxdist/_octree->getResolution()+1)))), octree(_octree), unknownOccupied(treatUnknownAsOccupied)
{
	treeDepth = octree->getTreeDepth();
	treeResolution = octree->getResolution();
	initializeOcTree(bbxMin, bbxMax);
	octree->enableChangeDetection(true);
}

template <class TREE>
DynamicEDTOctomapBase<TREE>::~DynamicEDTOctomapBase() {

}


template <class TREE>
void DynamicEDTOctomapBase<TREE>::update(bool updateRealDist){

	for(octomap::KeyBoolMap::const_iterator it = octree->changedKeysBegin(), end=octree->changedKeysEnd(); it!=end; ++it){
		//the keys in this list all go down to the lowest level!

		octomap::OcTreeKey key = it->first;

		//ignore changes outside of bounding box
		if(key[0] < boundingBoxMinKey[0] || key[1] < boundingBoxMinKey[1] || key[2] < boundingBoxMinKey[2])
			continue;
		if(key[0] > boundingBoxMaxKey[0] || key[1] > boundingBoxMaxKey[1] || key[2] > boundingBoxMaxKey[2])
			continue;

		typename TREE::NodeType* node = octree->search(key);
		assert(node);
		//"node" is not necessarily at lowest level, BUT: the occupancy value of this node
		//has to be the same as of the node indexed by the key *it

		updateMaxDepthLeaf(key, octree->isNodeOccupied(node));
	}
	octree->resetChangeDetection();

	DynamicEDT3D::update(updateRealDist);
}

template <class TREE>
void DynamicEDTOctomapBase<TREE>::initializeOcTree(octomap::point3d bbxMin, octomap::point3d bbxMax){

    boundingBoxMinKey = octree->coordToKey(bbxMin);
    boundingBoxMaxKey = octree->coordToKey(bbxMax);

	offsetX = -boundingBoxMinKey[0];
	offsetY = -boundingBoxMinKey[1];
	offsetZ = -boundingBoxMinKey[2];

	int _sizeX = boundingBoxMaxKey[0] - boundingBoxMinKey[0] + 1;
	int _sizeY = boundingBoxMaxKey[1] - boundingBoxMinKey[1] + 1;
	int _sizeZ = boundingBoxMaxKey[2] - boundingBoxMinKey[2] + 1;

	initializeEmpty(_sizeX, _sizeY, _sizeZ, false);


	if(unknownOccupied == false){
		for(typename TREE::leaf_bbx_iterator it = octree->begin_leafs_bbx(bbxMin,bbxMax), end=octree->end_leafs_bbx(); it!= end; ++it){
			if(octree->isNodeOccupied(*it)){
				int nodeDepth = it.getDepth();
				if( nodeDepth == treeDepth){
					insertMaxDepthLeafAtInitialize(it.getKey());
				} else {
					int cubeSize = 1 << (treeDepth - nodeDepth);
					octomap::OcTreeKey key=it.getIndexKey();
					for(int dx = 0; dx < cubeSize; dx++)
						for(int dy = 0; dy < cubeSize; dy++)
							for(int dz = 0; dz < cubeSize; dz++){
								unsigned short int tmpx = key[0]+dx;
								unsigned short int tmpy = key[1]+dy;
								unsigned short int tmpz = key[2]+dz;

								if(boundingBoxMinKey[0] > tmpx || boundingBoxMinKey[1] > tmpy || boundingBoxMinKey[2] > tmpz)
									continue;
								if(boundingBoxMaxKey[0] < tmpx || boundingBoxMaxKey[1] < tmpy || boundingBoxMaxKey[2] < tmpz)
									continue;

								insertMaxDepthLeafAtInitialize(octomap::OcTreeKey(tmpx, tmpy, tmpz));
							}
				}
			}
		}
	} else {
		octomap::OcTreeKey key;
		for(int dx=0; dx<sizeX; dx++){
			key[0] = boundingBoxMinKey[0] + dx;
			for(int dy=0; dy<sizeY; dy++){
				key[1] = boundingBoxMinKey[1] + dy;
				for(int dz=0; dz<sizeZ; dz++){
					key[2] = boundingBoxMinKey[2] + dz;

					typename TREE::NodeType* node = octree->search(key);
					if(!node || octree->isNodeOccupied(node)){
						insertMaxDepthLeafAtInitialize(key);
					}
				}
			}
		}
	}
}

template <class TREE>
void DynamicEDTOctomapBase<TREE>::insertMaxDepthLeafAtInitialize(octomap::OcTreeKey key){
	bool isSurrounded = true;


	for(int dx=-1; dx<=1; dx++)
		for(int dy=-1; dy<=1; dy++)
			for(int dz=-1; dz<=1; dz++){
				if(dx==0 && dy==0 && dz==0)
					continue;
				typename TREE::NodeType* node = octree->search(octomap::OcTreeKey(key[0]+dx, key[1]+dy, key[2]+dz));
				if((!unknownOccupied && node==NULL) || ((node!=NULL) && (octree->isNodeOccupied(node)==false))){
					isSurrounded = false;
					break;
				}
			}

	if(isSurrounded){
		//obstacles that are surrounded by obstacles do not need to be put in the queues,
		//hence this initialization
		dataCell c;
		int x = key[0]+offsetX;
		int y = key[1]+offsetY;
		int z = key[2]+offsetZ;
		c.obstX = x;
		c.obstY = y;
		c.obstZ = z;
		c.sqdist = 0;
		c.dist = 0.0;
		c.queueing = fwProcessed;
		c.needsRaise = false;
		data[x][y][z] = c;
	} else {
		setObstacle(key[0]+offsetX, key[1]+offsetY, key[2]+offsetZ);
	}
}

template <class TREE>
void DynamicEDTOctomapBase<TREE>::updateMaxDepthLeaf(octomap::OcTreeKey& key, bool occupied){
	if(occupied)
		setObstacle(key[0]+offsetX, key[1]+offsetY, key[2]+offsetZ);
	else
		removeObstacle(key[0]+offsetX, key[1]+offsetY, key[2]+offsetZ);
}

template <class TREE>
void DynamicEDTOctomapBase<TREE>::worldToMap(const octomap::point3d &p, int &x, int &y, int &z) const {
	octomap::OcTreeKey key = octree->coordToKey(p);
	x = key[0] + offsetX;
	y = key[1] + offsetY;
	z = key[2] + offsetZ;
}

template <class TREE>
void DynamicEDTOctomapBase<TREE>::mapToWorld(int x, int y, int z, octomap::point3d &p) const {
	p = octree->keyToCoord(octomap::OcTreeKey(x-offsetX, y-offsetY, z-offsetZ));
}

template <class TREE>
void DynamicEDTOctomapBase<TREE>::mapToWorld(int x, int y, int z, octomap::OcTreeKey &key) const {
	key = octomap::OcTreeKey(x-offsetX, y-offsetY, z-offsetZ);
}

template <class TREE>
void DynamicEDTOctomapBase<TREE>::getDistanceAndClosestObstacle(const octomap::point3d& p, float &distance, octomap::point3d& closestObstacle) const {
	int x,y,z;
	worldToMap(p, x, y, z);
	if(x>=0 && x<sizeX && y>=0 && y<sizeY && z>=0 && z<sizeZ){
		dataCell c= data[x][y][z];

		distance = c.dist*treeResolution;
		if(c.obstX != invalidObstData){
			mapToWorld(c.obstX, c.obstY, c.obstZ, closestObstacle);
		} else {
		  //If we are at maxDist, it can very well be that there is no valid closest obstacle data for this cell, this is not an error.
		}
	} else {
	  distance = distanceValue_Error;
	}
}

template <class TREE>
void DynamicEDTOctomapBase<TREE>::getDistanceAndClosestObstacle_unsafe(const octomap::point3d& p, float &distance, octomap::point3d& closestObstacle) const {
	int x,y,z;
	worldToMap(p, x, y, z);

	dataCell c= data[x][y][z];

	distance = c.dist*treeResolution;
	if(c.obstX != invalidObstData){
		mapToWorld(c.obstX, c.obstY, c.obstZ, closestObstacle);
	} else {
		//If we are at maxDist, it can very well be that there is no valid closest obstacle data for this cell, this is not an error.
	}
}

template <class TREE>
float DynamicEDTOctomapBase<TREE>::getDistance(const octomap::point3d& p) const {
  int x,y,z;
  worldToMap(p, x, y, z);
  if(x>=0 && x<sizeX && y>=0 && y<sizeY && z>=0 && z<sizeZ){
      return data[x][y][z].dist*treeResolution;
  } else {
      return distanceValue_Error;
  }
}

template <class TREE>
float DynamicEDTOctomapBase<TREE>::getDistance_unsafe(const octomap::point3d& p) const {
  int x,y,z;
  worldToMap(p, x, y, z);
  return data[x][y][z].dist*treeResolution;
}

template <class TREE>
float DynamicEDTOctomapBase<TREE>::getDistance(const octomap::OcTreeKey& k) const {
  int x = k[0] + offsetX;
  int y = k[1] + offsetY;
  int z = k[2] + offsetZ;

  if(x>=0 && x<sizeX && y>=0 && y<sizeY && z>=0 && z<sizeZ){
      return data[x][y][z].dist*treeResolution;
  } else {
      return distanceValue_Error;
  }
}

template <class TREE>
float DynamicEDTOctomapBase<TREE>::getDistance_unsafe(const octomap::OcTreeKey& k) const {
  int x = k[0] + offsetX;
  int y = k[1] + offsetY;
  int z = k[2] + offsetZ;

  return data[x][y][z].dist*treeResolution;
}

template <class TREE>
int DynamicEDTOctomapBase<TREE>::getSquaredDistanceInCells(const octomap::point3d& p) const {
  int x,y,z;
  worldToMap(p, x, y, z);
  if(x>=0 && x<sizeX && y>=0 && y<sizeY && z>=0 && z<sizeZ){
    return data[x][y][z].sqdist;
  } else {
    return distanceInCellsValue_Error;
  }
}

template <class TREE>
int DynamicEDTOctomapBase<TREE>::getSquaredDistanceInCells_unsafe(const octomap::point3d& p) const {
  int x,y,z;
  worldToMap(p, x, y, z);
  return data[x][y][z].sqdist;
}

template <class TREE>
bool DynamicEDTOctomapBase<TREE>::checkConsistency() const {

	for(octomap::KeyBoolMap::const_iterator it = octree->changedKeysBegin(), end=octree->changedKeysEnd(); it!=end; ++it){
		//std::cerr<<"Cannot check consistency, you must execute the update() method first."<<std::endl;
		return false;
	}

	for(int x=0; x<sizeX; x++){
		for(int y=0; y<sizeY; y++){
			for(int z=0; z<sizeZ; z++){

				octomap::point3d point;
				mapToWorld(x,y,z,point);
				typename TREE::NodeType* node = octree->search(point);

				bool mapOccupied = isOccupied(x,y,z);
				bool treeOccupied = false;
				if(node){
					treeOccupied = octree->isNodeOccupied(node);
				} else {
					if(unknownOccupied)
						treeOccupied = true;
				}

				if(mapOccupied != treeOccupied){
					//std::cerr<<"OCCUPANCY MISMATCH BETWEEN TREE AND MAP at "<<x<<","<<y<<","<<z<<std::endl;
					//std::cerr<<"Tree "<<treeOccupied<<std::endl;
					//std::cerr<<"Map "<<mapOccupied<<std::endl;
					return false;
				}
			}
		}
	}

	return true;
}
