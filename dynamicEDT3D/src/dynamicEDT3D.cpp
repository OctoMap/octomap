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

#include <dynamicEDT3D/dynamicEDT3D.h>

#include <math.h>
#include <stdlib.h>

#define FOR_EACH_NEIGHBOR_WITH_CHECK(function, p, ...) \
	int x=p.x;\
	int y=p.y;\
	int z=p.z;\
	int xp1 = x+1;\
	int xm1 = x-1;\
	int yp1 = y+1;\
	int ym1 = y-1;\
	int zp1 = z+1;\
	int zm1 = z-1;\
\
	if(z<sizeZm1) function(x, y, zp1, ##__VA_ARGS__);\
	if(z>0)       function(x, y, zm1, ##__VA_ARGS__);\
\
	if(y<sizeYm1){\
		function(x, yp1, z, ##__VA_ARGS__);\
		if(z<sizeZm1) function(x, yp1, zp1, ##__VA_ARGS__);\
		if(z>0)       function(x, yp1, zm1, ##__VA_ARGS__);\
	}\
\
	if(y>0){\
		function(x, ym1, z, ##__VA_ARGS__);\
		if(z<sizeZm1) function(x, ym1, zp1, ##__VA_ARGS__);\
		if(z>0)       function(x, ym1, zm1, ##__VA_ARGS__);\
	}\
\
\
	if(x<sizeXm1){\
		function(xp1, y, z, ##__VA_ARGS__);\
		if(z<sizeZm1) function(xp1, y, zp1, ##__VA_ARGS__);\
		if(z>0)       function(xp1, y, zm1, ##__VA_ARGS__);\
\
		if(y<sizeYm1){\
			function(xp1, yp1, z, ##__VA_ARGS__);\
			if(z<sizeZm1) function(xp1, yp1, zp1, ##__VA_ARGS__);\
			if(z>0)       function(xp1, yp1, zm1, ##__VA_ARGS__);\
		}\
\
		if(y>0){\
			function(xp1, ym1, z, ##__VA_ARGS__);\
			if(z<sizeZm1) function(xp1, ym1, zp1, ##__VA_ARGS__);\
			if(z>0)       function(xp1, ym1, zm1, ##__VA_ARGS__);\
		}\
	}\
\
	if(x>0){\
		function(xm1, y, z, ##__VA_ARGS__);\
		if(z<sizeZm1) function(xm1, y, zp1, ##__VA_ARGS__);\
		if(z>0)       function(xm1, y, zm1, ##__VA_ARGS__);\
\
		if(y<sizeYm1){\
			function(xm1, yp1, z, ##__VA_ARGS__);\
			if(z<sizeZm1) function(xm1, yp1, zp1, ##__VA_ARGS__);\
			if(z>0)       function(xm1, yp1, zm1, ##__VA_ARGS__);\
		}\
\
		if(y>0){\
			function(xm1, ym1, z, ##__VA_ARGS__);\
			if(z<sizeZm1) function(xm1, ym1, zp1, ##__VA_ARGS__);\
			if(z>0)       function(xm1, ym1, zm1, ##__VA_ARGS__);\
		}\
	}

float DynamicEDT3D::distanceValue_Error = -1.0;
int DynamicEDT3D::distanceInCellsValue_Error = -1;

DynamicEDT3D::DynamicEDT3D(int _maxdist_squared) {
	sqrt2 = sqrt(2.0);
	maxDist_squared = _maxdist_squared;
	maxDist = sqrt((double) maxDist_squared);
	data = NULL;
	gridMap = NULL;
}

DynamicEDT3D::~DynamicEDT3D() {
	if (data) {
		for (int x=0; x<sizeX; x++){
			for (int y=0; y<sizeY; y++)
				delete[] data[x][y];

			delete[] data[x];
		}
		delete[] data;
	}

	if (gridMap) {
		for (int x=0; x<sizeX; x++){
			for (int y=0; y<sizeY; y++)
				delete[] gridMap[x][y];

			delete[] gridMap[x];
		}
		delete[] gridMap;
	}
}

void DynamicEDT3D::initializeEmpty(int _sizeX, int _sizeY, int _sizeZ, bool initGridMap) {
	sizeX = _sizeX;
	sizeY = _sizeY;
	sizeZ = _sizeZ;

	sizeXm1 = sizeX-1;
	sizeYm1 = sizeY-1;
	sizeZm1 = sizeZ-1;

	if (data) {
		for (int x=0; x<sizeX; x++){
			for (int y=0; y<sizeY; y++)
				delete[] data[x][y];

			delete[] data[x];
		}
		delete[] data;
	}

	data = new dataCell**[sizeX];
	for (int x=0; x<sizeX; x++){
		data[x] = new dataCell*[sizeY];
		for(int y=0; y<sizeY; y++)
			data[x][y] = new dataCell[sizeZ];
	}

	if (initGridMap) {
		if (gridMap) {
			for (int x=0; x<sizeX; x++){
				for (int y=0; y<sizeY; y++)
					delete[] gridMap[x][y];

				delete[] gridMap[x];
			}
			delete[] gridMap;
		}

		gridMap = new bool**[sizeX];
		for (int x=0; x<sizeX; x++){
			gridMap[x] = new bool*[sizeY];
			for (int y=0; y<sizeY; y++)
				gridMap[x][y] = new bool[sizeZ];
		}
	}

	dataCell c;
	c.dist = maxDist;
	c.sqdist = maxDist_squared;
	c.obstX = invalidObstData;
	c.obstY = invalidObstData;
	c.obstZ = invalidObstData;
	c.queueing = fwNotQueued;
	c.needsRaise = false;

	for (int x=0; x<sizeX; x++){
		for (int y=0; y<sizeY; y++){
			for (int z=0; z<sizeZ; z++){
				data[x][y][z] = c;
			}
		}
	}

	if (initGridMap) {
		for (int x=0; x<sizeX; x++)
			for (int y=0; y<sizeY; y++)
				for (int z=0; z<sizeZ; z++)
					gridMap[x][y][z] = 0;
	}
}

void DynamicEDT3D::initializeMap(int _sizeX, int _sizeY, int _sizeZ, bool*** _gridMap) {
	gridMap = _gridMap;
	initializeEmpty(_sizeX, _sizeY, _sizeZ, false);

	for (int x=0; x<sizeX; x++) {
		for (int y=0; y<sizeY; y++) {
			for (int z=0; z<sizeZ; z++) {
				if (gridMap[x][y][z]) {
					dataCell c = data[x][y][z];
					if (!isOccupied(x,y,z,c)) {

						bool isSurrounded = true;
						for (int dx=-1; dx<=1; dx++) {
							int nx = x+dx;
							if (nx<0 || nx>sizeX-1) continue;
							for (int dy=-1; dy<=1; dy++) {
								int ny = y+dy;
								if (ny<0 || ny>sizeY-1) continue;
								for (int dz=-1; dz<=1; dz++) {
									if (dx==0 && dy==0 && dz==0) continue;
									int nz = z+dz;
									if (nz<0 || nz>sizeZ-1) continue;

									if (!gridMap[nx][ny][nz]) {
										isSurrounded = false;
										break;
									}
								}
							}
						}
						if (isSurrounded) {
							c.obstX = x;
							c.obstY = y;
							c.obstZ = z;
							c.sqdist = 0;
							c.dist = 0;
							c.queueing = fwProcessed;
							data[x][y][z] = c;
						} else setObstacle(x,y,z);
					}
				}
			}
		}
	}
}

void DynamicEDT3D::occupyCell(int x, int y, int z) {
	gridMap[x][y][z] = 1;
	setObstacle(x,y,z);
}

void DynamicEDT3D::clearCell(int x, int y, int z) {
	gridMap[x][y][z] = 0;
	removeObstacle(x,y,z);
}

void DynamicEDT3D::setObstacle(int x, int y, int z) {
	dataCell c = data[x][y][z];
	if(isOccupied(x,y,z,c)) return;

	addList.push_back(INTPOINT3D(x,y,z));
	c.obstX = x;
	c.obstY = y;
	c.obstZ = z;
	data[x][y][z] = c;
}

void DynamicEDT3D::removeObstacle(int x, int y, int z) {
	dataCell c = data[x][y][z];
	if(isOccupied(x,y,z,c) == false) return;

	removeList.push_back(INTPOINT3D(x,y,z));
	c.obstX = invalidObstData;
	c.obstY  = invalidObstData;
	c.obstZ  = invalidObstData;
	c.queueing = bwQueued;
	data[x][y][z] = c;
}

void DynamicEDT3D::exchangeObstacles(std::vector<INTPOINT3D> points) {

	for (unsigned int i=0; i<lastObstacles.size(); i++) {
		int x = lastObstacles[i].x;
		int y = lastObstacles[i].y;
		int z = lastObstacles[i].z;

		bool v = gridMap[x][y][z];
		if (v) continue;
		removeObstacle(x,y,z);
	}

	lastObstacles.clear();

	for (unsigned int i=0; i<points.size(); i++) {
		int x = points[i].x;
		int y = points[i].y;
		int z = points[i].z;
		bool v = gridMap[x][y][z];
		if (v) continue;
		setObstacle(x,y,z);
		lastObstacles.push_back(points[i]);
	}
}

void DynamicEDT3D::update(bool updateRealDist) {
	commitAndColorize(updateRealDist);

		while (!open.empty()) {
			INTPOINT3D p = open.pop();
			int x = p.x;
			int y = p.y;
			int z = p.z;
			dataCell c = data[x][y][z];

			if(c.queueing==fwProcessed) continue;

			if (c.needsRaise) {
				// RAISE
				raiseCell(p, c, updateRealDist);
				data[x][y][z] = c;
			}
			else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,c.obstZ,data[c.obstX][c.obstY][c.obstZ])) {
				// LOWER
				propagateCell(p, c, updateRealDist);
				data[x][y][z] = c;
			}
		}
}

void DynamicEDT3D::raiseCell(INTPOINT3D &p, dataCell &c, bool updateRealDist){
	/*
	for (int dx=-1; dx<=1; dx++) {
		int nx = p.x+dx;
		if (nx<0 || nx>sizeX-1) continue;
		for (int dy=-1; dy<=1; dy++) {
			int ny = p.y+dy;
			if (ny<0 || ny>sizeY-1) continue;
			for (int dz=-1; dz<=1; dz++) {
				if (dx==0 && dy==0 && dz==0) continue;
				int nz = p.z+dz;
				if (nz<0 || nz>sizeZ-1) continue;

				inspectCellRaise(nx,ny,nz, updateRealDist);
			}
		}
	}
*/
	FOR_EACH_NEIGHBOR_WITH_CHECK(inspectCellRaise,p, updateRealDist)

	c.needsRaise = false;
	c.queueing = bwProcessed;
}

void DynamicEDT3D::inspectCellRaise(int &nx, int &ny, int &nz, bool updateRealDist){
	dataCell nc = data[nx][ny][nz];
	if (nc.obstX!=invalidObstData && !nc.needsRaise) {
		if(!isOccupied(nc.obstX,nc.obstY,nc.obstZ,data[nc.obstX][nc.obstY][nc.obstZ])) {
			open.push(nc.sqdist, INTPOINT3D(nx,ny,nz));
			nc.queueing = fwQueued;
			nc.needsRaise = true;
			nc.obstX = invalidObstData;
			nc.obstY = invalidObstData;
			nc.obstZ = invalidObstData;
			if (updateRealDist) nc.dist = maxDist;
			nc.sqdist = maxDist_squared;
			data[nx][ny][nz] = nc;
		} else {
			if(nc.queueing != fwQueued){
				open.push(nc.sqdist, INTPOINT3D(nx,ny,nz));
				nc.queueing = fwQueued;
				data[nx][ny][nz] = nc;
			}
		}
	}
}

void DynamicEDT3D::propagateCell(INTPOINT3D &p, dataCell &c, bool updateRealDist){
	c.queueing = fwProcessed;
	/*
	for (int dx=-1; dx<=1; dx++) {
		int nx = p.x+dx;
		if (nx<0 || nx>sizeX-1) continue;
		for (int dy=-1; dy<=1; dy++) {
			int ny = p.y+dy;
			if (ny<0 || ny>sizeY-1) continue;
			for (int dz=-1; dz<=1; dz++) {
				if (dx==0 && dy==0 && dz==0) continue;
				int nz = p.z+dz;
				if (nz<0 || nz>sizeZ-1) continue;

				inspectCellPropagate(nx, ny, nz, c, updateRealDist);
			}
		}
	}
	 */

	if(c.sqdist==0){
		FOR_EACH_NEIGHBOR_WITH_CHECK(inspectCellPropagate, p, c, updateRealDist)
	} else {
		int x=p.x;
		int y=p.y;
		int z=p.z;
		int xp1 = x+1;
		int xm1 = x-1;
		int yp1 = y+1;
		int ym1 = y-1;
		int zp1 = z+1;
		int zm1 = z-1;

		int dpx = (x - c.obstX);
		int dpy = (y - c.obstY);
		int dpz = (z - c.obstZ);

		//    dpy=0;
		//    dpz=0;


		if(dpz >=0 && z<sizeZm1) inspectCellPropagate(x, y, zp1, c, updateRealDist);
		if(dpz <=0 && z>0)       inspectCellPropagate(x, y, zm1, c, updateRealDist);

		if(dpy>=0 && y<sizeYm1){
			inspectCellPropagate(x, yp1, z, c, updateRealDist);
			if(dpz >=0 && z<sizeZm1) inspectCellPropagate(x, yp1, zp1, c, updateRealDist);
			if(dpz <=0 && z>0)       inspectCellPropagate(x, yp1, zm1, c, updateRealDist);
		}

		if(dpy<=0 && y>0){
			inspectCellPropagate(x, ym1, z, c, updateRealDist);
			if(dpz >=0 && z<sizeZm1) inspectCellPropagate(x, ym1, zp1, c, updateRealDist);
			if(dpz <=0 && z>0)       inspectCellPropagate(x, ym1, zm1, c, updateRealDist);
		}


		if(dpx>=0 && x<sizeXm1){
			inspectCellPropagate(xp1, y, z, c, updateRealDist);
			if(dpz >=0 && z<sizeZm1) inspectCellPropagate(xp1, y, zp1, c, updateRealDist);
			if(dpz <=0 && z>0)       inspectCellPropagate(xp1, y, zm1, c, updateRealDist);

			if(dpy>=0 && y<sizeYm1){
				inspectCellPropagate(xp1, yp1, z, c, updateRealDist);
				if(dpz >=0 && z<sizeZm1) inspectCellPropagate(xp1, yp1, zp1, c, updateRealDist);
				if(dpz <=0 && z>0)       inspectCellPropagate(xp1, yp1, zm1, c, updateRealDist);
			}

			if(dpy<=0 && y>0){
				inspectCellPropagate(xp1, ym1, z, c, updateRealDist);
				if(dpz >=0 && z<sizeZm1) inspectCellPropagate(xp1, ym1, zp1, c, updateRealDist);
				if(dpz <=0 && z>0)       inspectCellPropagate(xp1, ym1, zm1, c, updateRealDist);
			}
		}

		if(dpx<=0 && x>0){
			inspectCellPropagate(xm1, y, z, c, updateRealDist);
			if(dpz >=0 && z<sizeZm1) inspectCellPropagate(xm1, y, zp1, c, updateRealDist);
			if(dpz <=0 && z>0)       inspectCellPropagate(xm1, y, zm1, c, updateRealDist);

			if(dpy>=0 && y<sizeYm1){
				inspectCellPropagate(xm1, yp1, z, c, updateRealDist);
				if(dpz >=0 && z<sizeZm1) inspectCellPropagate(xm1, yp1, zp1, c, updateRealDist);
				if(dpz <=0 && z>0)       inspectCellPropagate(xm1, yp1, zm1, c, updateRealDist);
			}

			if(dpy<=0 && y>0){
				inspectCellPropagate(xm1, ym1, z, c, updateRealDist);
				if(dpz >=0 && z<sizeZm1) inspectCellPropagate(xm1, ym1, zp1, c, updateRealDist);
				if(dpz <=0 && z>0)       inspectCellPropagate(xm1, ym1, zm1, c, updateRealDist);
			}
		}
	}
}

void DynamicEDT3D::inspectCellPropagate(int &nx, int &ny, int &nz, dataCell &c, bool updateRealDist){
	dataCell nc = data[nx][ny][nz];
	if(!nc.needsRaise) {
		int distx = nx-c.obstX;
		int disty = ny-c.obstY;
		int distz = nz-c.obstZ;
		int newSqDistance = distx*distx + disty*disty + distz*distz;
		if(newSqDistance > maxDist_squared)
			newSqDistance = maxDist_squared;
		bool overwrite =  (newSqDistance < nc.sqdist);
		if(!overwrite && newSqDistance==nc.sqdist) {
			//the neighbor cell is marked to be raised, has no valid source obstacle
			if (nc.obstX == invalidObstData){
				overwrite = true;
			}
			else {
				//the neighbor has no valid source obstacle but the raise wave has not yet reached it
				dataCell tmp = data[nc.obstX][nc.obstY][nc.obstZ];

				if((tmp.obstX==nc.obstX && tmp.obstY==nc.obstY && tmp.obstZ==nc.obstZ)==false)
					overwrite = true;
			}
		}
		if (overwrite) {
			if(newSqDistance < maxDist_squared){
				open.push(newSqDistance, INTPOINT3D(nx,ny,nz));
				nc.queueing = fwQueued;
			}
			if (updateRealDist) {
				nc.dist = sqrt((double) newSqDistance);
			}
			nc.sqdist = newSqDistance;
			nc.obstX = c.obstX;
			nc.obstY = c.obstY;
			nc.obstZ = c.obstZ;
		}
		data[nx][ny][nz] = nc;
	}
}


float DynamicEDT3D::getDistance( int x, int y, int z ) const {
	if( (x>=0) && (x<sizeX) && (y>=0) && (y<sizeY) && (z>=0) && (z<sizeZ)){
		return data[x][y][z].dist;
	}
	else return distanceValue_Error;
}

INTPOINT3D DynamicEDT3D::getClosestObstacle( int x, int y, int z ) const {
	if( (x>=0) && (x<sizeX) && (y>=0) && (y<sizeY) && (z>=0) && (z<sizeZ)){
	  dataCell c = data[x][y][z];
	  return INTPOINT3D(c.obstX, c.obstY, c.obstZ);
	}
	else return INTPOINT3D(invalidObstData, invalidObstData, invalidObstData);
}

int DynamicEDT3D::getSQCellDistance( int x, int y, int z ) const {
	if( (x>=0) && (x<sizeX) && (y>=0) && (y<sizeY) && (z>=0) && (z<sizeZ)){
		return data[x][y][z].sqdist;
	}
	else return distanceInCellsValue_Error;
}


void DynamicEDT3D::commitAndColorize(bool updateRealDist) {
	// ADD NEW OBSTACLES
	for (unsigned int i=0; i<addList.size(); i++) {
		INTPOINT3D p = addList[i];
		int x = p.x;
		int y = p.y;
		int z = p.z;
		dataCell c = data[x][y][z];

		if(c.queueing != fwQueued){
			if (updateRealDist) c.dist = 0;
			c.sqdist = 0;
			c.obstX = x;
			c.obstY = y;
			c.obstZ = z;
			c.queueing = fwQueued;
			data[x][y][z] = c;
			open.push(0, INTPOINT3D(x,y,z));
		}
	}

	// REMOVE OLD OBSTACLES
	for (unsigned int i=0; i<removeList.size(); i++) {
		INTPOINT3D p = removeList[i];
		int x = p.x;
		int y = p.y;
		int z = p.z;
		dataCell c = data[x][y][z];

		if (isOccupied(x,y,z,c)==true) continue; // obstacle was removed and reinserted
		open.push(0, INTPOINT3D(x,y,z));
		if (updateRealDist) c.dist  = maxDist;
		c.sqdist = maxDist_squared;
		c.needsRaise = true;
		data[x][y][z] = c;
	}
	removeList.clear();
	addList.clear();
}

bool DynamicEDT3D::isOccupied(int x, int y, int z) const {
	dataCell c = data[x][y][z];
	return (c.obstX==x && c.obstY==y && c.obstZ==z);
}

bool DynamicEDT3D::isOccupied(int &x, int &y, int &z, dataCell &c) { 
	return (c.obstX==x && c.obstY==y && c.obstZ==z);
}
