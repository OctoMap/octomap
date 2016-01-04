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

#ifndef OCTOMAP_MAP_COLLECTION_H
#define OCTOMAP_MAP_COLLECTION_H


#include <vector>
#include <octomap/MapNode.h>

namespace octomap {

  
  template <class MAPNODE>
  class MapCollection {
  public:
    MapCollection();
    MapCollection(std::string filename);
    ~MapCollection();

    void addNode( MAPNODE* node);
    MAPNODE* addNode(const Pointcloud& cloud, point3d sensor_origin);
    bool removeNode(const MAPNODE* n);
    MAPNODE* queryNode(const point3d& p);

    bool isOccupied(const point3d& p) const;
    bool isOccupied(float x, float y, float z) const;

    double getOccupancy(const point3d& p);

    bool castRay(const point3d& origin, const point3d& direction, point3d& end,
                 bool ignoreUnknownCells=false, double maxRange=-1.0) const;

    bool writePointcloud(std::string filename);
    bool write(std::string filename);

    // TODO
    void insertScan(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                    double maxrange=-1., bool pruning=true, bool lazy_eval = false);
    // TODO
    MAPNODE* queryNode(std::string id);

    typedef typename std::vector<MAPNODE*>::iterator iterator;
    typedef typename std::vector<MAPNODE*>::const_iterator const_iterator;
    iterator begin() { return nodes.begin(); }
    iterator end()   { return nodes.end(); }
    const_iterator begin() const { return nodes.begin(); }
    const_iterator end() const { return nodes.end(); }
    size_t size() const { return nodes.size(); }
        
  protected:
    void clear();
    bool read(std::string filename);

    // TODO
    std::vector<Pointcloud*> segment(const Pointcloud& scan) const;
    // TODO
    MAPNODE* associate(const Pointcloud& scan);

    static void splitPathAndFilename(std::string &filenamefullpath, std::string* path, std::string *filename);
    static std::string combinePathAndFilename(std::string path, std::string filename);
    static bool readTagValue(std::string tag, std::ifstream &infile, std::string* value);
    
  protected:

    std::vector<MAPNODE*> nodes;
  };

} // end namespace

#include "octomap/MapCollection.hxx"

#endif
