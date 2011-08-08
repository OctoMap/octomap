#ifndef OCTOMAP_OCTREE_FILEIO_H
#define OCTOMAP_OCTREE_FILEIO_H

// $Id$

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


#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeDataNode.h>
#include <fstream>
#include <string>
#include <iostream>
#include <typeinfo>

namespace octomap {

  /**
   * Static class for reading and writing OcTrees to files.
   * Implements a simple Factory design pattern.
   *
   */

  class OcTreeFileIO {
  public:
    template <class NODE>
    static bool write(const OcTreeBase<NODE>* tree, const std::string& filename);


    template <class NODE>
    static std::ostream& write(const OcTreeBase<NODE>* tree, std::ostream& s);


    // TODO: non-const version: prune tree first before writing?
//    template <class NODE>
//    static bool write(OcTreeBase<NODE>* tree, const std::string& filename);
//
//    template <class NODE>
//    static std::ostream& write(OcTreeBase<NODE>* tree, std::ostream& s);

    template <class NODE>
    static OcTreeBase<NODE>* read(const std::string& filename);

    template <class NODE>
    static std::istream& read(std::istream& s, OcTreeBase<NODE>*& tree);

    /**
     * Map OcTree classes to IDs
     *
     * @param tree
     * @return unique (unsigned) ID of OcTree class
     */
    template <class NODE>
    static unsigned getTreeID(const OcTreeBase<NODE>* tree);

    /**
     * Creates a certain OcTree (factory pattern)
     *
     * @param id unique ID of OcTree
     * @param res resolution of OcTree
     * @return pointer to newly created OcTree (empty). NULL if the ID is unknown!
     */
    template <class NODE>
    static OcTreeBase<NODE>* createTree(unsigned id, double res);


  };
}

#include "octomap/OcTreeFileIO.hxx"

#endif
