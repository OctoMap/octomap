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

#ifndef OCTOMAP_OCTREE_KEY_H
#define OCTOMAP_OCTREE_KEY_H

/* According to c++ standard including this header has no practical effect
 * but it can be used to determine the c++ standard library implementation.
 */
#include <ciso646>

#include <assert.h>

/* Libc++ does not implement the TR1 namespace, all c++11 related functionality
 * is instead implemented in the std namespace.
 */
#if defined(__GNUC__) && ! defined(_LIBCPP_VERSION)
  #include <tr1/unordered_set>
  #include <tr1/unordered_map>
  namespace octomap {
    namespace unordered_ns = std::tr1;
  };
#else
  #include <unordered_set>
  #include <unordered_map>
  namespace octomap {
    namespace unordered_ns = std;
  }
#endif

namespace octomap {

  /**
   * OcTreeKey is a container class for internal key addressing. The keys count the
   * number of cells (voxels) from the origin as discrete address of a voxel.
   * @see OcTreeBaseImpl::coordToKey() and OcTreeBaseImpl::keyToCoord() for conversions.
   */
  class OcTreeKey {

  public:
    OcTreeKey () {}
    OcTreeKey (unsigned int a, unsigned int b, unsigned int c)
      { k[0] = a; k[1] = b; k[2] = c; }
    OcTreeKey(const OcTreeKey& other){
      k[0] = other.k[0]; k[1] = other.k[1]; k[2] = other.k[2];
    }
    bool operator== (const OcTreeKey &other) const {
      return ((k[0] == other[0]) && (k[1] == other[1]) && (k[2] == other[2]));
    }
    bool operator!= (const OcTreeKey& other) const {
      return( (k[0] != other[0]) || (k[1] != other[1]) || (k[2] != other[2]) );
    }
    OcTreeKey& operator=(const OcTreeKey& other){
      k[0] = other.k[0]; k[1] = other.k[1]; k[2] = other.k[2];
      return *this;
    }
    const unsigned int& operator[] (unsigned int i) const {
      return k[i];
    }
    unsigned int& operator[] (unsigned int i) {
      return k[i];
    }

    unsigned int k[3];

    /// Provides a hash function on Keys
    struct KeyHash{
      size_t operator()(const OcTreeKey& key) const{
        // a hashing function

        // The hash keys are only used by octomap::KeySet and octomap::KeyBoolMap.
        // They both only operate on the current point cloud. Wherever the
        // robot is inside the very large octree-cube, these point's coordinates
        // will only differ in their last bits. So instead of coding every
        // coordinate with the full 32bit, we only use the last 16bit of each.
        // This way a unique 64bit hash key can be generated for all points of
        // _one_ pointcloud (from a range-limited sensor).
        return
            static_cast< std::size_t >( key.k[0] & 0xFFFF )
            + ( static_cast< std::size_t >( key.k[1] & 0xFFFF ) << 16 )
            + ( static_cast< std::size_t >( key.k[2] & 0xFFFF ) << 32 );
      }
    };

  };

  /**
   * Data structure to efficiently compute the nodes to update from a scan
   * insertion using a hash set.
   * @note you need to use boost::unordered_set instead if your compiler does not
   * yet support tr1!
   */
  typedef unordered_ns::unordered_set<OcTreeKey, OcTreeKey::KeyHash> KeySet;

  /**
   * Data structrure to efficiently track changed nodes as a combination of
   * OcTreeKeys and a bool flag (to denote newly created nodes)
   *
   */
  typedef unordered_ns::unordered_map<OcTreeKey, bool, OcTreeKey::KeyHash> KeyBoolMap;


  class KeyRay {
  public:

    KeyRay () {
      ray.resize(100000);
      reset();
    }
    void reset() {
      end_of_ray = begin();
    }
    void addKey(const OcTreeKey& k) {
      assert(end_of_ray != ray.end());
      *end_of_ray = k;
      end_of_ray++;
    }

    size_t size() const { return end_of_ray - ray.begin(); }
    size_t sizeMax() const { return 100000; }

    typedef std::vector<OcTreeKey>::iterator iterator;
    typedef std::vector<OcTreeKey>::const_iterator const_iterator;
    typedef std::vector<OcTreeKey>::reverse_iterator reverse_iterator;

    iterator begin() { return ray.begin(); }
    iterator end() { return end_of_ray; }
    const_iterator begin() const { return ray.begin(); }
    const_iterator end() const   { return end_of_ray; }

    reverse_iterator rbegin() { return (reverse_iterator) end_of_ray; }
    reverse_iterator rend() { return ray.rend(); }

  public:

    std::vector<OcTreeKey> ray;
    std::vector<OcTreeKey>::iterator end_of_ray;
  };

  /**
   * Computes the key of a child node while traversing the octree, given
   * child index and current key
   *
   * @param[in] pos index of child node (0..7)
   * @param[in] center_offset_key constant offset of octree keys
   * @param[in] parent_key current (parent) key
   * @param[out] child_key  computed child key
   */
  inline void computeChildKey (const unsigned int& pos, const unsigned int& center_offset_key,
                                          const OcTreeKey& parent_key, OcTreeKey& child_key) {
    // x-axis
    if (pos & 1) child_key[0] = parent_key[0] + center_offset_key;
    else         child_key[0] = parent_key[0] - center_offset_key - (center_offset_key ? 0 : 1);
    // y-axis
    if (pos & 2) child_key[1] = parent_key[1] + center_offset_key;
    else         child_key[1] = parent_key[1] - center_offset_key - (center_offset_key ? 0 : 1);
    // z-axis
    if (pos & 4) child_key[2] = parent_key[2] + center_offset_key;
    else         child_key[2] = parent_key[2] - center_offset_key - (center_offset_key ? 0 : 1);
  }

  /// generate child index (between 0 and 7) from key at given tree depth
  inline unsigned char computeChildIdx(const OcTreeKey& key, int depth){
    unsigned char pos = 0;
    if (key.k[0] & (1 << depth)) pos += 1;
    if (key.k[1] & (1 << depth)) pos += 2;
    if (key.k[2] & (1 << depth)) pos += 4;
    return pos;
  }

  /**
   * Generates a unique key for all keys on a certain level of the tree
   *
   * @param level from the bottom (= tree_depth - depth of key)
   * @param key input indexing key (at lowest resolution / level)
   * @return key corresponding to the input key at the given level
   */
  inline OcTreeKey computeIndexKey(unsigned int level, const OcTreeKey& key) {
    if (level == 0)
      return key;
    else {
      unsigned int mask = 4294967295 << level;
      OcTreeKey result = key;
      result[0] &= mask;
      result[1] &= mask;
      result[2] &= mask;
      return result;
    }
  }

} // namespace

#endif
