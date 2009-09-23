#ifndef OCTOMAP_BINARYOCTREE_HH
#define OCTOMAP_BINARYOCTREE_HH

// =====================================================
// octomap
// -----------------------------------------------------
// Kai M. Wurm <wurm@uni-freiburg.de>
// Armin Hornung <hornunga@informatik.uni-freiburg.de>
// =====================================================

#include "AbstractOcTree.h"
#include "OcTreeNode.h"
#include "ScanGraph.h"

namespace octomap {

  class OcTree : public AbstractOcTree <OcTreeNode> {

  public:
    static const int TREETYPE=3;

  public:

    OcTree(double _resolution);
    virtual ~OcTree();

    virtual OcTreeNode* updateNode(const point3d& value, bool occupied);


    /**
     * Inserts a 3d scan into the tree, creates delta nodes
     *
     * @param scan ScanNode to be inserted
     */
    void insertScan(const ScanNode& scan);
    virtual bool insertRay(const point3d& origin, const point3d& end);
    void integrateMissOnRay(const point3d& origin, const point3d& end);

    //! convert all delta nodes to binary nodes
    void deltaToBinary();


    //! get occupied leaf points (MIXED treated as occupied)
    void getOccupied(uint max_depth, double occ_thres, std::list<OcTreeVolume>& binary_nodes, std::list<OcTreeVolume>& delta_nodes) const;
    //! get free leaf points
    void getFreespace(uint max_depth, double occ_thres, std::list<OcTreeVolume>& binary_nodes, std::list<OcTreeVolume>& delta_nodes) const;
    //! gets potential obstacles: nodes labeled "free" (were binary before), but are now delta
    void getChangedFreespace(uint max_depth, double occ_tres, std::list<OcTreeVolume>& nodes) const;

    //! returns the memory usage of the OcTree in Bytes
    unsigned int memoryUsage() const;

    //! returns the memory usage of a full grid of the same size as OcTree in Bytes
    unsigned int memoryFullGrid();

    unsigned int memoryUsageEightPointers();

    //! size of OcTree in meters for x, y and z dimension
    void getMetricSize(double& x, double& y, double& z);
    //! minimum value in x, y, z
    void getMetricMin(double& x, double& y, double& z);
    //! maximum value in x, y, z
    void getMetricMax(double& x, double& y, double& z);

    //! Lossless compression of OcTree: merge children to parent when they
    //  have the same value
    void prune();

    void calcNumberOfNodesPerType(uint& num_binary, uint& num_delta) const;

    std::istream& readBinary(std::ifstream &s);
    std::ostream& writeBinary(std::ostream &s);

    void readBinary(std::string filename);
    void writeBinary(std::string filename);

  protected:

    /*!
     * Traces a ray from origin to end (excluding), returning all
     * cell centers of cells on beam
     * (Essentially using the DDA algorithm in 3D).
     */
    bool computeRay(const point3d& origin, const point3d& end, std::vector<point3d>& _ray) const;

    void insertScanUniform(const ScanNode& scan);

    // insert only freespace (freespace=true) or occupied space
    void insertScanFreeOrOccupied(const ScanNode& scan, bool freespace);


    OcTreeNode* updateNodeRecurs(OcTreeNode* node, bool node_just_created, unsigned short int key[3],
				 uint depth, bool occupied);

    void getOccupiedRecurs(OcTreeNode* node, uint depth, uint max_depth, double occ_thres, const point3d& parent_center,
			   std::list<OcTreeVolume>& binary_nodes, std::list<OcTreeVolume>& delta_nodes) const;
    void getFreespaceRecurs(OcTreeNode* node, uint depth, uint max_depth, double occ_thres, const point3d& parent_center,
			    std::list<OcTreeVolume>& binary_nodes, std::list<OcTreeVolume>& delta_nodes) const;
    void getChangedFreespaceRecurs(OcTreeNode* node, unsigned int depth, unsigned int max_depth, double occ_thres,
              const point3d& parent_center, std::list<OcTreeVolume>& nodes) const;

    void deltaToBinaryRecurs(OcTreeNode* node, uint depth, uint max_depth);

    void pruneRecurs(OcTreeNode* node, uint depth, uint max_depth, unsigned int& num_pruned);

    void calcNumberOfNodesPerTypeRecurs(OcTreeNode* node,
					unsigned int& num_binary,
					unsigned int& num_delta) const;

    unsigned int calcNumNodes() const;
    void calcNumNodesRecurs(OcTreeNode* node, unsigned int& num_nodes) const;
    // recalculates min and max in x, y, z. Only called when needed, after tree size changed.
    void calcMinMax();

  };


} // end namespace

#endif
