#ifndef OCTREEX64_H
#define OCTREEX64_H

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

typedef int64_t TimestampType;
typedef octomap::OcTreeNode LegacyNode;
// node definition
class OcTreeX64Node : public LegacyNode {

public:
    OcTreeX64Node() : LegacyNode(), timestamp() {}

    OcTreeX64Node(const OcTreeX64Node& rhs) : LegacyNode(rhs),
        timestamp(rhs.timestamp) {}

    bool operator==(const OcTreeX64Node& rhs) const{
        return (rhs.value == value && rhs.timestamp == timestamp);
    }

    // children
    inline OcTreeX64Node* getChild(unsigned int i) {
        return static_cast<OcTreeX64Node*> (LegacyNode::getChild(i));
    }
    inline const OcTreeX64Node* getChild(unsigned int i) const {
        return static_cast<const OcTreeX64Node*> (LegacyNode::getChild(i));
    }

    bool createChild(unsigned int i) {
        if (children == NULL) allocChildren();
        children[i] = new OcTreeX64Node();
        return true;
    }

    // timestamp
    inline TimestampType getTimestamp() const { return timestamp; }
    inline void updateTimestamp() { timestamp = 1000;}
    inline void setTimestamp(TimestampType timestamp) {this->timestamp = timestamp; }

    // update occupancy and timesteps of inner nodes
    inline void updateOccupancyChildren() {
        this->setLogOdds(this->getMaxChildLogOdds());  // conservative
        updateTimestamp();
    }

protected:
    TimestampType timestamp;
};


typedef octomap::OccupancyOcTreeBase<OcTreeX64Node> LegacyTree;
// tree definition
class OcTreeX64 : public LegacyTree {

public:
    /// Default constructor, sets resolution of leafs
    OcTreeX64(double resolution) : LegacyTree(resolution) {}

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    OcTreeX64* create() const {return new OcTreeX64(resolution); }

    std::string getTreeType() const {return "OcTree";}

    //! \return timestamp of last update
    TimestampType getLastUpdateTime();

    void degradeOutdatedNodes(unsigned int time_thres);

    virtual void updateNodeLogOdds(OcTreeX64Node* node, const float& update) const;
    void integrateMissNoTime(OcTreeX64Node* node) const;

protected:
    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer{
    public:
        StaticMemberInitializer() {
            OcTreeX64* tree = new OcTreeX64(0.1);
            AbstractOcTree::registerTreeType(tree);
        }
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer OcTreeX64MemberInit;

};

#endif // OCTREEX64_H
