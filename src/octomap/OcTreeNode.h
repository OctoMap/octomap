#ifndef MAPPING3D_BINARYOCTREE_NODE_HH
#define MAPPING3D_BINARYOCTREE_NODE_HH

// =====================================================
// octomap
// -----------------------------------------------------
// Kai M. Wurm <wurm@uni-freiburg.de>
// Armin Hornung <hornunga@informatik.uni-freiburg.de>
// =====================================================

#include "octomap_types.h"

namespace octomap {


#define PROB_HIT  0.7 
#define PROB_MISS 0.4
#define PRUNING_PROB_THRES 0.05
#define ML_OCC_PROB_THRES 0.5
#define CLAMPING_THRES_MIN -2
#define CLAMPING_THRES_MAX 3.5

#define UNKOWN_AS_OBSTACLE false

  class OcTreeNode {

  public:

    enum Labels {FREE=0, OCCUPIED=1, MIXED=2, UNKNOWN=3};

    OcTreeNode();
    ~OcTreeNode();

    // children
    OcTreeNode* getChild(unsigned int i);

    /**
     * const version of getChild aboce
     *
     * @param i child number (index)
     * @return const OcTreeNode* to child
     */
    const OcTreeNode* getChild(unsigned int i) const;

    bool createChild(unsigned int i); // returns true if children had to be alloced

    bool childExists(unsigned int i) const;
    bool hasChildren() const;
    bool collapsible() const;
    bool valid() const; // returns true if the node is valid

    // data
    bool isDelta() const;
    void setLabel(char l);
    char getLabel() const;
    /**
     * Converts a pure binary node to delta. Sets LogOdds only of leaf nodes,
     * inner nodes should be set by the update call
     */
    void convertToDelta();
    void convertToBinary();
    bool labelMatches(bool occupied) const;
    /**
     * @return mean of all child probabilities (when they exist), in log odds
     */
    double getMeanChildLogOdds() const;

    /**
     * @return max of all child probabilities (when they exist), in log odds
     */
    double getMaxChildLogOdds() const;

    void integrateHit();
    void integrateMiss();
    double getOccupancy() const;
    bool isOccupied() const;

    float getLogOdds() const{ return log_odds_occupancy; }
    void setLogOdds(float l) { log_odds_occupancy = l; }



    /**
     * Prunes a node when it is collapsible
     *
     * @return success of pruning
     */
    bool pruneNode();

    // file I/O
    std::istream& readBinary(std::istream &s);
    std::ostream& writeBinary(std::ostream &s);

  protected:

    double logodds(double p) const;
    void updateLogOdds(double p);
    double prior() const;

    void allocChildren();
    void setValid(bool v);
    void setDelta(bool a);
    char commonChildLabel() const;
    bool pruneBinary();

    float log_odds_occupancy; // store log odds occupancy probability
    char data; // store label and state

    OcTreeNode** itsChildren; // pointer to children, may be NULL
  };



  // for memory computation only
  class OcTreeNodeEightPointers {
  public:
    float log_odds_occupancy;
    char data;
    OcTreeNodeEightPointers* itsChildren[8];
  };

  // for memory computation only
  class OcTreeNodeLight {
  public:
    float log_odds_occupancy;
    OcTreeNodeLight* itsChildren;
  };

} // end namespace



#endif
