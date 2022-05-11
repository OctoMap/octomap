#include <octomap/SemanticOcTree.h>
#include <unordered_map>

namespace octomap{

    // node implementation  --------------------------------------
    std::ostream& SemanticOcTreeNode::writeData(std::ostream &s) const {
        s.write((const char*) &value, sizeof(value)); // occupancy
        s.write((const char*) &semantic_info, sizeof(Semantics)); // semantics

        return s;
    }

    std::istream& SemanticOcTreeNode::readData(std::istream &s) {
        s.read((char*) &value, sizeof(value)); // occupancy
        s.read((char*) &semantic_info, sizeof(Semantics)); // color

        return s;
    }

    //Basic implementation of this function: Needs to be improved for future
    Semantics SemanticOcTreeNode::getAverageChildSemanticInfo() const{
        std::unordered_map<int, int> semantic_hist;
        int max_count = 0;
        int max_est_category = -1;

        if(children!=NULL){
            for(int i=0; i<8; i++) { 
                SemanticOcTreeNode* child = static_cast<SemanticOcTreeNode*>(children[i]);

                if(child != NULL){
                    int child_est_catergory = child->getSemanticInfo().est_category;
                    semantic_hist[child_est_catergory] += 1;
                    if(semantic_hist[child_est_catergory]>max_count){
                        max_count = semantic_hist[child_est_catergory];
                        max_est_category = child_est_catergory;
                    }
                }
            }
        }

        return octomap::Semantics(-1, max_est_category, 0.0); 
    }

    void SemanticOcTreeNode::updateSemanticsChildren(){
        semantic_info = getAverageChildSemanticInfo();
    }
    
    //tree implementation
    SemanticOcTree::SemanticOcTree(double resolution)
    : OccupancyOcTreeBase<SemanticOcTreeNode>(resolution){
            semanticOcTreeMemberInit.ensureLinking();
    }
    
    SemanticOcTree::StaticMemberInitializer SemanticOcTree::semanticOcTreeMemberInit;
    
    SemanticOcTreeNode* SemanticOcTree::setNodeSemantics(const OcTreeKey& key,
                                                         int id,
                                                         int est_category,
                                                         float confidence) {
        
        SemanticOcTreeNode* n = search(key);
        if(n!=0){
            n->setSemanticInfo(id, est_category, confidence);
        }
        return n;
    }

    bool SemanticOcTree::isNodeCollapsible(const SemanticOcTreeNode* node) const{
    // all children must exist, must not have children of
    // their own and have the same occupancy probability
        if (!nodeChildExists(node, 0))
            return false;

        const SemanticOcTreeNode* firstChild = getNodeChild(node, 0);
        if (nodeHasChildren(firstChild))
            return false;

        for (unsigned int i = 1; i<8; i++) {
        // compare nodes only using their occupancy, ignoring color for pruning
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
                return false;
        } 

        return true;
    }
    
    bool SemanticOcTree::pruneNode(SemanticOcTreeNode* node) {
        if (!isNodeCollapsible(node))
            return false;

        // set value to children's values (all assumed equal)
        node->copyData(*(getNodeChild(node, 0)));

        if (node->isSemanticInfoSet()) // TODO check
            node->setSemanticInfo(node->getAverageChildSemanticInfo());

        // delete children
        for (unsigned int i=0;i<8;i++) {
            deleteNodeChild(node, i);
        }
        delete[] node->children;
        node->children = NULL;

        return true;
    }

    SemanticOcTreeNode* SemanticOcTree::integrateNodeSemantics(const OcTreeKey& key,
                                                                int id,
                                                                int category,
                                                                float confidence) {
        SemanticOcTreeNode* n = search (key);
        if (n != 0) {
            if (n->isSemanticInfoSet()) {
                //Currently integrates the latest info, but need better way to integrate semantics
                int new_id = id;
                int new_category = category;
                float new_confidence = confidence;
                n->setSemanticInfo(new_id, new_category, new_confidence);
            }
            else {
                n->setSemanticInfo(id, category, confidence);
            }
        }
        return n;
  }

   void SemanticOcTree::updateInnerOccupancy() {
      this->updateInnerOccupancyRecurs(this->root, 0);
   }

  void SemanticOcTree::updateInnerOccupancyRecurs(SemanticOcTreeNode* node, unsigned int depth) {
    // only recurse and update for inner nodes:
    if (nodeHasChildren(node)){
      // return early for last level:
      if (depth < this->tree_depth){
        for (unsigned int i=0; i<8; i++) {
          if (nodeChildExists(node, i)) {
            updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
          }
        }
      }
      node->updateOccupancyChildren();
      node->updateSemanticsChildren();
    }
  }


}//end of namespace