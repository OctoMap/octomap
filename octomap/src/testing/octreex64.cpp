#include "octreex64.h"

TimestampType OcTreeX64::getLastUpdateTime() {
  // this value is updated whenever inner nodes are
  // updated using updateOccupancyChildren()
  return root->getTimestamp();
}

void OcTreeX64::degradeOutdatedNodes(unsigned int time_thres_ms) {
  for(leaf_iterator it = this->begin_leafs(), end=this->end_leafs();
      it!= end; ++it) {
    if (isNodeOccupied(*it) && time_thres_ms > 5000) {
      integrateMissNoTime(&*it);
    }
  }
}

void OcTreeX64::updateNodeLogOdds(OcTreeX64Node* node, const float& update) const {
  LegacyTree::updateNodeLogOdds(node, update);
  node->updateTimestamp();
}

void OcTreeX64::integrateMissNoTime(OcTreeX64Node *node) const{
  LegacyTree::updateNodeLogOdds(node, prob_miss_log);
}

OcTreeX64::StaticMemberInitializer OcTreeX64::OcTreeX64MemberInit;
