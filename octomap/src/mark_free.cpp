
#include <string>
#include <fstream>
#include <iostream>
#include <octomap/octomap.h>
#include <cstdlib>
#include <cstring>

using namespace std;
using namespace octomap;

class USizeOcTree : public OcTree {
public:
	USizeOcTree(double size) : OcTree(size){}

	void updateSize(){
		this->tree_size = count_nodes(this->getRoot());
	}

protected:
	size_t count_nodes(OcTreeNode* node){
		size_t count = 1;
		
		for(unsigned int i = 0; i < 8; ++i){
			if(node->childExists(i)){
				count += count_nodes(node->getChild(i));
			}
		}

		return count;
	}

};

unsigned int mark_free(OcTreeNode* node, unsigned int depth, unsigned int max_depth, float clamping_thres_min, float occ_prob_thres_log){
	if(node->getLogOdds() <= occ_prob_thres_log)
		node->setLogOdds(clamping_thres_min);

	if(depth == max_depth)
		return 1;

	unsigned int count = 0;
	bool any_child_exist = false;

	for(unsigned int i = 0; i < 8; ++i){
		if(node->childExists(i)){
			any_child_exist = true;
			break;
		}
	}

	if(any_child_exist){
		for(unsigned int i = 0; i < 8; ++i){
			if(!node->childExists(i)){
				node->createChild(i);
			}

			count += mark_free(node->getChild(i), depth + 1, max_depth, clamping_thres_min, occ_prob_thres_log);
		}
	}

	// prevent the tree from hugging all the memory
	if(node->collapsible())
		node->pruneNode();
	
	return count;
}

unsigned int mark_free(USizeOcTree* tree) {
	float clamping_thres_min = tree->getClampingThresMinLog();
	float occ_prob_thres_log = tree->getOccupancyThresLog();

	// prevent float equality ambiguity
	// a single occupied hit sets the logodds way above that number
	// for most configuration (default is 0.85)
	occ_prob_thres_log += 1e-6;
	unsigned int max_depth = tree->getTreeDepth();

	unsigned int count = 0;
	OcTreeNode* root = tree->getRoot();

	for(unsigned int i = 0; i < 8; ++i){
		if(!root->childExists(i)){
			root->createChild(i);
		}

		count += mark_free(root->getChild(i), 2, max_depth, clamping_thres_min, occ_prob_thres_log);
	}

	tree->prune();

	// We added a few nodes 
	tree->updateSize();

	return count;
}

int main(int argc, char** argv) {
	bool show_help = false;
	string outputFilename("");
	string inputFilename("");

	if(argc == 1) show_help = true;
	for(int i = 1; i < argc && !show_help; i++) {
		if(strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ||
		   strcmp(argv[i], "--usage") == 0 || strcmp(argv[i], "-usage") == 0 ||
		   strcmp(argv[i], "-h") == 0
		  )
		show_help = true;
	}
		
	if(show_help) {
		cout << "Usage: "<<argv[0]<<" [OPTIONS] <filename.bt>" << endl;
		cout << "\t -o <file>        Output filename (default: first input filename + .bt)" << endl;
		exit(0);
	}

	for(int i = 1; i < argc; i++) {
		// Parse command line arguments
		if(strcmp(argv[i], "-o") == 0 && i < argc - 1) {
			i++;
			outputFilename = argv[i];
			continue;
		} else if (i == argc-1){
			inputFilename = string(argv[i]);
		}
	}

	if (outputFilename == ""){
		size_t lastdot = inputFilename.find_last_of(".");
		if (lastdot == std::string::npos)
			outputFilename = inputFilename + ".free.bt";
		else
			outputFilename = inputFilename.substr(0, lastdot) + ".free.bt";
	}

	// An udatable size octree
	USizeOcTree* tree = new USizeOcTree(0.1);
	if (!tree->readBinary(inputFilename)){
		OCTOMAP_ERROR("Could not open file, exiting.\n");
		exit(1);
	}

	cout << "Creating all necessary nodes and marking them free" << endl;
	unsigned int count = mark_free(tree);
	cout << "Marked " << count << " leaves as free" << endl;

	// Must convert the Updatable Size OcTree to an OcTree
	// for writing
	OcTree* otree = tree;
	cout << "Writing octree to " << outputFilename << endl;
	if (!otree->writeBinary(outputFilename)){
		OCTOMAP_ERROR("Error writing tree to %s\n", outputFilename.c_str());
		exit(1);
	}

	cout << "done" << endl;
	delete tree;
	return 0;
}