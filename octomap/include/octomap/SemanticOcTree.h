#ifndef OCTOMAP_SEMANTIC_OCTREE_H
#define OCTOMAP_SEMANTIC_OCTREE_H

#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <unordered_map>

namespace octomap{

    struct Semantics{
        std::unordered_map<int, int> category;
        int id;
        int est_category;
        float confidence;

        Semantics(): id(-1), est_category(-1), confidence(0.0){}
        Semantics(int _id, int _est_category, float _confidence)
         : id(_id), est_category(_est_category), confidence(_confidence){}

        bool operator== (const Semantics &other) const{
            return (est_category == other.est_category);
        } 

        bool operator!= (const Semantics &other) const{
            return (est_category != other.est_category);
        }
    };

    //forward declaration for "friend"
    class SemanticOcTree;

    class SemanticOcTreeNode : public OcTreeNode{
    
    protected:
        Semantics semantic_info;

    public:

        friend class SemanticOcTree;

        SemanticOcTreeNode() : OcTreeNode() {}        
        SemanticOcTreeNode(const SemanticOcTreeNode& rhs) : OcTreeNode(rhs), semantic_info(rhs.semantic_info){}
        bool operator==(const SemanticOcTreeNode& rhs) const{
            return (rhs.value == value && rhs.semantic_info == semantic_info);
        }

        void copyData(const SemanticOcTreeNode& from){
            OcTreeNode::copyData(from);
            this->semantic_info = from.getSemanticInfo();
        }

        bool isSemanticInfoSet(){return true;} //Always true for now.
        inline Semantics getSemanticInfo() const {return semantic_info;}
        inline void setSemanticInfo(Semantics s) {this->semantic_info = s;}
        inline void setSemanticInfo(int id, int est_category, float confidence) 
            {this->semantic_info = Semantics(id, est_category, confidence);}
        inline void clearSemanticInfo(){
            this->semantic_info.id = -1;
            this->semantic_info.est_category = -1;
            this->semantic_info.confidence = 0.0;
            this->semantic_info.category.clear();
        }        
        inline void setId(int id) {this->semantic_info.id = id;}
        inline void setCategory(int category) {this->semantic_info.est_category = category;}                          

        void addSemanticInfo(int id, int est_category, float confidence);
        Semantics& getSemanticInfo() {return semantic_info;}
        inline int getCategory() {return this->semantic_info.est_category;}
        inline int getId() {return this->semantic_info.id;}
        void updateSemanticsChildren();
        Semantics getAverageChildSemanticInfo() const;

        std::istream& readData(std::istream &s);
        std::ostream& writeData(std::ostream &s) const;
        SemanticOcTreeNode* getChild(unsigned int i){
            if ((children != NULL) && (children[i] != NULL)) {
                AbstractOcTreeNode * c = this->children[i];
                SemanticOcTreeNode * soctn = static_cast<SemanticOcTreeNode*>(c);
                return soctn;
            } else { 
                return NULL;
            }
        }
    };


    //Semantic OcTree Class
    class SemanticOcTree : public OccupancyOcTreeBase <SemanticOcTreeNode>{
    
    public:
        SemanticOcTree(double resolution);
        SemanticOcTree* create() const {return new SemanticOcTree(resolution);}
        std::string getTreeType() const {return "SemanticOcTree";}

        virtual bool pruneNode(SemanticOcTreeNode* node);
        virtual bool isNodeCollapsible(const SemanticOcTreeNode* node) const;
        
        // // set node color at given key or coordinate. Replaces previous color.
        SemanticOcTreeNode* setNodeSemantics(const OcTreeKey& key, int id, int est_category, float confidence);
        
        SemanticOcTreeNode* setNodeSemantics(float x, float y, 
                                             float z, int id, 
                                             int est_category, float confidence) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return setNodeSemantics(key,id,est_category,confidence);
        }

        
        SemanticOcTreeNode* integrateNodeSemantics(const OcTreeKey& key, int id, 
                                                    int est_category, float confidence);
        
        SemanticOcTreeNode* integrateNodeSemantics(const OcTreeKey& key);
        
        SemanticOcTreeNode* integrateNodeSemantics(float x, float y, 
                                                    float z, uint8_t id, 
                                                    int est_category, float confidence) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
            return integrateNodeSemantics(key,id, est_category, confidence);
        }

        void insertPointCloudAndSemantics(const Pointcloud& scan, const octomap::point3d& sensor_origin, 
                                    int id, int category, float confidence,
                                    double maxrange, bool lazy_eval, bool discretize);
          
        // update inner nodes, sets color to average child color
        void updateInnerOccupancy();

    protected:
        void updateInnerOccupancyRecurs(SemanticOcTreeNode* node, unsigned int depth);

        /**
        * Static member object which ensures that this OcTree's prototype
        * ends up in the classIDMapping only once. You need this as a 
        * static member in any derived octree class in order to read .ot
        * files through the AbstractOcTree factory. You should also call
        * ensureLinking() once from the constructor.
        */
        class StaticMemberInitializer{
        public:
            StaticMemberInitializer() {
                SemanticOcTree* tree = new SemanticOcTree(0.1);
                tree->clearKeyRays();
                AbstractOcTree::registerTreeType(tree);
            }

            /**
            * Dummy function to ensure that MSVC does not drop the
            * StaticMemberInitializer, causing this tree failing to register.
            * Needs to be called from the constructor of this octree.
            */
            void ensureLinking() {};
        };
        /// static member to ensure static initialization (only once)
        static StaticMemberInitializer semanticOcTreeMemberInit;
    };

} //end of namespace

#endif
