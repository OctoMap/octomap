/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
 * 
 * Created by Sandilya Sai Garimella (sgarimella34@gatech.edu)
 * 
* merge_octomaps
 * ---------------
 * Merge multiple OctoMap .bt files into one, keeping ONLY occupied voxels from
 * each input. Free/unknown voxels in later maps do not clear previously merged data.
 *
 * Build:
 *   g++ -std=c++17 merge_octomaps.cpp -loctomap -o merge_octomaps
 *
 * Usage:
 *   ./merge_octomaps <output.bt> <input0.bt> [input1.bt ... inputN.bt]
 *
 * Examples:
 *   # Simple two-layer merge
 *   ./merge_octomaps combined.bt layer_0.bt layer_1.bt
 *
 *   # Merge many layers found in a directory
 *   ./merge_octomaps combined.bt /path/to/layers/layer_*.bt
 *
 * Notes:
 *   - The resolution of the output tree is taken from the first input file.
 *   - Inner-node occupancy is recomputed after insertion.
 *   - Inputs with incompatible resolution are skipped (you can add checks if needed).
 * */

#include <octomap/OcTree.h>
#include <iostream>

using namespace octomap;

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0]
                  << " output.bt input1.bt [input2.bt ...]\n";
        return 1;
    }

    const char *out_fname = argv[1];

    // 1) Load the first layer
    OcTree *base = new OcTree(0.1);
    if (!base->readBinary(argv[2]))
    {
        std::cerr << "ERROR: cannot read base file: " << argv[2] << "\n";
        delete base;
        return 1;
    }

    // 2) For each additional layer, only merge OCCUPIED leaves
    for (int i = 3; i < argc; ++i)
    {
        OcTree *next = new OcTree(0.1);
        if (!next->readBinary(argv[i]))
        {
            std::cerr << "WARNING: skipping invalid file: " << argv[i] << "\n";
            delete next;
            continue;
        }

        double occThres = next->getOccupancyThres();
        for (auto it = next->begin_leafs(), end = next->end_leafs(); it != end; ++it)
        {
            // Only unite the occupied voxels
            if (it->getValue() > occThres)
            {
                base->updateNode(
                    it.getX(), it.getY(), it.getZ(),
                    true, // mark occupied
                    false // lazy eval off
                );
            }
        }

        delete next;
    }

    // 3) Recompute inner nodes from children
    base->updateInnerOccupancy();

    // 4) Write out
    if (!base->writeBinary(out_fname))
    {
        std::cerr << "ERROR: failed to write output file: " << out_fname << "\n";
        delete base;
        return 1;
    }

    std::cout << "Merged into " << out_fname << "\n";
    delete base;
    return 0;
}
