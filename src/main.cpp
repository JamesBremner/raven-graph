
#include "cGraph.h"

main()
{
    // construct test graph
    cGraph graph;
    graph.setEdges("a b\nb c\nc d\nd a");

    // display graph edges
    std::cout << graph.text();

    // run the cycle detector
    auto vCycle = graph.dfs_cycle_finder("a");

    for( auto& vv : vCycle )
    {
        std::cout << "cycle: ";
        for( auto v : vv )
            std::cout << graph.userName(v) << " ";
        std::cout << "\n";
    }

    return 0;

}
