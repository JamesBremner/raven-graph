#include <fstream>
#include "cGraph.h"

namespace raven {
graph_calc readfile(
    raven::cGraph& g,
    const std::string &fname)
{
    graph_calc option = graph_calc::none;
    g.clear();
    std::ifstream ifs(fname);
    if (!ifs.is_open())
        throw std::runtime_error(
            "Cannot open input file");
    std::string form, calc;
    ifs >> form >> calc;
    if (form != "format")
        throw std::runtime_error(
            "Bad input format");


    if (calc.find("cycle") != -1)
    {
        option = graph_calc::cycle;
        g.directed(true);
        std::string sn1, sn2;
        ifs >> sn1 >> sn2;

        while (ifs.good())
        {
            g.addEdge( sn1, sn2 );
            ifs >> sn1 >> sn2;
        }
    }

    else
        throw std::runtime_error(
            "bad calculation type ");

    return option;
}
}
