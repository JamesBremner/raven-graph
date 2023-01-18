#include <iostream>
#include "cutest.h"

#define UNIT_TEST

#include "cObstacle.h"

TEST( tourSpanningTree )
{
    cObstacle O;
    raven::cGraph g;
    g.setEdges(
        "a b "
        "a x "    );
    O.setGraph( g );
    O.tourSpanningTree();
    auto st = O.spanningTree_get();

}

main()
{
    return raven::set::UnitTest::RunAllTests();
}