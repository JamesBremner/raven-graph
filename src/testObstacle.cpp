#include <iostream>
#include "cutest.h"

#define UNIT_TEST

#include "cObstacle.h"

TEST( tourSpanningTree )
{
    cObstacle O;
    cGraph g;
    g.setEdges(
        "a b "
        "a x "    );
    O.setGraph( g );
    O.tourSpanningTree();

}

main()
{
    return raven::set::UnitTest::RunAllTests();
}