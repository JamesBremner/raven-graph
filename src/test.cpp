#include <iostream>
#include "cutest.h"
#include "cGraph.h"

TEST(adjacent)
{
    cGraph graph;
    graph.directed(false);
    graph.setEdges(
        "a b "
        "a x ");

    auto v = graph.findorAdd("a");
    CHECK_EQUAL(2, graph.adjacentIn(v).size());
    CHECK_EQUAL(2, graph.adjacentOut(v).size());
    CHECK_EQUAL(2, graph.adjacentAll(v).size());

    graph.clear();
    graph.directed(true);
    graph.setEdges(
        "a b "
        "a x ");

    v = graph.findorAdd("a");
    CHECK_EQUAL(2, graph.adjacentIn(v).size());
    CHECK_EQUAL(2, graph.adjacentOut(v).size());
    CHECK_EQUAL(2, graph.adjacentAll(v).size());

    auto vls = graph.leaves();
    CHECK_EQUAL(2, vls.size());

    graph.setEdges(
        "a b "
        "b c "
        "c d "
        "d a "
        "a x "
        "x y ");
    CHECK_EQUAL(3, graph.adjacentIn(graph.findorAdd("a")).size());
    CHECK_EQUAL(2, graph.adjacentIn(graph.findorAdd("b")).size());
    CHECK_EQUAL(2, graph.adjacentIn(graph.findorAdd("c")).size());
    CHECK_EQUAL(2, graph.adjacentIn(graph.findorAdd("d")).size());
    CHECK_EQUAL(2, graph.adjacentIn(graph.findorAdd("x")).size());
}

TEST(setEdges)
{
    cGraph graph;

    graph.setEdges(
        "a b 1 "
        "a x 2 "
        "b x 3 ",
        1);

    auto edges = graph.getlinkedVerticesNames();

    std::vector<std::pair<std::string, std::string>> expected{
        {"a", "b"},
        {"a", "x"},
        {"b", "x"}};
    CHECK_EQUAL(expected.size(), edges.size());
    for (int k = 0; k < expected.size(); k++)
    {
        CHECK_EQUAL(expected[k].first, edges[k].first);
        CHECK_EQUAL(expected[k].second, edges[k].second);
    }
    CHECK_EQUAL(1.0, graph.edgeAttrDouble("a", "b", 0));
    CHECK_EQUAL(2.0, graph.edgeAttrDouble("a", "x", 0));
    CHECK_EQUAL(3.0, graph.edgeAttrDouble("b", "x", 0));
}

TEST(breadth_first_search)
{
    // construct test graph
    cGraph graph;
    graph.setEdges(
        "a b "
        "b c "
        "c d "
        "d a "
        "a x "
        "x y ");

    std::vector<std::string> expected{"a", "b", "d", "x", "c", "y"};
    std::vector<std::string> visited;

    graph.bfs(
        "a",
        [&](int v)
        {
            visited.push_back(graph.userName(v));
        });

    CHECK_EQUAL(expected.size(), visited.size());
    for (int k = 0; k < expected.size(); k++)
        CHECK_EQUAL(visited[k], expected[k]);
}
TEST(depth_first_search)
{
    // construct test graph
    cGraph graph;
    graph.setEdges(
        "a b "
        "b c "
        "c d "
        "d a "
        "a x "
        "x y ");

    std::vector<std::string> expected_undirected{"a", "x", "y", "d", "c", "b"};
    std::vector<std::string> visited;

    graph.dfs(
        "a",
        [&](int v)
        {
            visited.push_back(graph.userName(v));
        });

    for (int k = 0; k < expected_undirected.size(); k++)
        CHECK_EQUAL(expected_undirected[k], visited[k]);

    // two components
    graph.directed(false);
    graph.setEdges(
        "a b "
        "b c "
        "x y");
    std::vector<std::string> expected2{"a", "b", "c"};
    visited.clear();
    graph.dfs(
        "a",
        [&](int v)
        {
            visited.push_back(graph.userName(v));
        });
    CHECK_EQUAL(3, visited.size());
    for (int k = 0; k < expected2.size(); k++)
        CHECK_EQUAL(expected2[k], visited[k]);
}

TEST( depth_first_directed)
{
    std::vector<std::string> expected_directed{"a", "x", "y", "d", "c", "b" };
    std::vector<std::string> visited;

    cGraph graph;
    graph.directed(true);
    graph.setEdges(
        "a b "
        "b c "
        "c d "
        "d a "
        "a x "
        "x y ");

    graph.dfs(
        "a",
        [&](int v)
        {
            visited.push_back(graph.userName(v));
        });

    for (int k = 0; k < expected_directed.size(); k++)
        CHECK_EQUAL(expected_directed[k], visited[k]);
}
TEST(dijsktra_undirected)
{
    // construct test graph
    cGraph graph;
    graph.setEdges(
        "a b "
        "b c "
        "c d "
        "d a "
        "a x "
        "x y ");

    std::vector<std::string> expected1{"a", "x", "y"};
    std::vector<std::string> visited;

    auto p1 = graph.path("a", "y");

    for (int k = 0; k < expected1.size(); k++)
        CHECK_EQUAL(graph.userName(p1[k]), expected1[k]);

    std::vector<std::string> expected2{"a", "d"};
    auto p2 = graph.path("a", "d");
    for (int k = 0; k < expected2.size(); k++)
        CHECK_EQUAL(graph.userName(p2[k]), expected2[k]);
}

// TEST(cycle_finder)
// {
//     // construct test graph
//     cGraph graph;

//     // one cycle
//     graph.setEdges("a b \nb c \nc d \nd a ");
//     std::vector<std::string> expected1{"a", "b", "c", "d", "a"};
//     auto vCycle = graph.dfs_cycle_finder("a");
//     CHECK_EQUAL(vCycle.size(), 1);
//     for (int k = 0; k < expected1.size(); k++)
//         CHECK_EQUAL(graph.userName(vCycle[0][k]), expected1[k]);

//     // no cycle ( directed )
//     graph.setEdges("a b \nb c \nc d \na x \nx y \ny d ");
//     vCycle = graph.dfs_cycle_finder("a");
//     CHECK_EQUAL(vCycle.size(), 0);

//     // two cycles
//     graph.setEdges("a b \nb c \nc d \nd a \nc a ");
//     vCycle = graph.dfs_cycle_finder("a");
//     CHECK_EQUAL(vCycle.size(), 2);
//     std::vector<std::string> expected0{"a", "b", "c", "a"};
//     for (int k = 0; k < expected0.size(); k++)
//         CHECK_EQUAL(graph.userName(vCycle[0][k]), expected0[k]);
//     for (int k = 0; k < expected1.size(); k++)
//         CHECK_EQUAL(graph.userName(vCycle[1][k]), expected1[k]);
// }

TEST(SpanningTree_undirected)
{
    cGraph graph;
    graph.setEdges(
        "a b "
        "b c "
        "c d "
        "a x "
        "x y "
        "y d ");

    auto span = graph.spanningTree("a");

    CHECK_EQUAL(6, span.vertexCount());

    auto edges = graph.getlinkedVerticesNames();
    std::vector<std::pair<std::string, std::string>> expected{
        {"a", "b"},
        {"a", "x"}};
    for (int k = 0; k < expected.size(); k++)
    {
        CHECK_EQUAL(expected[k].first, edges[k].first);
        CHECK_EQUAL(expected[k].second, edges[k].second);
    }
}

TEST(SpanningTreeCost)
{
    cGraph graph;

    graph.setEdges(
        "a b 1 "
        "a x 1 "
        "b x 10 ",
        1);

    auto span = graph.spanningTree("a");

    CHECK_EQUAL(3, span.vertexCount());

    auto edges = span.getlinkedVerticesNames();
    std::vector<std::pair<std::string, std::string>> expected{
        {"a", "b"},
        {"a", "x"},
    };
    for (int k = 0; k < expected.size(); k++)
    {
        CHECK_EQUAL(expected[k].first, edges[k].first);
        CHECK_EQUAL(expected[k].second, edges[k].second);
    }

    // increase cost of b-c edge
    graph.setEdges(
        "a b 1 "
        "a x 10 "
        "b x 1 ",
        1);
    span = graph.spanningTree("a");

    CHECK_EQUAL(3, span.vertexCount());

    edges = span.getlinkedVerticesNames();
    std::vector<std::pair<std::string, std::string>> expected2{
        {"a", "b"},
        {"b", "x"},
    };
    for (int k = 0; k < expected2.size(); k++)
    {
        CHECK_EQUAL(expected2[k].first, edges[k].first);
        CHECK_EQUAL(expected2[k].second, edges[k].second);
    }
}

TEST(componentCount)
{
    cGraph graph;
    graph.setEdges(
        "a b "
        "b c "
        "x y ");
    CHECK_EQUAL(2, graph.componentCount());
}

main()
{
    return raven::set::UnitTest::RunAllTests();
}