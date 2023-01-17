#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include "cRunWatch.h"
#include "cObstacle.h"

void read(
    cObstacle &obs,
    const std::string &fname)
{
    obs.clear();
    std::ifstream ifs(fname);
    if (!ifs.is_open())
        throw std::runtime_error(
            "Cannot open input file");
    std::string form, calc;
    ifs >> form >> calc;
    if (form != "format")
        throw std::runtime_error(
            "Bad input format");

    int nx, ny, view;
    if (calc.find("obs") != -1)
    {
        ifs >> nx >> ny >> view;
        obs.grid(nx, ny);
        obs.view(view);
        while (ifs.good())
        {
            // insert obstacle
            int ox, oy;
            ifs >> ox >> oy;
            obs.obstacle(ox, oy);
        }
    }

    else if (calc.find("farm") != -1)
    {
        ifs >> view;
        obs.view(view);
        std::string sid;
        ifs >> sid;
        if (sid[0] == 'p')
            obs.poly();
        else
        {
            ifs >> nx >> ny;
            obs.grid(nx, ny);
            return;
        }
        ifs >> nx >> ny;
        obs.polyAdd(cxy(nx, ny));
        while (ifs.good())
        {
            ifs >> sid >> nx >> ny;
            if (sid[0] == 'p')
                obs.polyAdd(cxy(nx, ny));
        }
    }
    else
        throw std::runtime_error(
            "bad calculation type ");
}

void cObstacle::obstacle(int x, int y)
{
    if (0 > x || x > nx - 1 || 0 > y || y >> ny - 1)
        throw std::runtime_error("Bad location");
    A->cell(x, y)->myType = 1;
}

void cObstacle::clear()
{
    myView = -999;
    myfrect = true;
    vN.clear(); ///< nodes to be included in path
    // vL.clear(); ///< links between nodes
    vPath.clear();
    myPolygon.clear();
    mySpanningTree.clear();
}

void cObstacle::grid(int x, int y)
{
    nx = x;
    ny = y;
    A = new cell::cAutomaton<cOCell>(nx, ny);
    A->wrap(false);
    A->ortho(false);
}

bool cObstacle::isBlocked(int x1, int y1, int x2, int y2)
{
    int W, H;
    A->size(W, H);
    for (int w = 0; w < W; w++)
        for (int h = 0; h < H; h++)
        {
            // check for obstacle
            if (A->cell(w, h)->myType != 1)
                continue;

            // check for obstacle close to connection
            int ow, oh;
            A->coords(ow, oh, A->cell(w, h));

            cxy obstacle(ow, oh);
            cxy line1(x1, y1);
            cxy line2(x2, y2);
            if (obstacle.dis2toline(line1, line2) < 2)
                return true;
        }
    // no obstacle found close to connection
    return false;
}

void cObstacle::unobstructedPoints()
{
    raven::set::cRunWatch aWatcher("unobstructedPoints");
    int W, H;

    if (myfrect)
        A->size(W, H);
    else
    {
        cxy wh = cxy::enclosingWidthHeight(myPolygon);
        W = wh.x;
        H = wh.y;
        grid(W, H);
    }

    /** The distance covered by the robot
     * The minimum is 2
     * If the robot that can cover 2 is located at 10
     * it can cover all points from 8 to to 12
     * so there is no need for the robot to revisit points
     * closer than 5 units apart
     *
     * If there is an obstacle at 10
     * the robot must visit points at 7 and 13
     *
     * 7->8->9->10<-11<-12<-13
     */
    int V;
    if (myView > 0)
        V = myView;
    else
        V = 2;
    int space = 2 * V + 1;

    for (int h = V; h < H - V + 1; h += space)
        for (int w = V; w <= W - V + 1; w += space)
        {
            if (!myfrect)
            {
                cxy p(w, h);
                if (!p.isInside(myPolygon))
                    continue;
            }
            cOCell *c = A->cell(w, h);
            c->myType = 2;
            vN.push_back(c);
        }
    if (!vN.size())
        throw std::runtime_error(
            "No unobstructed points");
    std::cout << "Node count " << vN.size() << "\n";
    for (auto p : vN)
    {
        int w, h;
        A->coords(w, h, p);
        std::cout << p->ID() << " " << w << " " << h << "\n";
    }
}

std::string cObstacle::draw(int w, int h) const
{
    switch (A->cell(w, h)->myType)
    {
    case 0:
    default:
        return ".";
    case 1:
        return "X";
    case 2:
        return std::to_string(A->cell(w, h)->ID());
    }
}

void cObstacle::connect()
{
    raven::set::cRunWatch aWatcher("connect");

    // square of max distance between adjacent visit points
    int d2max = 2 * myView + 1;
    d2max = d2max * d2max;

    std::string edgeDescriptors;

    // loop over node pairs
    for (auto n1 : vN)
    {
        bool fconnected = false;
        for (auto n2 : vN)
        {
            // do not self connect
            if (n1 == n2)
                continue;

            // do not connect nodes that are not neighbours
            int w1, h1, w2, h2;
            A->coords(w1, h1, n1);
            A->coords(w2, h2, n2);

            // std::cout << n1->ID() <<" "<< n2->ID()
            //      <<" "<< w1 <<" "<< h1 <<" "<< w2 <<" "<< h2 << "\n";

            int dx = w1 - w2;
            int dy = h1 - h2;
            int d2 = dx * dx + dy * dy;
            if (d2 > d2max)
                continue;

            // check for blocked
            if (isBlocked(w1, h1, w2, h2))
                continue;

            // OK to connect
            edgeDescriptors +=
                std::to_string(n1->ID()) + " " +
                std::to_string(n2->ID()) + " " +
                std::to_string(d2) +
                " ";

            fconnected = true;
        }
        if (!fconnected)
            throw std::runtime_error(
                "node unconnected");
    }

    myGraph.setEdges(edgeDescriptors, 1);
}

vlink_t cObstacle::spanningTree_get()
{
    vlink_t ret;
    auto stlinks = mySpanningTree.getlinkedVerticesNames();
    for (auto gl : stlinks)
    {
        link_t ol(
            A->cell(std::atoi(gl.first.c_str())),
            A->cell(std::atoi(gl.second.c_str())),
            -1);
        ret.push_back(ol);
    }
    return ret;
}
void cObstacle::tourSpanningTree()
{
    raven::set::cRunWatch aWatcher("tourSpanningTree");

    int myBestCountRevisited = 1e7;
    vlink_t bestPath;
    std::vector<cOCell *> bestNodesRevisited;

    // loop over nodes
    // for (int spanstart = 0; spanstart < vN.size(); spanstart++)
    auto spanstart = myGraph.begin();
    {
        // need only check spanning trees rooted near a margin
        //  if (adjacent(vN[spanstart], vL).size() != 8)
        //      continue;

        // construct spanning tree starting at the node
        mySpanningTree = myGraph.spanningTree((*spanstart)->userName());

        // // connect spanning tree leaves
        auto connectedLeaves = mySpanningTree;
        vVertex_t leaves = connectedLeaves.leaves();

        std::cout << connectedLeaves.text();
        std::cout << "leaves ";
        for( vertex_t l : leaves )
        {
            std::cout << l->userName() << " ";
        }
        std::cout << "\n";

        for (int kv = 0; kv < leaves.size(); kv++)
            for (int kw = 0; kw < leaves.size(); kw++)
            {
                // no self cycles
                if (kv == kw)
                    continue;

                // check for unblocked connection
                auto va = connectedLeaves.adjacentOut(leaves[kv]);
                if (std::find(va.begin(), va.end(), leaves[kw]) == va.end())
                    continue;

                double cost = myGraph.edgeAttrDouble(leaves[kv], leaves[kw],0);
                connectedLeaves.addEdge(leaves[kv], leaves[kw],
                    std::to_string( cost ) );

            }

        // // loop over leaf nodes in spanning tree
        // // to find the path starting at a leaf node
        // // that visits every node
        // // with fewest revisits to the same nodes
        // findBestPath(leaves, connectedLeaves);

        // if (vPath.size() && (myNodesRevisited.size() < myBestCountRevisited))
        // {
        //     // found a path with fewer nodes visited multiple times
        //     myBestCountRevisited = myNodesRevisited.size();
        //     bestPath = vPath;
        //     bestNodesRevisited = myNodesRevisited;
        //     if (!myNodesRevisited.size())
        //         break;
    }

    // std::cout << spanstart << " of " << vN.size()
    //           << " revisited " << bestCountRevisited << "\n";

    // vPath = bestPath;
    // myNodesRevisited = bestNodesRevisited;
}