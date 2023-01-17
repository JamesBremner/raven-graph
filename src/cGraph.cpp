#include <stack>
#include <algorithm>
#include <sstream>
#include <queue>

#include "cGraph.h"

void cGraph::clear()
{
    myfDirected = false;
    vVertexName.clear();
    vEdgeDst.clear();
    vEdgeAttr.clear();
    vOutEdges.clear();
}

void cGraph::setEdges(const std::string &sEdges)
{
    clear();
    std::istringstream iss(sEdges);
    std::string n1, n2;
    iss >> n1 >> n2;
    while (iss.good())
    {
        addEdge(n1,n2);
        iss >> n1 >> n2;
    }
}
void cGraph::setEdges(
    const std::string &sEdges,
    int countAttributes)
{
    std::istringstream iss(sEdges);
    std::string n1, n2, sattr;

    clear();

    iss >> n1 >> n2;
    while (iss.good())
    {
        sattr = "";
        std::vector<std::string> vattr;
        std::string sattr_one;
        for (int k = 0; k < countAttributes; k++)
        {
            iss >> sattr_one;
            vattr.push_back(sattr_one);
        }
        vEdgeAttr.push_back(vattr);
        if( ! myfDirected )
            vEdgeAttr.push_back(vattr);
        addEdge(n1,n2);

        iss >> n1 >> n2;
    }
}
int cGraph::addEdge(
    vertex_t src,
    vertex_t dst,
    const std::string &sattr)
{
    //std::cout << "addedge indices " << src <<" "<< dst << "\n";
    int ei = vEdgeDst.size(); // index of new edge
    vEdgeDst.push_back(dst);  // store edge destination

    if (!sattr.empty())
        vEdgeAttr[ei][0] = sattr; // assumes just one attribute

    vOutEdges[src].push_back(ei); // store new out edge for source vertex

    return ei;
}
void cGraph::addEdge(
    const std::string &src,
    const std::string &dst)
{
    //std::cout << "addedge names " << src <<" "<< dst << "\n";
    vertex_t sv = findorAdd(src);
    vertex_t dv = findorAdd(dst);
    addEdge(sv, dv);

    /* An undirected ( default ) graph is modeled
      with a directed graph with every pair of connected vertices
      having two directed edges going in opposite directions
    */
    if (!myfDirected)
        addEdge(dv, sv);
}

vertex_t cGraph::findorAdd(const std::string &sn)
{
    int ret = -1;
    for (auto &n : vVertexName)
    {
        ret++;
        if (sn == n)
            return ret;
    }
    // vertex with this name absent
    // add it
    ret = vVertexName.size();
    vVertexName.push_back(sn);
    vOutEdges.push_back({});
    return ret;
}

std::string cGraph::text()
{
    std::stringstream ss;

    for (auto &n : vVertexName)
    {
        ss << "node " << n << " linked to ";
        for (auto &dst : adjacentOut(index(n)))
        {
            ss << vVertexName[dst] << " ";
        }
        ss << "\n";
    }
    return ss.str();
}

std::vector<std::pair<std::string, std::string>>
cGraph::getlinkedVerticesNames()
{
    std::vector<std::pair<std::string, std::string>> ret;

    int ni = -1;
    for (auto &n : vVertexName)
    {
        ni++;
        for (vertex_t w : adjacentOut(ni))
        {
            if( ! myfDirected )
                if( ni > w )
                    continue;
            ret.push_back(
                std::make_pair(
                    n,
                    vVertexName[w]));
        }
    }
    return ret;
}

void cGraph::bfs(
    const std::string &start,
    std::function<void(vertex_t)> visitor)
{
    // queue of vertices with adjacencies to be explored
    std::queue<int> Q;

    // true for vertex that has been visited
    std::vector<bool> visited(vVertexName.size(), false);

    int si = index(start);
    visitor(si);
    visited[si] = true;
    Q.push(si);

    while (Q.size())
    {
        int iv = Q.front();
        Q.pop();

        for (auto iw : adjacentOut(iv))
        {
            if (!visited[iw])
            {
                visitor(iw);
                visited[iw] = true;
                Q.push(iw);
            }
        }
    }
}

void cGraph::dfs(
    const std::string &start,
    std::function<void(int v)> visitor)
{
    // track visited vertices
    std::vector<bool> visited(vVertexName.size(), false);

    // vertices waiting to be visited
    std::stack<vertex_t> wait;

    /*  1 Start by putting one of the graph's vertices on top of a stack.
        2 Take the top vertex of the stack and add it to the visited list.
        3 Add adjacent vertices which aren't in the visited list to the top of the stack.
        4 Keep repeating steps 2 and 3 until the stack is empty.
    */

    wait.push(index(start));

    while (!wait.empty())
    {
        vertex_t v = wait.top();
        wait.pop();
        if (visited[v])
            continue;
        visitor(v);
        visited[v] = true;

        for (vertex_t w : adjacentAll(v))
            if (!visited[w])
                wait.push(w);

    }
}

vVertex_t cGraph::path(
    const std::string &start,
    const std::string &finish)
{
    return path(
        index(start),
        index(finish));
}
vVertex_t cGraph::path(
    vertex_t start,
    vertex_t finish)
{
    vVertex_t ret;

    // run Dijsktra finding path from start to every node
    auto pred = dijsktra(
        start);

    // pick out path to finish node
    int vi = finish;
    while (vi != start)
    {
        if (vi == -1)
        {
            ret.clear();
            return ret;
        }
        ret.push_back(vi);
        vi = pred[vi];
    }
    ret.push_back(start);

    std::reverse(ret.begin(), ret.end());

    return ret;
}

std::vector<int> cGraph::dijsktra(
    vertex_t start)
{
    // shortest distance from start to each node
    std::vector<double> dist(vVertexName.size(), INT_MAX);

    // previous node on shortest path to each node
    std::vector<int> pred(vVertexName.size(), -1);

    std::vector<bool> sptSet(vVertexName.size(), false); // sptSet[i] will be true if vertex i is included in shortest
                                                         // path tree or shortest distance from src to i is finalized

    // Distance of source vertex from itself is always 0
    dist[start] = 0;
    pred[start] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < vVertexName.size() - 1; count++)
    {
        // Pick the minimum distance vertex from the set of vertices not
        // yet processed. u is always equal to src in the first iteration.
        int min = INT_MAX, uidx;
        for (int vidx = 0; vidx < vVertexName.size(); vidx++)
            if (sptSet[vidx] == false && dist[vidx] <= min)
            {
                min = dist[vidx];
                uidx = vidx;
            }
        if (min == INT_MAX)
        {
            // no more nodes connected to start
            break;
        }

        // Mark the picked vertex as processed
        sptSet[uidx] = true;

        // Update dist value of the adjacent vertices of the picked vertex.
        for (auto vp : adjacentOut(uidx))
        {
            if (sptSet[vp])
                continue; // already processed

            // Update dist[v] only if total weight of path from src to  v through u is
            // smaller than current value of dist[v]
            double cost = 1;
            if (dist[uidx] + cost < dist[vp])
            {
                dist[vp] = dist[uidx] + cost;
                pred[vp] = uidx;
            }
        }
    }
    return pred;
}

vVertex_t cGraph::adjacentOut(vertex_t v)
{
    std::vector<int> ret;
    for( int ei : vOutEdges[v] )
    {
        ret.push_back( vEdgeDst[ei] );
    }
    return ret;
}
vVertex_t cGraph::adjacentIn(vertex_t v)
{
    vVertex_t ret;
    if (!myfDirected)
    {
        for (int t : vOutEdges[v])
            ret.push_back(t);
        return ret;
    }

    int i = -1;
    for (auto t : vOutEdges)
    {
        i++;
        for (auto ei : t)
        {
            if (ei == v)
                ret.push_back(i);
        }
    }
    return ret;
}
vVertex_t cGraph::adjacentAll(vertex_t v)
{
    vVertex_t ret(adjacentOut(v));

    if (myfDirected)
    {
        vVertex_t vin = adjacentIn(v);
        ret.insert(ret.end(), vin.begin(), vin.end());
    }
    return ret;
}

int cGraph::edgeIndex(
    vertex_t src,
    vertex_t dst)
{
    for (int ei : vOutEdges[src])
        if (vEdgeDst[ei] == dst)
            return ei;

    return -1;
}

double cGraph::edgeAttrDouble(
    vertex_t src,
    vertex_t dst,
    int attrIndex)
{
    int eindex = edgeIndex(src, dst);
    if (0 > eindex || eindex >= vEdgeAttr.size())
        return 1.0;
    if (0 > attrIndex || attrIndex >= vEdgeAttr.size())
        return 1.0;
    return atof(vEdgeAttr[eindex][attrIndex].c_str());
}

double cGraph::edgeAttrDouble(
    const std::string &src,
    const std::string &dst,
    int attrIndex)
{
    return edgeAttrDouble(
        findorAdd(src),
        findorAdd(dst),
        attrIndex);
}

vVertex_t cGraph::leaves()
{
    vVertex_t ret;
    for (int v = 0; v < vVertexName.size(); v++)
    {
        if (adjacentAll(v).size() == 1)
            ret.push_back(v);
    }
    return ret;
}

int cGraph::index(const std::string &sn) const
{
    int ret = 0;
    for (auto &n : vVertexName)
    {
        if (sn == n)
            return ret;
        ret++;
    }
    return -1;
}

std::string cGraph::userName(int v) const
{
    return vVertexName[v];
}

std::vector<std::vector<vertex_t>>
cGraph::dfs_cycle_finder(const std::string &start)
{
    std::vector<std::vector<vertex_t>> ret;

    // track visited vertices
    std::vector<bool> visited(vVertexName.size(), false);

    // vertices waiting to be processed
    std::stack<vertex_t> wait;

    // start at the beginning
    wait.push(index(start));

    // continue until no more vertices need processing
    while (!wait.empty())
    {
        vertex_t v = wait.top();
        wait.pop();
        int vi = v;
        if (!visited[vi])
        {
            visited[vi] = true;

            for (vertex_t w : adjacentOut(v))
            {
                if (!visited[w])
                {
                    wait.push(w);
                }
                else
                {
                    // previously visited node, check for ancestor
                    auto cycle = path(w, v);
                    if (cycle.size() > 0)
                    {
                        // found a cycle
                        cycle.push_back(w);
                        ret.push_back(cycle);
                    }
                }
            }
        }
    }
    return ret;
}

cGraph cGraph::spanningTree(const std::string &start)
{
    cGraph spanTree;

    // track visited vertices
    std::vector<bool> visited(vVertexName.size(), false);

    // add initial arbitrary link
    vertex_t v = index(start);
    auto va = adjacentAll(v);
    if (!va.size())
        throw std::runtime_error(
            "spanning tree start vertex unconnected");
    auto w = va[0];
    spanTree.addEdge(vVertexName[v], vVertexName[w]);
    visited[v] = true;
    visited[w] = true;

    // while nodes remain outside of span
    while (vVertexName.size() > spanTree.vertexCount())
    {
        double min_cost = INT_MAX;
        std::pair<vertex_t, vertex_t> bestLink;

        // loop over nodes in span
        for (int kv = 0; kv < vVertexName.size(); kv++)
        {
            if (!visited[kv])
                continue;
            v = kv;

            // loop over adjacent nodes not in span
            for (auto w : adjacentOut(v))
            {
                if (visited[w])
                    continue;

                double cost = edgeAttrDouble(v, w, 0);
                if (cost > 0)
                {
                    if (cost < min_cost)
                    {
                        min_cost = cost;
                        bestLink = std::make_pair(v, w);
                    }
                }
            }
        }

        // add cheapest link between node in tree to node not yet in tree
        spanTree.addEdge(
            vVertexName[bestLink.first],
            vVertexName[bestLink.second]);
        visited[bestLink.first] = true;
        visited[bestLink.second] = true;
    }

    return spanTree;
}

int cGraph::componentCount()
{
    int ret = 0;
    std::vector<bool> visited(vertexCount(), false);
    for (int v = 0; v < vVertexName.size(); v++)
    {
        // if this vertex already visited
        //  then it is part of a component that has already been counted
        if (visited[v])
        {
            continue;
        }

        // increment component count
        ret++;

        // vist all the vertices in the component
        dfs(
            vVertexName[v],
            [&](int n)
            {
                visited[n] = true;
            });
    }
    return ret;
}
