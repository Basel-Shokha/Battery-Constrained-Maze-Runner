#include "MazeSolver.h"
#include <queue>
#include <climits>
#include <algorithm>


#define TOINDEX(i,j) (((i)*(columns) + (j)))
#define TOROW(k) (k/(columns))
#define TOCOLUMN(k) (k%(columns))

using nlohmann::json;
static const int infinity = INT_MAX;

MazeSolver::MazeSolver(json& req) : rows(req["config"]["rows"].get<int>()),
columns(req["config"]["columns"].get<int>()), batteryCapacity(req["config"]["battery_capacity"].get<int>()),
start(TOINDEX(req["start"][0].get<int>(), req["start"][1].get<int>())),
end(TOINDEX(req["end"][0].get<int>(), req["end"][1].get<int>())), stations(req["charging_stations"].size()),
active(rows*columns), maze(rows*columns) {

    for (int i = 0; i < req["charging_stations"].size(); i++) {
        stations[i] = TOINDEX(req["charging_stations"][i][0].get<int>(), req["charging_stations"][i][1].get<int>());
    }


    json activeCells = req["active_cells"];
    for (int i = 0; i < activeCells.size(); i++) { //all inited to false at start cuz of resizing
        active[TOINDEX(activeCells[i][0].get<int>(), activeCells[i][1].get<int>())] = true;
    }


    for (int i = 0; i < req["active_cells"].size(); i++) {
        int r = activeCells[i][0].get<int>();
        int c = activeCells[i][1].get<int>();

        int index = TOINDEX(r, c);

        if (r-1 >= 0 && active[TOINDEX(r-1, c)]) {
            maze[index].push_back({TOINDEX(r-1,c), 1});
        }
        if (c-1 >= 0 && active[TOINDEX(r, c-1)]) {
            maze[index].push_back({TOINDEX(r,c-1), 1});
        }
        if (c+1 < columns && active[TOINDEX(r, c+1)]) {
            maze[index].push_back({TOINDEX(r,c+1), 1});
        }
        if (r+1 < rows && active[TOINDEX(r+1, c)]) {
            maze[index].push_back({TOINDEX(r+1,c), 1});
        }
    }
        json& walls = req["walls"];
        for (int j = 0; j < walls.size(); j++) {
            json& cell1 = walls[j]["between"][0];
            json& cell2 = walls[j]["between"][1];
            if (cell1[0].get<int>() == -1 || cell2[0].get<int>() == -1 ||
                cell1[1].get<int>() == -1 || cell2[1].get<int>() == -1 ||
                cell1[0].get<int>() == rows || cell2[0].get<int>() == rows ||
                cell1[1].get<int>() == columns || cell2[1].get<int>() == columns) {
                continue;
            }
            int index1 = TOINDEX(cell1[0].get<int>(), cell1[1].get<int>());
            int index2 = TOINDEX(cell2[0].get<int>(), cell2[1].get<int>());


            for (int k = 0 ; k < maze[index1].size(); k++) {
                if (maze[index1][k].node == index2) {
                    maze[index1].erase(maze[index1].begin() + k);
                }
            }
            for (int i = 0 ; i < maze[index2].size(); i++) {
                if (maze[index2][i].node == index1) {
                    maze[index2].erase(maze[index2].begin() + i);
                }
            }
        }
}

void MazeSolver::dumpSolution(json& response) {
    std::vector<std::pair<int, bool>> path = solveMaze();
    if (path.empty()) {
        response["feasible"] = false;
        response["message"] = "cannot make it";
        response["battery_consumed"] = 0;
        response["path"] = json::array();
    } else {
        response["feasible"] = true;
        response["message"] = "SUCCESSFULLY FOUND A PATH";
        response["battery_consumed"] = path.size() - 1;

        response["path"] = json::array();
        for (int i = 0 ; i < path.size(); i++) {
            json leg;

            leg["step"] = i;
            leg["cell"] = json::array();
            leg["cell"].push_back(TOROW(path[i].first));
            leg["cell"].push_back(TOCOLUMN(path[i].first));

            if (i == 0) {
                leg["type"] = "start";
            } else if (i == path.size() - 1) {
                leg["type"] = "destination";
            } else {
                if (path[i].second) {
                    leg["type"] = "charge";
                } else {
                    leg["type"] = "move";
                }
            }

            response["path"].push_back(leg);
        }
    }
}




Graph MazeSolver::condenseGraph() { /// to be reviewed
    Graph condensedGraph(2 + stations.size()); // at 0 and 1 start and end

    info* bfs_info = BFS(start, maze); //from stations
    if (bfs_info[end].dv <=  batteryCapacity) {
        condensedGraph[0].push_back({1, bfs_info[end].dv}); // indices are mapped. start,end are 0,1
        int p = end;
        while (bfs_info[p].parent != p) {
            condensedGraph[0].back().subPath.push_front(p);
            p = bfs_info[p].parent;
        }
        delete[] bfs_info;
        return condensedGraph; //exit early, there is a path with no charging needed..
    }
    for (int i = 0 ; i < stations.size(); i++) {
        int weight = bfs_info[stations[i]].dv;
        if (weight <=  batteryCapacity) {
            condensedGraph[0].push_back({i+2, weight}); // indiecs are mapped. start,end are 0,1

            int p = stations[i];
            while (bfs_info[p].parent != p) {
                condensedGraph[0].back().subPath.push_front(p);
                p = bfs_info[p].parent;
            }
        }
    }

    delete[] bfs_info;

    for (int i = 0 ; i < stations.size(); i++) {
        bfs_info = BFS(stations[i], maze);
        for (int j = 0 ; j < stations.size(); j++) {
            if (j!=i) {
                int weight = bfs_info[stations[j]].dv;

                if (weight <=  batteryCapacity) {
                    condensedGraph[i+2].push_back({j+2, weight}); // indices are mapped. start,end are 0,1
                    int p = stations[j];
                    while (bfs_info[p].parent != p) {
                        condensedGraph[i+2].back().subPath.push_front(p);
                        p = bfs_info[p].parent;
                    }
                }
            }

            if (j == 0) {
                if (bfs_info[end].dv <=  batteryCapacity) {
                    condensedGraph[i+2].push_back({1, bfs_info[end].dv}); // indiecs are mapped. start,end are 0,1
                    int p = end;
                    while (bfs_info[p].parent != p) {
                        condensedGraph[i+2].back().subPath.push_front(p);
                        p = bfs_info[p].parent;
                    }
                }
            }
        }

        delete[] bfs_info;
    }

    return condensedGraph;

}

info* MazeSolver::BFS(int start, Graph& graph) {
    std::queue<int> q;
    q.push(start);

    info* bfs_info = new info[graph.size()];

    for (int i = 0; i < graph.size(); i++) {
        bfs_info[i].dv = infinity;
        bfs_info[i].parent = -1;
    }

    bfs_info[start].dv = 0;
    bfs_info[start].parent = start;

    while (!q.empty()) {
        int node = q.front();
        q.pop();
        for (int i = 0 ; i < graph[node].size(); i++) {
            int node2 = graph[node][i].node;
            if (bfs_info[node2].dv >  bfs_info[node].dv + 1) {
                bfs_info[node2] = {bfs_info[node].dv + 1,node};
                q.push(node2);
            }
        }


    }

    return bfs_info;
}

using HeapNode = std::pair<int, int>;
info* MazeSolver::DIJKSTRA(int start, Graph& graph) {
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<HeapNode>> minHeap;
    minHeap.push({0, start});

    info* dijkstra_info = new info[graph.size()];
    for (int i = 0; i < graph.size(); i++) {
        dijkstra_info[i].dv = infinity;
        dijkstra_info[i].parent = -1;
    }
    dijkstra_info[start].dv = 0;
    dijkstra_info[start].parent = start;

    while (!minHeap.empty() && minHeap.top().first != infinity) {
        int node = minHeap.top().second;
        minHeap.pop();
        for (int i = 0 ; i < graph[node].size(); i++) {
            int node2 = graph[node][i].node;
            int weight = graph[node][i].weight;
            if (dijkstra_info[node2].dv >  dijkstra_info[node].dv + weight) {
               dijkstra_info[node2] = {dijkstra_info[node].dv + weight, node};
                minHeap.push({dijkstra_info[node2].dv, node2});
            }
        }
    }


    return dijkstra_info;
}



std::vector<std::pair<int,bool>> MazeSolver::solveMaze() {
    Graph condensedGraph = condenseGraph();
    std::vector<std::pair<int, bool>> path; //true means charging at it

    info* condensedGraphInfo = DIJKSTRA(0, condensedGraph);
    if (condensedGraphInfo[1].dv == infinity) {
        delete[] condensedGraphInfo;
        return path;
    }

    std::list<Helper*> condensedPath;
    int node2 = 1;
    int node1 = condensedGraphInfo[node2].parent;
    while (node1 != node2) {
        Helper* helper = nullptr;
        for (int i = 0 ; i < condensedGraph[node1].size(); i++) {
            if (condensedGraph[node1][i].node == node2) {
                helper = &condensedGraph[node1][i];
                break;
            }
        }
        assert(helper != nullptr);
        condensedPath.push_front(helper);
        node2 = node1;
        node1 = condensedGraphInfo[node1].parent;
    }
    delete[] condensedGraphInfo;


    path.push_back({start, false});


    for (Helper* elem : condensedPath) {
        std::list<int>& nodes = elem->subPath;
        for (int n : nodes) {
            path.push_back({n, false});
        }
        if (elem->node > 1) {
            path[path.size() - 1] = {stations[elem->node-2], true};
        }
    }

    return path;
}

