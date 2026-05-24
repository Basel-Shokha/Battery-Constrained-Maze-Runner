#include "External Libraries/json.hpp"
#include <vector>
#include <list>

using std::vector;
using nlohmann::json;
using std::string;

struct Helper {
    int node;
    int weight;
    std::list<int> subPath;
};


using Graph = vector<vector<Helper>>;

struct info {
    int dv;
    int parent;
};

class MazeSolver {

public:

    MazeSolver(json& req);
    void dumpSolution(json& response);

/*
private:
    */
    Graph condenseGraph();
    info* BFS(int start, Graph& graph);
    info* DIJKSTRA(int start, Graph& graph);
    std::vector<std::pair<int,bool>> solveMaze();


    int rows;
    int columns;
    int start;
    int end;
    int batteryCapacity;

    Graph maze;
    vector<bool> active;
    vector<int> stations;
    vector<int> finalPath;

};
