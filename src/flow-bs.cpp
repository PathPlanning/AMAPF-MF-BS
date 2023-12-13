#include <bits/stdc++.h>
#define mk make_pair
#define fs first
#define sc second
using namespace std;
typedef long long ll;
typedef long double ld;
const int INF = 1e9;
int maxTimestep = 1e9;
const int MX_WIDTH = 2402500;
const int MX_HEIGHT = 1;
const int SZ = MX_WIDTH * MX_HEIGHT + 1;
int width;  // chanage it one time to the width of the map.
int height; // chanage it one time to the height of the map.
int heuristic[SZ];

// Classes with operators:
struct Edge
{
    int cellID, timeStep;
    Edge(int cellID, int timeStep) : cellID(cellID), timeStep(timeStep) {}
    bool operator==(const Edge &other) const
    {
        return cellID == other.cellID && timeStep == other.timeStep;
    }
};
bool operator<(Edge a, Edge b)
{
    return a.timeStep != b.timeStep ? a.timeStep < b.timeStep : a.cellID < b.cellID;
}
namespace std
{
    template <>
    struct hash<Edge>
    {
        auto operator()(const Edge &e) const -> size_t
        {
            return e.timeStep * SZ + e.cellID;
        }
    };
}
struct Interval
{
    int l, h;
    Interval() {}
    Interval(int l, int h) : l(l), h(h) {}
};
struct TimestepCostPair
{
    int timeStep;
    bool blockedToMove; // Added to mark the nodes which are blocked to use the move actions because they are created in the infinite blocked interval of a goal cell (and no inverse edge is used yet).
    TimestepCostPair() {}
    TimestepCostPair(int timeStep) : timeStep(timeStep)
    {
        blockedToMove = false;
    }
    TimestepCostPair(int timeStep, bool blockedToMove) : timeStep(timeStep), blockedToMove(blockedToMove) {}
};
bool operator<(TimestepCostPair a, TimestepCostPair b)
{
    return a.timeStep < b.timeStep;
}
struct Node
{
    int parent, cellID;
    TimestepCostPair timeStepCostPair;
    Node() {}
    Node(TimestepCostPair timeStepCostPair, int parent, int cellID) : timeStepCostPair(timeStepCostPair), parent(parent), cellID(cellID) {}
};
bool operator<(Node a, Node b)
{ // inversed to inverse the default priority_queue
    return a.timeStepCostPair.timeStep+heuristic[a.cellID] > b.timeStepCostPair.timeStep+heuristic[b.cellID];
}
const int SZBIG = 1000000;
// storage::
bool mapCell[SZBIG]; // cell=mapCell[id]: true -> obstacle, false -> free
vector<int> nebors[SZ];
int sinkID, mapSize;
vector<int> startCells;
bool isGoalCell[SZ];
// The edges will be stored in the 'edges[destination]'.
set<int> extraToGoalEdges;    // To the goal, it is enough to store the id of the extra nodes which are connected to goal.
set<Edge> edges[SZ];          // between other cells, it is neccessary to store the time step and the id of the next cell.
set<TimestepCostPair> visitedTimeSTeps[SZ];
set<int> blocks[SZ];
int timeAssigningGoalCell[SZ];
inline bool isSink(int cellID)
{
    return cellID == sinkID;
}
int freeCells[SZBIG];
void genAllNeighors()
{ // fill 'map' and 'isGoalCell' sequentially (in 1D arrays), 'width' and 'height' and you are ready to call this function.
    int cntNodes = 0, cntFree=0;
    int di[4] = {0, 0, 1, -1}, dj[4] = {1, -1, 0, 0};
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            if (!mapCell[cntNodes])
            {
                freeCells[cntNodes] = cntFree;
                ++cntFree;
            }
            ++cntNodes;
        }
    }
    sinkID = cntFree;
    cntNodes = 0;
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            if (!mapCell[cntNodes])
            {
                int cell = freeCells[cntNodes];
                for (int k = 0; k < 4; ++k)
                {
                    int ii = i + di[k];
                    int jj = j + dj[k];
                    int id = cntNodes + dj[k] + di[k] * width;
                    if (ii < height && ii >= 0 && jj < width && jj >= 0)
                    {
                        if (!mapCell[id])
                        {
                            nebors[cell].push_back(freeCells[id]);
                        }
                    }
                }
            }
            ++cntNodes;
        }
    }
}
pair<bool, Interval> checkTimestepAndGetUnvisitedSafeInterval(TimestepCostPair timeStepCostPair, int cellID)
{
    int prevBlock = -1, nxtBlock = maxTimestep;
    auto itr = blocks[cellID].lower_bound(timeStepCostPair.timeStep);
    if (itr != blocks[cellID].end())
    {
        nxtBlock = *itr;
    }
    if (itr != blocks[cellID].begin())
    {
        --itr;
        prevBlock = *itr;
    }
    int nxtVisitedTimestep = maxTimestep + 1;
    auto itrt = visitedTimeSTeps[cellID].upper_bound(TimestepCostPair(timeStepCostPair.timeStep));
    if (itrt != visitedTimeSTeps[cellID].begin())
    {
        --itrt;
        if (itrt->timeStep > prevBlock)
        {
            return mk(false, Interval());
        }
        ++itrt;
    }

    if (itrt != visitedTimeSTeps[cellID].end() && itrt->timeStep <= nxtBlock)
    {
        nxtVisitedTimestep = itrt->timeStep;
        return {true, Interval(timeStepCostPair.timeStep, nxtVisitedTimestep - 1)};
    }
    return {true, Interval(timeStepCostPair.timeStep, nxtBlock)};
}
bool checkTimestep(TimestepCostPair timeStepCostPair, int cellID)
{
    int prevBlock = -1;
    auto itr = blocks[cellID].lower_bound(timeStepCostPair.timeStep);
    if (itr != blocks[cellID].begin())
    {
        --itr;
        prevBlock = *itr;
    }
    auto itrt = visitedTimeSTeps[cellID].upper_bound(TimestepCostPair(timeStepCostPair.timeStep));
    if (itrt != visitedTimeSTeps[cellID].begin())
    {
        --itrt;
        if (itrt->timeStep > prevBlock) //&& prvCost <= timeStepCostPair.cost - timeStepCostPair.timeStep)
        {
            return false;
        }
    }
    return true;
}
bool isThereAnEdgeToGoalFromExtraNode(int cellID)
{
    return extraToGoalEdges.count(cellID);
}
bool isThereAnEdge(int source, int destination, int timeStep)
{
    return edges[destination].count(Edge(source, timeStep));
}
void visit(int cellID, TimestepCostPair timeStepCostPair)
{
    visitedTimeSTeps[cellID].insert(timeStepCostPair);
}
void getAllTimeSteps(int lowestTimeStep, int highestTimeStep, int cellID, vector<pair<int, bool>> &validtimeStepsWithMoveBlocked)
{
    if (isGoalCell[cellID] && timeAssigningGoalCell[cellID] < INF)
    {
        for (auto &it : blocks[cellID])
        {
            if (lowestTimeStep > highestTimeStep)
                break;
            if (it >= lowestTimeStep)
            {
                validtimeStepsWithMoveBlocked.push_back(mk(lowestTimeStep, false));
                lowestTimeStep = it + 2;
            }
        }
        if (lowestTimeStep <= highestTimeStep)
        {
            validtimeStepsWithMoveBlocked.push_back(mk(lowestTimeStep, true));
        }
    }
    else
    {
        for (auto &it : blocks[cellID])
        {
            if (lowestTimeStep > highestTimeStep)
                break;
            if (it >= lowestTimeStep)
            {
                validtimeStepsWithMoveBlocked.push_back(mk(lowestTimeStep, false));
                lowestTimeStep = it + 2;
            }
        }
        if (lowestTimeStep <= highestTimeStep)
        {
            validtimeStepsWithMoveBlocked.push_back(mk(lowestTimeStep, false));
        }
    }
}
void getAllTimestepCostPairs(int cellSource, int cellDestination, Interval it, vector<TimestepCostPair> &result)
{
    vector<pair<int, bool>> timeStepsWithMoveBlocked; // When we move to the infinite blocked interval of a goal cell (the last waiting before connecting to sink node), we don't allow the resulting node to make move actions.
    int lowestTimeStep, highestTimeStep;
    if (it.h % 2 == 0) // the high time step is in the copy 0 of the layer.
    {
        if (isThereAnEdge(cellDestination, cellSource, it.h - 1))
        {
            timeStepsWithMoveBlocked.push_back(mk(it.h - 1, it.h - 1 >= timeAssigningGoalCell[cellDestination]));
        }
        highestTimeStep = it.h;
    }
    else
    {
        if (isThereAnEdge(cellSource, cellDestination, it.h))
        {
            highestTimeStep = it.h - 1;
        }
        else
        {
            highestTimeStep = it.h + 1;
        }
    }
    if (it.l % 2)
    {
        if (isThereAnEdge(cellSource, cellDestination, it.l))
        {
            lowestTimeStep = it.l + 3;
        }
        else
        {
            lowestTimeStep = it.l + 1;
        }
    }
    else
    {
        if (isThereAnEdge(cellDestination, cellSource, it.l - 1))
            lowestTimeStep = it.l - 1;
        else
            lowestTimeStep = it.l + 2;
    }
    if (highestTimeStep >= lowestTimeStep)
    {
        getAllTimeSteps(lowestTimeStep, highestTimeStep, cellDestination, timeStepsWithMoveBlocked);
    }
    for (auto t : timeStepsWithMoveBlocked)
    {
        result.push_back(TimestepCostPair(t.first, t.second));
    }
}
void addPathEdges(vector<Node> &nodes, int sinkNodeID)
{
    Node node1, node2 = nodes[sinkNodeID];
    int par = node2.parent;
    vector<Node> pathNodes;
    pathNodes.push_back(node2);
    while (par != -1)
    {
        node1 = nodes[par];
        Node nodeTmp = node1;
        if (node1.timeStepCostPair.timeStep <= node2.timeStepCostPair.timeStep)
        {
            if (node2.timeStepCostPair.timeStep % 2)
            { // Case when go to the upper bound of the time interval then go downward using negative edge.
                for (int timeStep = node2.timeStepCostPair.timeStep + 1; timeStep >= node1.timeStepCostPair.timeStep; timeStep--)
                {
                    nodeTmp.timeStepCostPair.timeStep = timeStep;
                    pathNodes.push_back(nodeTmp);
                }
            }
            else
            {
                for (int timeStep = node2.timeStepCostPair.timeStep - 1; timeStep >= node1.timeStepCostPair.timeStep; timeStep--)
                {
                    nodeTmp.timeStepCostPair.timeStep = timeStep;
                    pathNodes.push_back(nodeTmp);
                }
            }
        }
        else
        {
            for (int timeStep = node2.timeStepCostPair.timeStep + 1; timeStep <= node1.timeStepCostPair.timeStep; timeStep++)
            {
                nodeTmp.timeStepCostPair.timeStep = timeStep;
                pathNodes.push_back(nodeTmp);
            }
        }
        node2 = node1;
        par = node1.parent;
    }
    reverse(pathNodes.begin(), pathNodes.end());
    for (unsigned int i = 0; i + 1 < pathNodes.size(); ++i)
    {
        node1 = pathNodes[i];
        node2 = pathNodes[i + 1];
        if (isGoalCell[node1.cellID])
        {
            if (timeAssigningGoalCell[node1.cellID] < node1.timeStepCostPair.timeStep)
            {
                for (int t = timeAssigningGoalCell[node1.cellID] + 1; t <= node1.timeStepCostPair.timeStep; ++t)
                {
                    edges[node1.cellID].insert({node1.cellID, t});
                    blocks[node1.cellID].insert(t);
                }
                timeAssigningGoalCell[node1.cellID] = node1.timeStepCostPair.timeStep;
            }
        }
        if (node1.timeStepCostPair.timeStep > node2.timeStepCostPair.timeStep)
        { // --> the edges is between map-cells (none-extra-none-goal nodes).
            edges[node1.cellID].erase({node2.cellID, node2.timeStepCostPair.timeStep});
            if (node1.cellID == node2.cellID)
            {
                blocks[node1.cellID].erase(node2.timeStepCostPair.timeStep);
            }
        }
        else // if (node1.timeStepCostPair.timeStep < node2.timeStepCostPair.timeStep)
        {    // --> the edge is between map-cells (none-extra-none-goal nodes).
            edges[node2.cellID].insert({node1.cellID, node1.timeStepCostPair.timeStep});
            if (node1.cellID == node2.cellID)
            {
                blocks[node1.cellID].insert(node1.timeStepCostPair.timeStep);
            }
        }
    }
    blocks[node2.cellID].insert(node2.timeStepCostPair.timeStep);
    edges[node2.cellID].insert({node2.cellID, node2.timeStepCostPair.timeStep});
    timeAssigningGoalCell[node2.cellID] = node2.timeStepCostPair.timeStep;
}

vector<Node> nodes;
priority_queue<Node> q;
int numOfExpansions;
void partialReset()
{
    // startCells.clear();
    // memset(isGoalCell, 0, sizeof isGoalCell);
    extraToGoalEdges.clear();
    for (int i = 0; i < SZ; ++i)
    {
        blocks[i].clear();
        edges[i].clear();
        // nebors[i].clear();
        timeAssigningGoalCell[i] = INF; // This value means that this goal is not used yet.
    }
}
void calHeuristics(){
    queue<pair<int, int>> q;
    for(int i=0; i<SZ; ++i){
        if(isGoalCell[i] && timeAssigningGoalCell[i]==INF){
            q.push({i,0});
        }
        heuristic[i]=-1;
    }
    while(!q.empty()){
        auto tmp = q.front();
        q.pop();
        if(heuristic[tmp.first]!=-1){
            continue;
        }
        heuristic[tmp.first]=tmp.second;
        for(auto it:nebors[tmp.first]){
            if(heuristic[it]==-1){
                q.push({it,tmp.second+1});
            }
        }
    }
}
int remainingGoals;
int solve(int min_t)
{
    maxTimestep = min_t*2-1;
    while (remainingGoals)
    {
        calHeuristics();
        // cout<<"Agents remaining: "<<goalsNum<<" "<<numOfExpansions<<endl;
        while (!q.empty())
        {
            q.pop();
        }
        nodes.clear();
        for (auto it : startCells)
        {
            q.push(Node(TimestepCostPair(0, 0), -1, it));
        }
        int sinkNodeID = -1;
        for (int i = 0; i <= sinkID; ++i)
        {
            visitedTimeSTeps[i].clear();
        }
        while (!q.empty())
        {
            Node tp = q.top();
            int nodeID = nodes.size();
            nodes.push_back(tp);
            q.pop();
            int currentCell = tp.cellID;
            TimestepCostPair timeStepCostPair = tp.timeStepCostPair;
            pair<bool, Interval> res = checkTimestepAndGetUnvisitedSafeInterval(timeStepCostPair, currentCell);
            if (!res.first)
            { // visited time step in ths cell.
                continue;
            }
            numOfExpansions++;
            if (isGoalCell[currentCell] && timeAssigningGoalCell[currentCell] == INF && res.second.h == maxTimestep)
            {
                sinkNodeID = nodeID;
                break;
            }
            visit(currentCell, timeStepCostPair);
            vector<int> &nebs = nebors[currentCell];
            Interval currentInterval = res.second;
            // special edge: inverse of wait action.
            if (isThereAnEdge(currentCell, currentCell, currentInterval.l - 1) || currentInterval.l - 1 >= timeAssigningGoalCell[currentCell])
            {
                q.push(Node(TimestepCostPair(currentInterval.l - 1, false), nodeID, currentCell));
            }
            // special edge: infinite node (came from infinite interval and projected to infinite blocked interval).
            if (tp.timeStepCostPair.blockedToMove)
            {
                if (timeStepCostPair.timeStep + 2 <= currentInterval.h)
                {
                    q.push(Node(TimestepCostPair(timeStepCostPair.timeStep + 2, true), tp.parent, currentCell));
                }
            }
            else
            {
                for (auto neborCell : nebs)
                {
                    vector<TimestepCostPair> result;
                    getAllTimestepCostPairs(currentCell, neborCell, currentInterval, result);
                    for (auto it : result)
                    {
                        if (checkTimestep(it, neborCell))
                            q.push(Node(it, nodeID, neborCell));
                    }
                }
            }
        }
        if (sinkNodeID == -1)
        {
            return false;
        }
        addPathEdges(nodes, sinkNodeID);
        remainingGoals--;
    }
    return true;
}
void readMap(string mapName)
{
    ifstream in(mapName.c_str());
    if (!in.good())
    {
        cerr<<"Couldn't open the file "<<mapName<<". Exit"<<endl;
        exit(0);
    }
    string type, height_, width_, map_, row;
    in >> type >> type >> height_ >> height >> width_ >> width >> map_;
    mapSize = height * width;
    int cnt = 0;
    for (int i = 0; i < height; ++i)
    {
        in >> row;
        for (int j = 0; j < width; ++j)
        {
            if (row[j] == '@' || row[j] == 'T')
            {
                mapCell[cnt++] = 1;
            }
            else
            {
                mapCell[cnt++] = 0;
            }
        }
    }
    in.close();
}
void readInputs(string fileName, int numOfAgents)
{
    ifstream in(fileName.c_str());
    if (!in.good())
    {
        cerr<<"Couldn't open the file "<<fileName<<". Exit"<<endl;
        exit(0);
    }
    string version;
    in >> version >> version;
    string num, map_;
    int w, h, r, c;
    double doubleNum;
    int cnt = 0;
    while (in >> num >> map_ && cnt++ < numOfAgents)
    {
        in >> w >> h;
        assert(w == width);
        assert(h == height);
        in >> c >> r;
        startCells.push_back(freeCells[r * width + c]);
        in >> c >> r;
        isGoalCell[freeCells[r * width + c]] = 1;
        in >> doubleNum;
    }
    in.close();
}
int num_of_agents(string fileName)
{
    ifstream in(fileName.c_str());
    if (!in.good())
    {
        cerr<<"Couldn't open the file "<<fileName<<". Exit"<<endl;
        exit(0);
    }
    string version;
    in >> version >> version;
    string num, map;
    int w, h, r, c;
    double doubleNum;
    int cnt = 0;
    while (in >> num >> map)
    {
        in >> w >> h;
        in >> c >> r;
        in >> c >> r;
        in >> doubleNum;
        ++cnt;
    }
    in.close();
    return cnt;
}
void reset()
{
    startCells.clear();
    memset(isGoalCell, 0, sizeof isGoalCell);
    for (int i = 0; i < SZ; ++i)
    {
        nebors[i].clear();
    }
}

vector<string> mapNames = {
    // "random_1550_1550"
    "random-800-800",
    "random-1100-1100"
    // "Berlin_1_256",
    // "Boston_0_256",
    // "brc202d",
    // "den520d",
    // "maze-128-128-10",
    // "orz900d",
    // "Paris_1_256",
    // "warehouse-20-40-10-2-2",
    // "w_woundedcoast",
    // "empty-8-8",
    // "empty-16-16",
    // "empty-32-32",
    // "ht_mansion_n",
    // "maze-32-32-2",
    // "maze-32-32-4",
    // "maze-128-128-1",
    // "maze-128-128-2",
    // "lak303d",
    // "lt_gallowstemplar_n",
    // "ost003d",
    // "random-32-32-10",
    // "random-32-32-20",
    // "random-64-64-20",
    // "warehouse-10-20-10-2-1",
    // "warehouse-20-40-10-2-1",
    // "room-32-32-4",
    // "room-64-64-8",
    // "room-64-64-16",
    // "ht_chantry",
    // "warehouse-10-20-10-2-2",
    // "random-64-64-10",
    // "empty-48-48",
    // "den312d"
};
map<string, int>estimates[26][1001];
void readEstimations(){
    ifstream in("../data/estimates.txt");
    if (!in.good())
    {
        cerr<<"Couldn't open the estimations file"<<std::endl;
        exit(0);
    }
    string map;
    int scen, numOfAgents, t;
    while (in >> map >> scen >> numOfAgents >> t)
    {
        estimates[scen][numOfAgents][map] = t;
    }
    in.close();
}

using namespace std::chrono;
int main(int argc, char *argv[])
{
    string method = "incremental_from_one";
    if(argc>1){
        string arg = argv[1];
        if(arg=="-s"){
            readEstimations();
            method = "from_estimations";
        }
        else if(arg=="-b"){
            method = "binary";
        }
        else{
            cerr<<"Unrecognized method is defined."<<endl;
        }
    }
    else{
        cerr<<"No method is defined."<<endl;
    }
    cerr<<"Method "<<method<<" is used."<<endl;
    ofstream resultscout("../results/results*2.csv");
    resultscout << "Map,TestNum,NumOfAgents,Cost,Runtime,Expansions\n";
    cout << "Map,TestNum,NumOfAgents,Cost,Runtime,Expansions\n";
    for (unsigned int map_id = 0; map_id < mapNames.size(); ++map_id)
    {
        string mapName = mapNames[map_id];
        reset();
        for (int scen = 1; scen <= 1; ++scen)
        {
            int maximumNumOfAgents = num_of_agents("../data/mapf-scen-random/scen-random/" + mapName + "-random-" + to_string(scen) + ".scen");
            cout<<maximumNumOfAgents<<endl;
            for(int numOfAgents=1; true; numOfAgents = min(numOfAgents*2, maximumNumOfAgents))
            {
                readMap("../data/mapf-map/" + mapName + ".map");
                genAllNeighors();
                readInputs("../data/mapf-scen-random/scen-random/" + mapName + "-random-" + to_string(scen) + ".scen", numOfAgents);
                auto start = high_resolution_clock::now();
                numOfExpansions = 0;
                int makespan = -1;
                if(method == "binary"){
                    int st = 1, nd = width * height + numOfAgents - 2;
                    while(st<nd){
                        int T = (st+nd)/2;
                        remainingGoals = numOfAgents;
                        partialReset();
                        if(solve(T)){
                            nd = T;
                        }
                        else{
                            st = T+1;
                        }
                    }
                    makespan = nd;
                }
                else if(method == "from_estimations"){
                    int T = estimates[scen][numOfAgents][mapName];
                    remainingGoals = numOfAgents;
                    partialReset();
                    while(!solve(T))++T;
                    makespan = T;
                }
                else{ // method == "incremental_from_one"
                    remainingGoals = numOfAgents;
                    partialReset();
                    int T = 1;
                    while(!solve(T))++T;
                    makespan = T;
                }
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                // if (duration.count() > 30000000)
                //     break;
                resultscout << mapName << "," << scen << "," << numOfAgents << "," << makespan << "," << int(duration.count() / 1000) << ","<<numOfExpansions<<endl;
                cout << mapName << "," << scen << "," << numOfAgents << "," << makespan << "," << int(duration.count() / 1000)<< ","<<numOfExpansions << endl;
                reset();
                if(numOfAgents==maximumNumOfAgents){
                    break;
                }
            }
            reset();
        }
    }
    resultscout.close();
}