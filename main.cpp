#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <map>
#include <cmath>

#define X_MAX 9
#define Y_MAX 9

using namespace std;

int perceptionZone; // perception zone of Thanos
int X, Y; // coordinates of infinity stone
const int dx[4] = {1, -1, 0, 0}; // change position for Thanos on X axis
const int dy[4] = {0, 0, 1, -1}; // change position for Thanos on Y axis
// Map to define properties of different items on the grid
map<char, bool> items = {
        {'I', true},
        {'.', true},
        {'H', false},
        {'T', false},
        {'M', false},
        {'S', true},
        {'P', false},
};

struct Point { // structure for information of items in table
    int x;
    int y;
    char item;

    Point(int xx, int yy, char it);
};
// Constructor for the Point struct
Point::Point(int xx, int yy, char it) {
    x = xx;
    y = yy;
    item = it;
}

struct Node { // structure for information about cells for A* algorithm
    int x; // coord x
    int y; //coord y
    int gCost; // Cost to reach this cell from the start
    int hCost; // Heuristic cost to reach the goal
    int fCost; // Total cost (gCost + hCost)
    Node* parent = nullptr;  // Parent node (used for path reconstruction)

    // Operator overloads for comparison
    bool operator>(const Node* other) const{
        // Compare based on fCost, hCost, and gCost
        if (fCost > other->fCost) return true;
        else if (fCost < other->fCost) return false;
        else if (hCost > other->hCost) return true;
        else if (hCost < other->hCost) return false;
        else if (gCost > other->gCost) return true;
        else if (gCost < other->gCost) return false;
        return false;
    }
    bool operator<(const Node* other) const{
        // Compare based on fCost, hCost, and gCost
        if (fCost < other->fCost) return true;
        else if (fCost > other->fCost) return false;
        else if (hCost < other->hCost) return true;
        else if (hCost > other->hCost) return false;
        else if (gCost < other->gCost) return true;
        else if (gCost > other->gCost) return false;
        return false;
    }
    bool operator==(const Node* other)const{
        // Check if two nodes have the same coordinates
        if (this->x == other->x && this->y == other->y) return true;
        return false;
    }
    Node(int x_1, int y_1, int g, Node* par);
};
// Constructor for the Node struct
Node::Node(int x_1, int y_1, int g, Node* par) {
    x = x_1;
    y = y_1;
    gCost = g;
    parent = par;
    hCost = 0;  // Initialize hCost
    fCost = 0;  // Initialize fCost

}
// 2D matrix to represent the grid
vector<vector<Point*>> matrix(X_MAX, vector<Point*>(Y_MAX));

// Comparator for the min priority queue used in A*
struct comp{
    bool operator()(const Node* node, const Node* other) const{
        if (node->fCost < other->fCost) return false; // first consider g(x) + h(x) cost
        else if (node->fCost > other->fCost) return true;
        else if (node->hCost < other->hCost) return false; // next consider h(x) cost
        else if (node->hCost > other->hCost) return true;
        else if (node->gCost < other->gCost) return false; // next consider g(x) cost
        else if (node->gCost > other->gCost) return true;
        else if (matrix[node->x][node->y]->item == 'S') return true; // if these cells have the same costs,
                                                                    // therefore take with shield
        else if (matrix[other->x][other->y]->item == 'S') return true;
        return false;
    }
};
set<Node*> closed; // Set to store closed (visited) nodes
priority_queue<Node*, vector<Node*>, comp> open; // Priority queue for open nodes
bool isValidCoordinate(int x, int y) { // check that moving consider in table size
    return (x >= 0 && x < X_MAX && y >= 0 && y < Y_MAX);
}

int calculateHCost(const Node& node, const Node& goal) { // calculate manhattan distance from current and goal cells
    return abs(node.x - goal.x) + abs(node.y - goal.y);
}

bool checkSuperHeroes(int x, int y) { // Check if superheroes (Thor or Hulk) are in proximity to the given coordinates
    int dxx[] = {1, 1, 1, -1, -1, -1, 0, 0};
    int dyy[] = {0, 1, -1, 0, 1, -1, 1, -1};

    for (int i = 0; i < 8; i++) {
        int newX = x + dxx[i];
        int newY = y + dyy[i];

        if (matrix[newX][newY]->item == 'M') {
            return false;
        } else if (matrix[newX][newY]->item == 'T' || matrix[newX][newY]->item == 'H') {
            return true;
        }
    }
    return false;
}
bool shield = false; // existing shield or not
void get_back_for_next_move(Node* current, Node* previous){ // Function to backtrack to a previous cell with a lower cost
    vector<Node> path_from_next_to_zero; // path for previous node to initial
    vector<Node> path; // final path
    Node* cur = current->parent; // take parent of new node
    Node* start = new Node{0, 0, 0, nullptr}; // initialize start node
    while (cur->x != start->x || cur->y != start->y){ // find all parents from new node to start
        path_from_next_to_zero.push_back(*cur);
        cur = cur->parent;
    }
    // next find intersection between new node and previous
    path_from_next_to_zero.push_back(*start);
    cur = previous->parent; // take parent of the previous node
    path.push_back(*cur); //
    int ind = -1;
    for (int i = 0; i < path_from_next_to_zero.size();i++ ){
        if (*cur == &path_from_next_to_zero[i]) {
            ind = i;
            break;
        }
    }
    while (ind == -1 && cur != start){
        cur = cur->parent;
        path.push_back(*cur);
        for (int i = 0; i < path_from_next_to_zero.size();i++ ){
            if (*cur == &path_from_next_to_zero[i]) {
                ind = i;
                break;
            }
        }
    }
    //add intersections nodes in
    for (int i = ind - 1; i>= 0; i--){
        path.push_back(path_from_next_to_zero[i]);
    }
    // print moves for backtracking
    for (auto & i : path){
        cout << "m " << i.x << " " << i.y << endl;
        int n;
        cin >> n;
        int xx, yy;
        char c;
        for (int j = 0; j < n; j++) {
            cin >> xx >> yy >> c;
            if(c == 'S'){
                shield = true;
            }
            matrix[xx][yy] =  new Point{xx, yy, c};
        }
    }
}
// A* search algorithm to find the path from start to goal
vector<Node*> AStar(Node* start, Node* goal) {
    // Initialize grid and priority queue
    vector<Node*> path;
    //this map for save all costs
    map<pair<int, int>, Node*> nodeMap;
    //initialize grid
    for (int i = 0; i < X_MAX; i++){
        for (int j = 0; j < Y_MAX; j++){
            matrix[i][j] = new Point {i, j, '.'};
        }
    }
    open.push(start);
    nodeMap[make_pair(start->x, start->y)] = start;
    Node previous = *start;
    Node* current = start;
    //
    while (!open.empty()) {
        current= open.top();
        open.pop();
        //if the distance between current and previous nodes more than 2, we should reach current node step by step
        if (abs(current->x - previous.x) > 1 || abs(current->y - previous.y) > 1){
            get_back_for_next_move(current, &previous);
        }
        //save previous node
        previous = *current;
        //print new move
        cout << "m " << current->x << " " << current->y << endl;
        //if we reach coordinates of Infinity stone, the break process
        if (current->x == goal->x && current->y == goal->y) {
            Node *node = current;
            while (node != nullptr) {
                path.push_back(node);
                node = node->parent;
            }
            reverse(path.begin(), path.end());
            break;
        }
        //this read information about grid from console and fill the map
        int n;
        cin >> n;
        int xx, yy;
        char c;
        for (int i = 0; i < n; i++) {
            cin >> xx >> yy >> c;
            auto *point = new Point{xx, yy, c};
            matrix[xx][yy] = point;
        }
        //in this for loop consider 4 directions of moves (-1,0), (1,0), (0,-1), (0,1)
        for (int i = 0; i < 4; i++) {
            //change current coordinates on dx and dy
            int x = current->x + dx[i];
            int y = current->y + dy[i];
            //this condition check, that changing coordinates placed in grid 9x9, and these coord were not attended,
            //and new cell is empty, consists stone or shield.
            if (isValidCoordinate(x, y) && closed.find(nodeMap[make_pair(x, y)]) == closed.end() &&
                (matrix[x][y]->item == '.' || matrix[x][y]->item == 'I' || matrix[x][y]->item == 'S' ||
                 (shield && matrix[x][y]->item == 'P' && checkSuperHeroes(current->x, current->y)))) {
                int gCost = current->gCost + 1;
                // if new cell consist stone, then visit this cell
                if (matrix[x][y]->item == 'S') {
                    //make that we took shield
                    shield = true;
                    //take node information from map
                    Node *neighbor = nodeMap[make_pair(x, y)];
                    //if new node does not exist in map, then calculate all costs and add in priority queue
                    if (neighbor == nullptr) {
                        neighbor = new Node(x, y, gCost, current);
                        neighbor->hCost = calculateHCost(*neighbor, *goal);
                        neighbor->fCost = neighbor->gCost + neighbor->hCost;
                        open.push(neighbor);
                        nodeMap[make_pair(x, y)] = neighbor;
                    } //otherwise improve gCost if we visited this cell
                    else if (gCost < neighbor->gCost) {
                        neighbor->gCost = gCost;
                        neighbor->parent = current;
                        neighbor->fCost = neighbor->gCost + neighbor->hCost;
                    }
                } else {
                    //also as above
                    Node *neighbor = nodeMap[make_pair(x, y)];
                    if (neighbor == nullptr) {
                        neighbor = new Node(x, y, gCost, current);
                        neighbor->hCost = calculateHCost(*neighbor, *goal);
                        neighbor->fCost = neighbor->gCost + neighbor->hCost;
                        open.push(neighbor);
                        nodeMap[make_pair(x, y)] = neighbor;
                    } else if (gCost < neighbor->gCost) {
                        neighbor->gCost = gCost;
                        neighbor->parent = current;
                        neighbor->fCost = neighbor->gCost + neighbor->hCost;
                    }
                }
            }
        }
        closed.insert(current);// Mark the current node as closed
    }
    return path;
}
int main() {
    // Read input for perception zone and the location of the infinity stone
    cin >> perceptionZone;
    cin >> X >> Y;
    Node* start = new Node(0, 0, 0, nullptr);
    Node* goal = new Node(X, Y, 0, nullptr);
    // Call the A* function to find the path
    vector<Node*> path = AStar(start, goal);
    if (path.empty()){
        cout << "e " << -1;
    }else {
        cout << "e " << path.size() - 1;
    }
    return 0;
}
