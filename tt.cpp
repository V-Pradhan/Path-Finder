#include <bits/stdc++.h>
using namespace std;

#define ROW 8
#define COL 8

vector<vector<int>> grid = {
    {0,1,0,1,0,1,1,0},
    {1,0,1,0,1,0,1,0},
    {0,0,0,0,1,0,0,0},
    {0,1,1,0,0,1,1,0},
    {0,1,0,0,0,1,0,0},
    {0,0,1,0,0,0,0,1},
    {0,1,0,1,0,0,0,0},
    {1,0,0,1,0,1,0,0}
};

vector<pair<int,int>> directions = {{0,1},{1,0},{0,-1},{-1,0}};

bool isValid(int r, int c) {
    return (r >= 0 && r < ROW && c >= 0 && c < COL && grid[r][c] == 0);
}

// BFS
vector<pair<int,int>> bfs(pair<int,int> start, pair<int,int> goal) {
    queue<pair<pair<int,int>, vector<pair<int,int>>>> q;
    set<pair<int,int>> visited;

    q.push(make_pair(start, vector<pair<int,int>>(1, start)));
    visited.insert(start);

    while (!q.empty()) {
        pair<pair<int,int>, vector<pair<int,int>>> front = q.front();
        q.pop();

        pair<int,int> curr = front.first;
        vector<pair<int,int>> path = front.second;

        if (curr == goal) return path;

        for (int i = 0; i < directions.size(); i++) {
            int nr = curr.first + directions[i].first;
            int nc = curr.second + directions[i].second;

            if (isValid(nr,nc) && !visited.count(make_pair(nr,nc))) {
                visited.insert(make_pair(nr,nc));

                vector<pair<int,int>> new_path = path;
                new_path.push_back(make_pair(nr,nc));

                q.push(make_pair(make_pair(nr,nc), new_path));
            }
        }
    }
    return vector<pair<int,int>>();
}

// DFS
vector<pair<int,int>> dfs(pair<int,int> start, pair<int,int> goal) {
    stack<pair<pair<int,int>, vector<pair<int,int>>>> st;
    set<pair<int,int>> visited;

    st.push(make_pair(start, vector<pair<int,int>>(1, start)));
    visited.insert(start);

    while (!st.empty()) {
        pair<pair<int,int>, vector<pair<int,int>>> top = st.top();
        st.pop();

        pair<int,int> curr = top.first;
        vector<pair<int,int>> path = top.second;

        if (curr == goal) return path;

        for (int i = 0; i < directions.size(); i++) {
            int nr = curr.first + directions[i].first;
            int nc = curr.second + directions[i].second;

            if (isValid(nr,nc) && !visited.count(make_pair(nr,nc))) {
                visited.insert(make_pair(nr,nc));

                vector<pair<int,int>> new_path = path;
                new_path.push_back(make_pair(nr,nc));

                st.push(make_pair(make_pair(nr,nc), new_path));
            }
        }
    }
    return vector<pair<int,int>>();
}

// Print Path
void printPath(vector<pair<int,int>> path, string name) {
    if (path.empty()) {
        cout << name << ": No Path Found\n";
        return;
    }

    cout << name << " Path: ";
    for (int i = 0; i < path.size(); i++) {
        cout << "(" << path[i].first << "," << path[i].second << ") ";
    }
    cout << "\nLength: " << path.size() << "\n\n";
}

int main() {
    int sr, sc, gr, gc;

    cout << "Enter START (row col): ";
    cin >> sr >> sc;

    cout << "Enter GOAL (row col): ";
    cin >> gr >> gc;

    pair<int,int> start = make_pair(sr, sc);
    pair<int,int> goal = make_pair(gr, gc);

    vector<pair<int,int>> bfs_path = bfs(start, goal);
    vector<pair<int,int>> dfs_path = dfs(start, goal);

    printPath(bfs_path, "BFS");
    printPath(dfs_path, "DFS");

    return 0;
}