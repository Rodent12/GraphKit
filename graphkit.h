#ifndef GRAPH_H
#define GRAPH_H

#include<iostream>
#include<vector>
#include<string>
#include<stack>
#include<queue>
#include<climits>

using namespace std;

class Graph{
private:
    vector<vector<vector<int>>> adjList;
    vector<vector<int>> edges;
    int number_of_nodes;
    int number_of_edges;
    bool isDirected;
    bool isWeighted;

public:
    Graph(){
        number_of_nodes=0;
        number_of_edges=0;
        isDirected = false;
        isWeighted = false;
    }

    Graph(int n,int m,bool isDirected,bool isWeighted){
        number_of_nodes = n;
        number_of_edges = m;
        this->isDirected = isDirected;
        this->isWeighted = isWeighted;
        cout << "Enter " << m <<  " edges : " << endl;
        for(int i=0;i<m;i++){
            int a,b,w;
            vector<int> edge;
            if(!isWeighted){
                cin >> a >> b;
                if(a>n-1 || b>n-1){cout << "Error : Only enter nodes with number(0,"+to_string(n-1)+")" << endl; return;}
                edge.push_back(a);
                edge.push_back(b);
                edge.push_back(-1);
            }
            else{
                cin >> a >> b >> w;
                if(a>n-1 || b>n-1){cout << "Error : Only enter nodes with number(0,"+to_string(n-1)+")" << endl; return;}
                edge.push_back(a);
                edge.push_back(b);
                if(w < 0){cout << "Error : Weights can only be positive." << endl; return;}
                edge.push_back(w);
            }
            edges.push_back(edge);
        }

        vector<vector<vector<int>>> adj(n);

        for(auto it : edges){
            adj[it[0]].push_back({it[1],it[2]});
            if(!isDirected){adj[it[1]].push_back({it[0],it[2]});}
        }

        adjList = adj;

    }

    Graph(int n,bool isDirected,vector<vector<int>>&edges){
        number_of_nodes = n;
        this->isDirected = isDirected;
        this->edges = edges;
        this->number_of_edges = edges.size();
        this->isWeighted = 0;


        vector<vector<vector<int>>> adj(n);

        for(auto it : edges){
            if(it.size()==2){
                adj[it[0]].push_back({it[1],-1});
                if(!isDirected){adj[it[1]].push_back({it[0],-1});}
            }
            else{
                this->isWeighted = 1;
                adj[it[0]].push_back({it[1],it[2]});
                if(!isDirected){adj[it[1]].push_back({it[0],it[2]});}
            }
        }

        adjList = adj;

    }

    void getGraph(){
        for(int i=0;i<number_of_nodes;i++){
            cout << i << " -> ";
            for(auto it : adjList[i]){
                cout << "{" << it[0] << " " << it[1] << "}" << " ";
            }
        }
    }

    int get_number_of_nodes(){
        return number_of_nodes;
    }

    int get_number_of_edges(){
        return number_of_edges;
    }

    bool Directed(){
        return isDirected;
    }

    bool Weighted(){
        return isWeighted;
    }

    bool isCycle(int s,vector<bool>&vis,int parent){
        vis[s] = 1;
        for(auto it : adjList[s]){
            if(!vis[it[0]]){
                if(isCycle(it[0],vis,s)){
                    return true;
                }
            }
            else if(it[0] != parent){
                return true;
            }
        }
        return false;
    }

    bool isCycle(int s,vector<bool>&dfsvis,vector<bool>&vis){
        vis[s] = 1;
        dfsvis[s] = 1;
        for(auto it : adjList[s]){
            if(!vis[it[0]] && isCycle(it[0],dfsvis,vis)){
                return true;
            }
            else if(dfsvis[it[0]]){
                return true;
            }
        }
        dfsvis[s] = 0;
        return false;
    }

    bool detectCycle(){
        bool ans = 0;
        if(isDirected){
            vector<bool>vis(number_of_nodes);
            vector<bool>dfsvis(number_of_nodes);
            for(int i=0;i<number_of_nodes;i++){
                if(!vis[i]){
                    ans |= isCycle(i,dfsvis,vis);
                }
            }
        }
        else{
            vector<bool>vis(number_of_nodes);
            for(int i=0;i<number_of_nodes;i++){
                if(!vis[i]){
                    ans |= isCycle(i,vis,-1);
                }
            }
        }
        return ans;
    }

    void topologicalSort(int i,vector<bool>&vis,stack<int>&s){
        vis[i] = 1;
        for(auto it : adjList[i]){
            if(!vis[it[0]]){
                topologicalSort(it[0],vis,s);
            }
        }
        s.push(i);
    }

    vector<int> getTopologicalSort(){

        vector<int> ans;
        stack<int> s;
        vector<bool> vis(number_of_nodes);

        for(int i=0;i<number_of_nodes;i++){
            if(!vis[i]){
                topologicalSort(i,vis,s);
            }
        }
        
        while(!s.empty()){
            ans.push_back(s.top());
            s.pop();
        }

        return ans;

    }

    vector<int> shortestPath(int s){
        // Djikstra's Algorithm
        if(!isWeighted){cout << "Not applicable to unweighted graphs" << endl; return {};}
        vector<int> dist(number_of_nodes,INT_MAX);
        vector<bool> vis(number_of_nodes,0);
        queue<int> q;
        q.push(s);
        dist[s] = 0;
        vis[s] = 1;
        while(!q.empty()){
            int f = q.front();
            q.pop();
            for(auto it : adjList[f]){
                if(!vis[it[0]]){
                    dist[it[0]] = min(dist[it[0]],dist[f]+it[1]);
                }
            }
            int minidx = -1;
            int min = INT_MAX;
            for(int i=0;i<number_of_nodes;i++){
                if(!vis[i] && min >= dist[i]){
                    min = dist[i];
                    minidx = i;
                }
            }
            if(minidx == -1){break;}
            vis[minidx] = 1;
            q.push(minidx);
        }
        return dist;
    }

    vector<int> minimumSpanningTree(){
        if(isDirected){cout << "Error : Not applicable to directed graphs" << endl; return {};}
        vector<int> key(number_of_nodes, INT_MAX); // Key values used to pick minimum weight edge
        vector<int> parent(number_of_nodes, -1); // Array to store constructed MST
        vector<bool> inMST(number_of_nodes, false); // To represent set of vertices not yet included in MST

        // Min-heap priority queue
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

        // Start with the first node
        key[0] = 0;
        pq.push({0, 0}); // {key, node}

        while (!pq.empty()) {
            int u = pq.top().second; // Extract the node with the minimum key value
            pq.pop();

            inMST[u] = true; // Include this node in MST

            // Traverse all adjacent vertices of the extracted vertex u
            for (auto it : adjList[u]) {
                int v = it[0];
                int weight = it[1];
                // If v is not in MST and weight of (u,v) is smaller than current key of v
                if (!inMST[v] && weight < key[v]) {
                    key[v] = weight;
                    pq.push({key[v], v});
                    parent[v] = u;
                }
            }
        }

        return parent; // The parent array represents the MST
    }

    bool isBipartite() {
        if(isDirected){cout << "Error : Not applicable to directed graphs" << endl; return 0;}
        vector<int> color(number_of_nodes, -1); // -1 indicates no color is assigned

        for (int start = 0; start < number_of_nodes; start++) {
            if (color[start] == -1) { // If this node is not colored, start BFS
                queue<int> q;
                q.push(start);
                color[start] = 0; // Start coloring with 0

                while (!q.empty()) {
                    int u = q.front();
                    q.pop();

                    // Traverse all adjacent vertices
                    for (auto v : adjList[u]) {
                        if (color[v[0]] == -1) { // If the vertex is not colored, color it with alternate color
                            color[v[0]] = 1 - color[u];
                            q.push(v[0]);
                        } else if (color[v[0]] == color[u]) { // If adjacent vertex has the same color, graph is not bipartite
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    }

    void kosarajuDFS(vector<vector<vector<int>>>&transpose,vector<bool>&visited,int s){
        visited[s] = 1;
        for(auto it : transpose[s]){
            if(!visited[it[0]]){
                kosarajuDFS(transpose,visited,it[0]);
            }
        }
    }


    int SCC(){
        if(!isDirected){cout << "Error : Not applicable to undirected graphs." << endl; return 0;}
        vector<int> topologicalSort = getTopologicalSort();

        // reverse the edges.
        vector<vector<vector<int>>> transpose(number_of_nodes);
        for(int i=0;i<number_of_nodes;i++){
            for(auto it : adjList[i]){
                transpose[it[0]].push_back({i,it[1]});
            }
        }
        int cnt = 0;
        vector<bool> visited(number_of_nodes);
        for(int i=0;i<number_of_nodes;i++){
            if(!visited[i]){
                kosarajuDFS(transpose,visited,i);
                cnt++;
            }
        }
        return cnt;
    }

};

#endif 