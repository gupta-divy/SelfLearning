#include <iostream>
#include <vector>
#include <unordered_map>
#include <stack>
#include <queue>
using namespace std;

class Graph {
private:
    int num_vertices;
    unordered_map<char, int> vertex_index;
    unordered_map<int, char> index_vertex;
    vector<vector<int>> adjacency_matrix;
    bool directed;
    char source_v = 'D';

public:
    Graph(const vector<char>& vertices, bool directed = false){
        num_vertices = vertices.size();
        adjacency_matrix.resize(num_vertices, vector<int>(num_vertices, 0));
        for (int i = 0; i < num_vertices; ++i) {
            vertex_index[vertices[i]] = i;
            index_vertex[i] = vertices[i];
        }
    }

    void add_edge(char vertex1, char vertex2, int cost = 1) {
        auto it1 = vertex_index.find(vertex1);
        auto it2 = vertex_index.find(vertex2);
        if (it1 == vertex_index.end() || it2 == vertex_index.end()) {
            cout << "Error: One or both vertices not found!" << endl;
            return;
        }
        int i = it1->second;
        int j = it2->second;
        adjacency_matrix[i][j] = cost;
        if(!directed) adjacency_matrix[j][i] = cost;
    }

    void print_adjacency_matrix() {
        for (const auto& row : adjacency_matrix) {
            for (int val : row) {
                cout << val << " ";
            }
            cout << endl;
        }
    }

    void dfs_util(int curr_vertex, vector<bool>& visit_tracker){
        if(visit_tracker[curr_vertex])return;
        visit_tracker[curr_vertex]=true;
        cout<<index_vertex[curr_vertex]<<' ';
        // check its neighbors
        for(int i=0; i<num_vertices; i++){
            if(adjacency_matrix[curr_vertex][i]>0) dfs_util(i,visit_tracker);
        }
    }

    void dfs_recursive(char start_vertex){
        vector<bool>visited_vertex(num_vertices,false);
        int start_idx = vertex_index[start_vertex];
        dfs_util(start_idx, visited_vertex);
        for(int i=0;i<num_vertices;i++){if(!visited_vertex[i])dfs_util(start_idx,visited_vertex);} // for disconnected graphs
    }

    void dfs_iterative(char start_vertex){
        // Assuming connected graph
        stack<int> v_stack;
        vector<bool>v_visited(num_vertices,false);
        cout<<endl;
        int start_idx = vertex_index[start_vertex];
        v_stack.push(start_idx);
        while(!v_stack.empty()){
            int vertex = v_stack.top();
            v_stack.pop();
            if(!v_visited[vertex]){
                v_visited[vertex]=true;
                cout<<index_vertex[vertex]<<' ';
                for(int i=0;i<num_vertices;i++){if(adjacency_matrix[vertex][i]>0 && !v_visited[i])v_stack.push(i);}
            }
        }
    }

    void bfs_iterative(char start_vertex){
        // Assuming connected graph
        queue<int> v_queue;
        vector<bool>v_visited(num_vertices,false);
        cout<<endl;
        int start_idx = vertex_index[start_vertex];
        v_queue.push(start_idx);
        while(!v_queue.empty()){
            int vertex = v_queue.front();
            v_queue.pop();
            if(!v_visited[vertex]){
                v_visited[vertex]=true;
                cout<<index_vertex[vertex]<<' ';
                for(int i=0;i<num_vertices;i++){if(adjacency_matrix[vertex][i]>0 && !v_visited[i])v_queue.push(i);}
            }
        }
        for(int i=0;i<num_vertices;i++)if(!v_visited[i]){cout<<index_vertex[i];}
    }

    bool cyclic_util_undirected(int v, int p, vector<bool>& visited){
        visited[v]=true;
        for(int i=0;i<num_vertices;i++){
            if(adjacency_matrix[v][i]>0 && !visited[i]){if(cyclic_util_undirected(i,v,visited)) return true;}
            else{if(visited[i] && i != p) return true;}
        }
        return false;
    }

    bool cyclic_util_directed(int v, vector<bool> &visited, vector<bool> &pathVisited){
         visited[v] = true;
         pathVisited[v] = true;
         for(int i=0; i<num_vertices; i++){
            if(adjacency_matrix[v][i]>0 && !visited[i]){if(cyclic_util_directed(i,visited,pathVisited))return true;}
            else{if(visited[i] && pathVisited[i]) return true;}
         }
         pathVisited[v] = false;
         return false;
    }

    bool is_cyclic(){
        int strt_idx = vertex_index[source_v];
        vector<bool> visited(num_vertices,false);
        vector<bool> pathVisited(num_vertices, false);
        if(!directed){
        for(int i=0;i<num_vertices;i++){
            if(!visited[i]){if(cyclic_util_undirected(i,-1,visited))return true;}
        }
        }
        else{
            for(int i=0;i<num_vertices;i++){
                if(!visited[i]){if(cyclic_util_directed(i, visited, pathVisited))return true;}
            }   
        }
        return false;
    }

    vector<int> dijkstra(char source_vertex, char end_vertex = ' '){
        vector<int> distances(num_vertices, INT_MAX);
        distances[vertex_index[source_vertex]] = 0;
        vector<bool> visited(num_vertices,false);
        vector<int> predecessors(num_vertices, -1);
        
        while(true){
            int curr_index = -1;

            // update current vertex
            int last_min = INT_MAX;
            for(int i=0;i<num_vertices;i++){
                if(!visited[i] && distances[i]<last_min){last_min=distances[i]; curr_index=i;} 
            }

            // No Unvisited index left
            if(curr_index==-1)break;

            // reached end index
            if(end_vertex != ' ' && vertex_index[end_vertex] == curr_index)break;

            // Updated distances for each neighboring vertices
            for(int i=0;i<num_vertices;i++){
                if(adjacency_matrix[curr_index][i]>0){
                    int curr_dist = distances[curr_index]+adjacency_matrix[curr_index][i];
                    if(curr_dist<distances[i]){distances[i] = curr_dist; predecessors[i]=curr_index;}
                }
            }
            
            visited[curr_index] = true;
        }

        // if end_vertex given then print the path b/w start and end
        if(end_vertex != ' '){
            vector<int> path;
            int curr_index = vertex_index[end_vertex];

            while(curr_index!=-1){
                path.push_back(curr_index);
                curr_index = predecessors[curr_index];
            }
            cout<<endl;
            for(int i=path.size(); i>=0; i--){cout<<index_vertex[path[i]]<<' ';}
            cout<<endl;
        }

        return distances;
    }


};

int main() {
    vector<char> vertices = {'A', 'B', 'C', 'D', 'E','F','G'};
    Graph g(vertices);
    g.add_edge('D', 'A', 4);
    g.add_edge('D', 'E', 2);
    g.add_edge('A', 'C', 3);
    g.add_edge('A', 'E', 4);
    g.add_edge('E', 'C', 4);
    g.add_edge('E', 'G', 5);
    g.add_edge('C', 'F', 5);
    g.add_edge('C', 'B', 2);
    g.add_edge('B', 'F', 2);
    g.add_edge('G', 'F', 5);

    cout << "Adjacency Matrix:" << endl;
    g.print_adjacency_matrix();
    g.dfs_iterative('D');
    g.bfs_iterative('D');
    cout<<endl<<g.is_cyclic();
    
    vector<int> distances = g.dijkstra('D','F');
    for(auto it : distances)cout<<it<<' ';

    return 0;
}
