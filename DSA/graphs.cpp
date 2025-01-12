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
    }

    void dfs_iterative(char start_vertex){
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
        for(int i=0;i<num_vertices;i++)if(!v_visited[i]){cout<<index_vertex[i];}
    }

    void bfs_iterative(char start_vertex){
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
};

int main() {
    vector<char> vertices = {'A', 'B', 'C', 'D', 'E','F','G', 'H'};
    Graph g(vertices);
    g.add_edge('D', 'A');
    g.add_edge('A', 'C'); 
    g.add_edge('A', 'D'); 
    g.add_edge('A', 'E'); 
    g.add_edge('E', 'C'); 
    g.add_edge('C', 'F'); 
    g.add_edge('C', 'B'); 
    g.add_edge('C', 'G'); 
    g.add_edge('B', 'F');
    cout << "Adjacency Matrix:" << endl;
    g.print_adjacency_matrix();
    g.dfs_iterative('D');
    g.bfs_iterative('D');
    return 0;
}
