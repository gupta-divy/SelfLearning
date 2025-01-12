#include <iostream>
#include <vector>
#include <unordered_map>
using namespace std;

class Graph {
private:
    int num_vertices;
    unordered_map<char, int> vertex_index; // Maps vertex labels to indices
    vector<vector<int>> adjacency_matrix;

public:
    Graph(const vector<char>& vertices) {
        num_vertices = vertices.size();
        adjacency_matrix.resize(num_vertices, vector<int>(num_vertices, 0));
        for (int i = 0; i < num_vertices; ++i) {
            vertex_index[vertices[i]] = i;
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
        adjacency_matrix[j][i] = cost;
    }

    void print_adjacency_matrix() {
        for (const auto& row : adjacency_matrix) {
            for (int val : row) {
                cout << val << " ";
            }
            cout << endl;
        }
    }
};

int main() {
    vector<char> vertices = {'A', 'B', 'C', 'D'};
    Graph g(vertices);

    g.add_edge('A', 'B', 5);
    g.add_edge('A', 'C', 3);
    g.add_edge('C', 'D', 2);

    cout << "Adjacency Matrix:" << endl;
    g.print_adjacency_matrix();

    return 0;
}
