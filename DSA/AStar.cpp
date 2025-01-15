#include <iostream>
#include <vector>
#include <climits>
#include <cmath>
#include <algorithm>


using namespace std;

class Gridworld2D {
private:
    vector<vector<int>> grid_mat;

public:
    int rows;
    int cols;
    Gridworld2D() : rows(0), cols(0) {}

    Gridworld2D(int rows, int cols) : rows(rows), cols(cols) {
        grid_mat.resize(rows, vector<int>(cols, 1));
    }

    void block_coords(vector<vector<int>> coords) {
        for (const auto& coord : coords) {
            if (coord[0] < 0 || coord[0] >= rows || coord[1] < 0 || coord[1] >= cols) {
                cout << "Skipping out of range coordinate: " << coord[0] << ", " << coord[1] << endl;
                continue;
            }
            grid_mat[coord[0]][coord[1]] = 0;
        }
    }

    vector<vector<int>> get_neighbors_with_MovingCost(vector<int> curr_coord) {
        int x = curr_coord[0];
        int y = curr_coord[1];
        vector<vector<int>> neighbors;
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0) continue;  // Same Cell
                if (x + i < 0 || x + i >= rows || y + j < 0 || y + j >= cols) continue;  // Out of bounds
                if (grid_mat[x + i][y + j] == 0) continue;  // Blocked

                vector<int> coord(3);
                coord[0] = x + i;
                coord[1] = y + j;
                coord[2] = (abs(i) + abs(j) == 2) ? 3 : 2;  // Diagonal (3), else (2)
                neighbors.push_back(coord);
            }
        }
        return neighbors;
    }
};

class AStar {
private:
    Gridworld2D gridworld;
    vector<vector<double>> cost_map;
    vector<vector<bool>> visited;
    vector<vector<vector<int>>> predecessor;
    vector<int> start_idx, end_idx;

public:
    AStar(Gridworld2D gridworld2d, vector<int> start, vector<int> end) {
        gridworld = gridworld2d;
        start_idx = start;
        end_idx = end;
        cost_map.resize(gridworld.rows, vector<double>(gridworld.cols, INT_MAX));
        visited.resize(gridworld.rows, vector<bool>(gridworld.cols, false));
        predecessor.resize(gridworld.rows, vector<vector<int>>(gridworld.cols));
        cost_map[start_idx[0]][start_idx[1]] = 0;
    }

    double h_function(int x, int y) {
        return sqrt(pow(x - end_idx[0], 2) + pow(y - end_idx[1], 2));  // Euclidean Distance
    }

    vector<int> get_lowest_val_idx() {
        vector<int> coord(2);
        double minValue = INT_MAX;
        for (int i = 0; i < gridworld.rows; ++i) {
            for (int j = 0; j < gridworld.cols; ++j) {
                if (cost_map[i][j] < minValue && !visited[i][j]) {
                    minValue = cost_map[i][j];
                    coord[0] = i;
                    coord[1] = j;
                }
            }
        }
        return coord;
    }

    void run_astar() {
        vector<int> curr_coord(2);
        vector<vector<int>> neighbors;
        while (true) {
            curr_coord = get_lowest_val_idx();

            if (curr_coord[0] == end_idx[0] && curr_coord[1]==end_idx[1]) break;

            neighbors = gridworld.get_neighbors_with_MovingCost(curr_coord);

            for (auto neighbor : neighbors) {
                double local_cost = neighbor[2] + h_function(neighbor[0], neighbor[1]);
                if (local_cost < cost_map[neighbor[0]][neighbor[1]]) {
                    cost_map[neighbor[0]][neighbor[1]] = local_cost;
                    predecessor[neighbor[0]][neighbor[1]] = curr_coord;
                }
            }

            visited[curr_coord[0]][curr_coord[1]] = true;
        }
    }

    void get_path() {
        run_astar();

        vector<vector<int>> path;
        vector<int> current = end_idx;

        while (current != start_idx) {
            path.push_back(current);
            current = predecessor[current[0]][current[1]];
        }

        path.push_back(start_idx);
        reverse(path.begin(), path.end());

        cout << "Path: ";
        for (const auto& step : path) {
            cout << "(" << step[0] << ", " << step[1] << ") ";
        }
        cout << endl;
    }

};

int main() {
    Gridworld2D gridworld(10, 10);

    vector<int> start = {0, 0};  // Starting position
    vector<int> end = {9, 9};    // Ending position

    vector<vector<int>> obstacles = {
        {1, 1}, {1, 2}, {1, 3}, {2, 3}, {3, 3}, {4, 3}, {5, 3}, {5, 4},
        {5, 5}, {5, 6}, {5, 7}, {6, 7}, {7, 7}, {7, 8}, {8, 8}, {8, 9}
    };

    gridworld.block_coords(obstacles);

    AStar astar(gridworld, start, end);
    astar.get_path();

    return 0;
}