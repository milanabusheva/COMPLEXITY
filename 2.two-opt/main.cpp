#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <filesystem>
#include <algorithm>

struct Point {
    double x, y;
};

double dist(const Point& a, const Point& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

std::vector<int> greedy(const std::vector<Point>& points) {
    size_t n = points.size();
    std::vector<bool> used(n, false);
    std::vector<int> path;
    path.reserve(n);
    int cur = 0;
    used[0] = true;
    path.emplace_back(0);

    for (int i = 1; i < n; ++i) {
        int best = -1;
        double bestDist = 1e18;

        for (int j = 0; j < n; ++j) {
            if (used[j]) continue;
            double d = dist(points[cur], points[j]);
            if (d < bestDist) {
                bestDist = d;
                best = j;
            }
        }

        used[best] = true;
        path.emplace_back(best);
        cur = best;
    }

    return path;
}

void twoOpt(const std::vector<Point>& points, std::vector<int>& path) {
    size_t n = path.size();
 
    for (bool improved = true; improved; ) {
        improved = false;
        for (int i = 0; i < n - 1; ++i) {
            int a = path[i];
            int b = path[i + 1];
            for (int j = i + 2; j < n; ++j) {
                if (i == 0 && j == n - 1) continue;
                int c = path[j];
                int d = (j + 1 < n) ? path[j + 1] : path[0];
 
                double cur_dist = dist(points[a], points[b]) + dist(points[c], points[d]);
                double new_dist = dist(points[a], points[c]) + dist(points[b], points[d]);
 
                if (new_dist < cur_dist) {
                    std::reverse(path.begin() + i + 1, path.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
}

std::vector<Point> readFile(const std::string& filename) {
    std::ifstream in(filename);
    int n;
    in >> n;
    std::vector<Point> points(n);
    for (int i = 0; i < n; ++i) {
        in >> points[i].x >> points[i].y;
    }

    return points;
}

int main(int argc, char* argv[]) {
    std::string filename = "../data/" +  std::string(argv[1]);

    std::vector<Point> points = readFile(filename);
    std::vector<int> path = greedy(points);
    twoOpt(points, path);
 
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i] << ' ';
    }
    std::cout << '\n';
}
