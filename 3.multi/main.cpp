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

std::vector<int> greedy(const std::vector<Point>& points, int start) {
    size_t n = points.size();
    std::vector<bool> used(n, false);
    std::vector<int> path;
    path.reserve(n);
    int cur = start;
    used[start] = true;
    path.emplace_back(start);
 
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

double pathLength(const std::vector<Point>& points, const std::vector<int>& path) {
    double res_dist = 0;
    size_t n = path.size();
    for (size_t i = 0; i < n; ++i) {
        int a = path[i];
        res_dist += dist(points[path[i]], points[(i + 1) < n ? path[i + 1] : path[0]]);
    }
    return res_dist;
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
    int cntStarts = std::min((size_t)5, points.size());
    std::vector<int> bestPath;
    double bestLen = 1e18;
 
    for (int i = 0; i < cntStarts; ++i) {
        int start = i * (points.size() / cntStarts);
        std::vector<int> path = greedy(points, start);
        double len = pathLength(points, path);
        if (len < bestLen) {
            bestLen = len;
            bestPath = path;
        }
    }
 
    twoOpt(points, bestPath);
    bestLen = pathLength(points, bestPath);
 
    for (size_t i = 0; i < bestPath.size(); ++i) {
        std::cout << bestPath[i] << ' ';
    }
    std::cout << "\n Best Len: " << bestLen << '\n';
}