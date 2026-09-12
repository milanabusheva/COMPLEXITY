#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <filesystem>

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
    for (auto i : greedy(points)) {
        std::cout << i << ' ';
    }
    std::cout << '\n';
}