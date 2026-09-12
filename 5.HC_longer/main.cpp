  #include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
 
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
 
double pathLength(const std::vector<Point>& points, const std::vector<int>& path) {
    double res_dist = 0;
    size_t n = path.size();
    for (size_t i = 0; i < n; ++i) {
        res_dist += dist(points[path[i]], points[(i + 1) < n ? path[i + 1] : path[0]]);
    }
    return res_dist;
}
 
void hcStep(const std::vector<Point>& points, std::vector<int>& path, double& length,
            std::mt19937& rng, double T) {
    size_t n = path.size();
    std::uniform_int_distribution<int> distIdx(0, static_cast<int>(n) - 1);
 
    int i = distIdx(rng);
    int j = distIdx(rng);
 
    if (i > j) {
      std::swap(i, j);
    }
    if (j - i < 2 || (i == 0 && j == static_cast<int>(n) - 1)) return;
 
    int a = path[i];
    int b = path[i + 1];
    int c = path[j];
    int d = (j + 1 < static_cast<int>(n)) ? path[j + 1] : path[0];
 
    double cur_dist = dist(points[a], points[b]) + dist(points[c], points[d]);
    double new_dist = dist(points[a], points[c]) + dist(points[b], points[d]);
    double delta = new_dist - cur_dist;
    double p = 1.0 / (1.0 + std::exp(delta / T));
 
    std::uniform_real_distribution<double> distProb(0.0, 1.0);
    if (distProb(rng) < p) {
        std::reverse(path.begin() + i + 1, path.begin() + j + 1);
        length += delta;
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
    std::string filename = "../data/" + std::string(argv[1]);
    std::vector<Point> points = readFile(filename);
 
    const int POP_SIZE = 10;
    const double T_START = 10.0;
    const double T_END = 0.01;
    const double TIME_LIMIT_SECONDS = 30.0 * 60.0; // увеличили лимит
    std::mt19937 rng(42);
 
    std::vector<int> basePath = greedy(points);
    double baseLength = pathLength(points, basePath);
    std::vector<std::vector<int>> population(POP_SIZE, basePath);
    std::vector<double> lengths(POP_SIZE, baseLength);
 
    //храним лучшее найденное решение
    std::vector<std::vector<int>> bestPopulation = population;
    std::vector<double> bestLengths = lengths;
 
    auto startTime = std::chrono::steady_clock::now();
    for (auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
          elapsed <= TIME_LIMIT_SECONDS;) {
 
        double progress = elapsed / TIME_LIMIT_SECONDS;
        double T = T_START - (T_START - T_END) * progress;
 
        for (int i = 0; i < POP_SIZE; ++i) {
            hcStep(points, population[i], lengths[i], rng, T);
            if (lengths[i] < bestLengths[i]) {
                bestLengths[i] = lengths[i];
                bestPopulation[i] = population[i];
            }
        }
        elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
    }
 
    int bestIdx = 0;
    for (int i = 1; i < POP_SIZE; ++i) {
        if (bestLengths[i] < bestLengths[bestIdx]) {
          bestIdx = i;
        }
    }
 
    for (size_t i = 0; i < bestPopulation[bestIdx].size(); ++i) {
        std::cout << bestPopulation[bestIdx][i] << ' ';
    }
    std::cout << '\n';
}
 