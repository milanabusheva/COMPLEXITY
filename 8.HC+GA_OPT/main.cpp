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

const int POP_SIZE = 10;
const double MUT_PROB = 0.02;
const double MUT_B = 5.0;
const int CNT_INTO_STEP = 2000;
const double LOCAL_SEARCH_T = 1.0;
const double TL_SEC = 30.0 * 60.0;
const int CHECK_NEIGHBORS = 6;

std::mt19937 rng(42);
std::uniform_real_distribution<double> distProb(0.0, 1.0);
std::uniform_int_distribution<int> distIdx(0, POP_SIZE - 1);
std::uniform_int_distribution<int> distPathIdx;
std::uniform_int_distribution<int> distNeighborIdx(0, CHECK_NEIGHBORS - 1);

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

// для каждого города - его CHECK_NEIGHBORS соседей предподсчет
std::vector<std::vector<int>> buildNeighbors(const std::vector<Point>& points) {
    int n = points.size();
    int k = std::min(CHECK_NEIGHBORS, n - 1);
    std::vector<std::vector<int>> neighbors(n);
    if (n <= 1) return neighbors;
    
    std::vector<std::pair<double, int>> distances(n - 1);
    for (int i = 0; i < n; ++i) {
        int idx = 0;
        for (int j = 0; j < n; ++j) {
            if (j != i) {
                distances[idx++] = {dist(points[i], points[j]), j};
             }
        }
        
        // сортируем только k ближайших
        std::nth_element(distances.begin(), distances.begin() + k, distances.end());
        std::sort(distances.begin(), distances.begin() + k);
        
        neighbors[i].reserve(k);
        for (int t = 0; t < k; ++t) {
            neighbors[i].push_back(distances[t].second);
        }
    }
    
    return neighbors;
}

// \в hcStep проверяем только с близкими точками, поэтому более вероятен успех
void hcStep(const std::vector<Point>& points, const std::vector<std::vector<int>>& neighbors,
            std::vector<int>& path, std::vector<int>& posInPath, double& len) {
    size_t n = path.size();

    int i = distPathIdx(rng);
    int cityA = path[i];
    const std::vector<int>& candidates = neighbors[cityA];
    int cityC = candidates[distNeighborIdx(rng)];
    int j = posInPath[cityC];

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
    double p = 1.0 / (1.0 + std::exp(delta / LOCAL_SEARCH_T));

    if (distProb(rng) < p) {
        std::reverse(path.begin() + i + 1, path.begin() + j + 1);
        for (int k = i + 1; k <= j; ++k) {
            posInPath[path[k]] = k;
        }
        len += delta;
    }
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

std::vector<int> decode(const std::vector<double>& keys) {
    size_t n = keys.size();
    std::vector<int> order(n);
    for (int i = 0; i < n; ++i) {
      order[i] = i;
    }

    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return keys[a] < keys[b];
    });
    return order;
}

std::vector<double> encode(const std::vector<int>& path) {
    size_t n = path.size();
    std::vector<double> keys(n);
    for (size_t i = 0; i < n; ++i) {
        keys[path[i]] = static_cast<double>(i) / n;
    }
    return keys;
}

int tournamentSelect(const std::vector<double>& fitness) {
    int a = distIdx(rng);
    int b = distIdx(rng);
    return (fitness[a] < fitness[b]) ? a : b;
}

std::vector<double> crossover(const std::vector<double>& p1, const std::vector<double>& p2) {
    size_t n = p1.size();
    double w = distProb(rng);

    std::vector<double> child(n);
    for (size_t i = 0; i < n; ++i) {
        child[i] = w * p1[i] + (1.0 - w) * p2[i];
    }
    return child;
}

void mutate(std::vector<double>& keys, double progress) {
    for (double& x : keys) {
        if (distProb(rng) < MUT_PROB) {
            double r = distProb(rng);
            double f = 1.0 - std::pow(r, std::pow(1.0 - progress, MUT_B));
            if (distProb(rng) < 0.5) {
                x = x + (1.0 - x) * f;
            } else {
                x = x - (x - 0.0) * f;
            }
        }
    }
}

// GA просто берем из статьи
std::vector<int> runGA(const std::vector<Point>& points) {
    int n = static_cast<int>(points.size());
    distPathIdx = std::uniform_int_distribution<int>(0, n - 1);
    std::vector<std::vector<int>> neighbors = buildNeighbors(points);
    std::vector<std::vector<double>> population(POP_SIZE, std::vector<double>(n));
    for (int i = 0; i < POP_SIZE; ++i) {
        for (int j = 0; j < n; ++j) {
            population[i][j] = distProb(rng);
        }
    }

    std::vector<int> greedypath = greedy(points);
    population[0] = encode(greedypath); // 1 популяция стартует с жадника: не все как раньше, т.к. для GA надо разнообразие
    std::vector<double> fitness(POP_SIZE);
    for (int i = 0; i < POP_SIZE; ++i) {
        fitness[i] = pathLength(points, decode(population[i]));
    }
    int bestIdx = static_cast<int>(std::min_element(fitness.begin(), fitness.end()) - fitness.begin());
    std::vector<double> bestKeys = population[bestIdx];
    double bestFitness = fitness[bestIdx];
    auto start = std::chrono::steady_clock::now();
    for (auto curTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
          curTime <= TL_SEC;) {

        double progress = curTime / TL_SEC;

        std::vector<std::vector<double>> newPopulation(POP_SIZE);
        std::vector<double> newFitness(POP_SIZE);

        newPopulation[0] = bestKeys;
        newFitness[0] = bestFitness;

        for (int i = 1; i < POP_SIZE; ++i) {
            int p1 = tournamentSelect(fitness);
            int p2 = tournamentSelect(fitness);
            std::vector<double> child = crossover(population[p1], population[p2]);
            mutate(child, progress);

            std::vector<int> childPath = decode(child);
            double  lenChild = pathLength(points, childPath);
            std::vector<int> posInPath(n);
            for (int k = 0; k < n; ++k) {
                posInPath[childPath[k]] = k;
            }
            for (int j = 0; j < CNT_INTO_STEP; ++j) {
                hcStep(points, neighbors, childPath, posInPath,  lenChild);
            }
            child = encode(childPath);
            newPopulation[i] = child;
            newFitness[i] =  lenChild;
            if ( lenChild < bestFitness) {
                bestFitness =  lenChild;
                bestKeys = child;
            }
        }
        std::swap(population, newPopulation);
        std::swap(fitness, newFitness);

        curTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    }

    std::vector<int> bestPath = decode(bestKeys);
    twoOpt(points, bestPath); // когда уже выбрали путь - применяем 2-opt пока можем
    return bestPath;
}

int main(int argc, char* argv[]) {
    std::string filename = "../data/" + std::string(argv[1]);
    std::vector<Point> points = readFile(filename);

    std::vector<int> bestPath = runGA(points);
    for (size_t i = 0; i < bestPath.size(); ++i) {
        std::cout << bestPath[i] << ' ';
    }
    std::cout << '\n';
}