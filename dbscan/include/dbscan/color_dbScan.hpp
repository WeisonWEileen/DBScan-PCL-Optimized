
// designed for dbscan clustering 10 dim hsv space color. we use std::vector as outside container
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <vector>

namespace color {

const int UNDEFINED = -1;
const int NOISE = 0;

template <typename T>
class DBSCAN {
 public:
  DBSCAN(double eps, int minPts, std::function<double(const T&, const T&)> distFunc);
  std::vector<int> cluster(const std::vector<T>& data);

 private:
  double eps;
  int minPts;
  std::function<double(const T&, const T&)> distFunc;
  std::vector<int> rangeQuery(const std::vector<T>& data, int i);
};

template <typename T>
DBSCAN<T>::DBSCAN(double eps, int minPts, std::function<double(const T&, const T&)> distFunc) : eps(eps), minPts(minPts), distFunc(distFunc) {}

template <typename T>
std::vector<int> DBSCAN<T>::cluster(const std::vector<T>& data) {
  int clusterId = 0;

  std::vector<int> labels(data.size(), UNDEFINED);

  for (size_t i = 0; i < data.size(); ++i) {
    if (labels[i] != UNDEFINED) {
      continue;
    }  // Skip if already processed

    std::vector<int> neighbors_idx = rangeQuery(data, i);
    if (neighbors_idx.size() < minPts) {
      labels[i] = NOISE;  // Mark as noise
      continue;
    }

    clusterId++;            // Start a new cluster
    labels[i] = clusterId;  // Label initial point

    // search neighbors
    while (!neighbors_idx.empty()) {
      int q_idx = neighbors_idx.back();
      neighbors_idx.pop_back();

      if (labels[q_idx] == NOISE) {
        labels[q_idx] = clusterId;  // Change noise to border point
      }

      if (labels[q_idx] != UNDEFINED) {
        continue;
      }  // Already processed

      labels[q_idx] = clusterId;  // Label neighbor
      std::vector<int> qNeighbors_idx = rangeQuery(data, q_idx);
      if (qNeighbors_idx.size() >= minPts) {
        std::copy(qNeighbors_idx.begin(), qNeighbors_idx.end(), std::back_inserter(neighbors_idx));
      }
    }
  }
  return labels;
}

template <typename T>
std::vector<int> DBSCAN<T>::rangeQuery(const std::vector<T>& data, int idx) {
  std::vector<int> neighbors_indexes;
  auto point = data[idx];
  for (int i = 0; i < data.size(); i++) {
    if (i == idx) {
      continue;
    }
    auto dis = distFunc(point, data[i]);
    // std::cout << dis << std::endl;
    // if (dis == 0) {
    //   std::cout << idx << " " << i << std::endl;
    // //   std::cout << data[idx] << std::endl;
    // }
    if (dis <= eps) {
      neighbors_indexes.push_back(i);
    }
  }
  return neighbors_indexes;
}
}  // namespace color