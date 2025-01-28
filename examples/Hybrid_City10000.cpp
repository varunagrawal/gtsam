/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Hybrid_City10000.cpp
 * @brief  Example of using hybrid estimation
 *         with multiple odometry measurements.
 * @author Varun Agrawal
 * @date   January 22, 2025
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/HybridSmoother.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <time.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;
using namespace boost::algorithm;

using symbol_shorthand::L;
using symbol_shorthand::M;
using symbol_shorthand::X;

// Testing params
const size_t max_loop_count = 3000;  // 2000;  // 200 //2000 //8000

noiseModel::Diagonal::shared_ptr prior_noise_model =
    noiseModel::Diagonal::Sigmas(
        (Vector(3) << 0.0001, 0.0001, 0.0001).finished());

noiseModel::Diagonal::shared_ptr pose_noise_model =
    noiseModel::Diagonal::Sigmas(
        (Vector(3) << 1.0 / 30.0, 1.0 / 30.0, 1.0 / 100.0).finished());

/**
 * @brief Write the result of optimization to filename.
 *
 * @param result The Values object with the final result.
 * @param num_poses The number of poses to write to the file.
 * @param filename The file name to save the result to.
 */
void write_result(const Values& result, size_t num_poses,
                  const std::string& filename = "Hybrid_city10000.txt") {
  ofstream outfile;
  outfile.open(filename);

  for (size_t i = 0; i < num_poses; ++i) {
    Pose2 out_pose = result.at<Pose2>(X(i));

    outfile << out_pose.x() << " " << out_pose.y() << " " << out_pose.theta()
            << std::endl;
  }
  outfile.close();
  std::cout << "output written to " << filename << std::endl;
}

/**
 * @brief Create a hybrid loop closure factor where
 * 0 - loose noise model and 1 - loop noise model.
 *
 * @param loop_counter
 * @param key_s
 * @param key_t
 * @param measurement
 * @return HybridNonlinearFactor
 */
HybridNonlinearFactor HybridLoopClosureFactor(size_t loop_counter, size_t key_s,
                                              size_t key_t,
                                              const Pose2& measurement) {
  DiscreteKey l(L(loop_counter), 2);

  auto f0 = std::make_shared<BetweenFactor<Pose2>>(
      X(key_s), X(key_t), measurement,
      noiseModel::Diagonal::Sigmas(Vector3::Ones() * 100));
  auto f1 = std::make_shared<BetweenFactor<Pose2>>(
      X(key_s), X(key_t), measurement, pose_noise_model);
  std::vector<NonlinearFactorValuePair> factors{{f0, 0.0}, {f1, 0.0}};
  HybridNonlinearFactor mixtureFactor(l, {f0, f1});
  return mixtureFactor;
}

HybridNonlinearFactor HybridOdometryFactor(
    size_t num_measurements, size_t key_s, size_t key_t, const DiscreteKey& m,
    const std::vector<Pose2>& pose_array,
    const SharedNoiseModel& pose_noise_model) {
  auto f0 = std::make_shared<BetweenFactor<Pose2>>(
      X(key_s), X(key_t), pose_array[0], pose_noise_model);
  auto f1 = std::make_shared<BetweenFactor<Pose2>>(
      X(key_s), X(key_t), pose_array[1], pose_noise_model);
  std::vector<NonlinearFactorValuePair> factors{{f0, 0.0}, {f1, 0.0}};
  HybridNonlinearFactor mixtureFactor(m, factors);
  // HybridNonlinearFactor mixtureFactor(m, {f0, f1});
  return mixtureFactor;
}

void SmootherUpdate(HybridSmoother& smoother, HybridNonlinearFactorGraph& graph,
                    const Values& initial, size_t maxNrHypotheses,
                    Values* result) {
  HybridGaussianFactorGraph linearized = *graph.linearize(initial);
  // std::cout << "index: " << index << std::endl;
  smoother.update(linearized, maxNrHypotheses);
  graph.resize(0);
  HybridValues delta = smoother.hybridBayesNet().optimize();
  result->insert_or_assign(initial.retract(delta.continuous()));
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  ifstream in(findExampleDataFile("T1_city10000_04.txt"));
  // ifstream in("../data/mh_T1_city10000_04.txt"); //Type #1 only
  // ifstream in("../data/mh_T3b_city10000_10.txt"); //Type #3 only
  // ifstream in("../data/mh_T1_T3_city10000_04.txt"); //Type #1 + Type #3

  // ifstream in("../data/mh_All_city10000_groundtruth.txt");

  size_t discrete_count = 0, index = 0;
  size_t loop_count = 0;
  size_t nrHybridFactors = 0;

  std::list<double> time_list;

  HybridSmoother smoother(0.99);

  HybridNonlinearFactorGraph graph;

  Values initial, result;

  size_t maxNrHypotheses = 3;

  double x = 0.0;
  double y = 0.0;
  double rad = 0.0;

  Pose2 prior_pose(x, y, rad);

  initial.insert(X(0), prior_pose);

  graph.push_back(PriorFactor<Pose2>(X(0), prior_pose, prior_noise_model));

  std::vector<std::pair<size_t, double>> smoother_update_times;

  clock_t before_update = clock();
  SmootherUpdate(smoother, graph, initial, maxNrHypotheses, &result);
  clock_t after_update = clock();
  smoother_update_times.push_back({index, after_update - before_update});

  size_t key_s, key_t{0};

  clock_t start_time = clock();
  std::string str;
  while (getline(in, str) && index < max_loop_count) {
    vector<string> parts;
    split(parts, str, is_any_of(" "));

    key_s = stoi(parts[1]);
    key_t = stoi(parts[3]);

    int num_measurements = stoi(parts[5]);
    vector<Pose2> pose_array(num_measurements);
    for (int i = 0; i < num_measurements; ++i) {
      x = stod(parts[6 + 3 * i]);
      y = stod(parts[7 + 3 * i]);
      rad = stod(parts[8 + 3 * i]);
      pose_array[i] = Pose2(x, y, rad);
    }

    // Flag if we should run smoother update
    bool smoother_update = false;

    // Take the first one as the initial estimate
    Pose2 odom_pose = pose_array[0];
    if (key_s == key_t - 1) {  // odometry

      if (num_measurements > 1) {
        DiscreteKey m(M(discrete_count), num_measurements);

        // Add hybrid factor which considers both measurements
        HybridNonlinearFactor mixtureFactor = HybridOdometryFactor(
            num_measurements, key_s, key_t, m, pose_array, pose_noise_model);
        graph.push_back(mixtureFactor);

        discrete_count++;

        smoother_update = true;

      } else {
        graph.add(BetweenFactor<Pose2>(X(key_s), X(key_t), odom_pose,
                                       pose_noise_model));
      }

      initial.insert(X(key_t), initial.at<Pose2>(X(key_s)) * odom_pose);

    } else {  // loop
      HybridNonlinearFactor loop_factor =
          HybridLoopClosureFactor(loop_count, key_s, key_t, odom_pose);
      graph.add(loop_factor);

      smoother_update = true;

      loop_count++;
    }

    if (smoother_update) {
      gttic_(SmootherUpdate);
      before_update = clock();
      SmootherUpdate(smoother, graph, initial, maxNrHypotheses, &result);
      after_update = clock();
      smoother_update_times.push_back({index, after_update - before_update});
      gttoc_(SmootherUpdate);
    }

    // Print loop index and time taken in processor clock ticks
    // if (index % 50 == 0 && key_s != key_t - 1) {
    if (index % 100 == 0) {
      std::cout << "index: " << index << std::endl;
      std::cout << "acc_time:  " << time_list.back() / CLOCKS_PER_SEC
                << std::endl;
      // delta.discrete().print("The Discrete Assignment");
      tictoc_finishedIteration_();
      tictoc_print_();
    }

    if (key_s == key_t - 1) {
      clock_t cur_time = clock();
      time_list.push_back(cur_time - start_time);
    }

    index += 1;
  }

  before_update = clock();
  SmootherUpdate(smoother, graph, initial, maxNrHypotheses, &result);
  after_update = clock();
  smoother_update_times.push_back({index, after_update - before_update});

  gttic_(HybridSmootherOptimize);
  HybridValues delta = smoother.hybridBayesNet().optimize();
  gttoc_(HybridSmootherOptimize);
  result.insert_or_assign(initial.retract(delta.continuous()));

  std::cout << "Final error: " << smoother.hybridBayesNet().error(delta)
            << std::endl;
  clock_t end_time = clock();
  clock_t total_time = end_time - start_time;
  cout << "total_time: " << total_time / CLOCKS_PER_SEC << " seconds" << endl;

  /// Write result to file
  write_result(result, (key_t + 1), "Hybrid_City10000.txt");

  //TODO Write to file
  // for (size_t i = 0; i < smoother_update_times.size(); i++) {
  //   auto p = smoother_update_times.at(i);
  //   std::cout << p.first << ", " << p.second / CLOCKS_PER_SEC << std::endl;
  // }
  ofstream outfile_time;
  std::string time_file_name = "Hybrid_City10000_time.txt";
  outfile_time.open(time_file_name);
  for (auto acc_time : time_list) {
    outfile_time << acc_time << std::endl;
  }
  outfile_time.close();
  cout << "output " << time_file_name << " file." << endl;
  std::cout << nrHybridFactors << std::endl;

  return 0;
}
