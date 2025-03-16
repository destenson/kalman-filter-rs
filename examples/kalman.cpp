#include <chrono>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"

int main() {
  cv::KalmanFilter kf{2, 1, 0};
  kf.transitionMatrix = (cv::Mat_<float>(2, 2) << 1, 1, 0, 1);
  kf.measurementMatrix = (cv::Mat_<float>(1, 2) << 1, 0);
  kf.processNoiseCov = (cv::Mat_<float>(2, 2) << 1e-5, 0, 0, 1e-5);
  kf.measurementNoiseCov = (cv::Mat_<float>(1, 1) << 1e-1);
  kf.errorCovPost = (cv::Mat_<float>(2, 2) << 1, 0, 0, 1);

  kf.statePost = (cv::Mat_<float>(2, 1) << 10, 0);

  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 10000; ++i) {
    kf.predict();
    kf.correct((cv::Mat_<float>(1, 1) << i + 0.1));
  }
  auto t1 = std::chrono::steady_clock::now();
  std::cout
      << std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()
      << std::endl;

  return 0;
}