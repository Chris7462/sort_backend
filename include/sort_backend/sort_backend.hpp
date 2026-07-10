///////////////////////////////////////////////////////////////////////////////
// sort_backend.hpp: Simple Online and Realtime Tracking (SORT) algorithm
// Multi-object tracker using Kalman filters and Hungarian algorithm
// Tracks objects across frames using bounding box detections
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <Eigen/Dense>
#include <memory>

namespace sort
{

using MatrixXf = Eigen::MatrixXf;
using VectorXf = Eigen::VectorXf;

class Sort
{
public:
  /**
   * @brief Initialize SORT tracker with parameters
   * @param max_age Maximum number of frames to keep tracker without detections
   * @param min_hits Minimum hits before tracker is considered confirmed
   * @param iou_threshold Minimum IoU for detection-tracker association
   */
  explicit Sort(int max_age = 1, int min_hits = 3, float iou_threshold = 0.3f);

  /**
   * @brief Destructor (defined in .cpp: required since Impl is incomplete here)
   */
  ~Sort();

  // Non-copyable - Impl is exclusively owned; movable is cheap and safe.
  Sort(const Sort &) = delete;
  Sort & operator=(const Sort &) = delete;
  Sort(Sort &&) noexcept;
  Sort & operator=(Sort &&) noexcept;

  /**
   * @brief Update tracker with new detections
   * @param detections Detection matrix where each row is [x1, y1, x2, y2, score]
   *                   Use empty matrix (0 rows, 5 cols) for frames without detections
   * @return Tracking results where each row is [x1, y1, x2, y2, track_id]
   *         Note: Number of returned tracks may differ from input detections
   */
  MatrixXf update(const MatrixXf & detections = MatrixXf::Zero(0, 5));

  /**
   * @brief Get current frame count
   * @return Number of frames processed
   */
  int getFrameCount() const;

  /**
   * @brief Get number of active trackers
   * @return Current number of trackers
   */
  size_t getTrackerCount() const;

  /**
   * @brief Reset tracker state (clear all trackers)
   */
  void reset();

private:
  // Opaque implementation - hides KalmanBoxTracker, KalmanFilter, and the
  // Hungarian algorithm dependency from consumers of this header.
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace sort
