#pragma once

#include <memory>
#include <vector>

#include "sort_backend/sort_backend.hpp"
#include "internal/kalman_box_tracker.hpp"

// NOTE: This is a private implementation header. It lives under src/internal/
// and is never installed - it must not be included by any consumer of the
// sort_backend public API. It completes the Sort::Impl type that the public
// header only forward-declares.

namespace sort
{

// Sort::Impl - holds everything that needs KalmanBoxTracker / the Hungarian
// algorithm. Public surface mirrors Sort exactly: update(), getFrameCount(),
// getTrackerCount(), reset(). Everything else - invalid-tracker pruning,
// output-track construction - is an internal implementation detail and
// stays private.
class Sort::Impl
{
public:
  Impl(int max_age, int min_hits, float iou_threshold);

  MatrixXf update(const MatrixXf & detections);
  int getFrameCount() const;
  size_t getTrackerCount() const;
  void reset();

private:
  // SORT parameters
  int max_age_;           // Maximum frames without detection before deletion
  int min_hits_;          // Minimum hits before tracker is confirmed
  float iou_threshold_;   // IoU threshold for association

  // Tracker state
  std::vector<std::unique_ptr<KalmanBoxTracker>> trackers_;
  int frame_count_;       // Current frame number

  /**
   * @brief Remove trackers that have invalid predictions (NaN values)
   * @param predicted_tracks Matrix of predicted tracker states
   * @return Indices of trackers to remove
   */
  std::vector<int> findInvalidTrackers(const MatrixXf & predicted_tracks);

  /**
   * @brief Build output matrix from confirmed trackers
   * @return Matrix where each row is [x1, y1, x2, y2, track_id]
   */
  MatrixXf buildOutputTracks();
};

} // namespace sort
