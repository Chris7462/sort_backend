#include "sort_backend/sort_backend.hpp"
#include "internal/sort_impl.hpp"


namespace sort
{

Sort::Sort(int max_age, int min_hits, float iou_threshold)
: impl_(std::make_unique<Impl>(max_age, min_hits, iou_threshold))
{
}

Sort::~Sort() = default;
Sort::Sort(Sort &&) noexcept = default;
Sort & Sort::operator=(Sort &&) noexcept = default;

MatrixXf Sort::update(const MatrixXf & detections)
{
  return impl_->update(detections);
}

int Sort::getFrameCount() const
{
  return impl_->getFrameCount();
}

size_t Sort::getTrackerCount() const
{
  return impl_->getTrackerCount();
}

void Sort::reset()
{
  impl_->reset();
}

} // namespace sort
