#include "falconguide/estimation/estimator_pipeline.hpp"
#include "falconguide/logger/logger.hpp"

namespace falconguide::estimation {

EstimatorPipeline::EstimatorPipeline(
    std::unique_ptr<INavigationEstimator> estimator,
    std::size_t queue_capacity)
    : estimator_(std::move(estimator)),
      queue_(queue_capacity) {}

EstimatorPipeline::~EstimatorPipeline() {
  Stop();
}

void EstimatorPipeline::RegisterObserver(INavigationObserver* observer) {
  observers_.push_back(observer);
}

bool EstimatorPipeline::Push(SensorMeasurement measurement) {
  if (!running_.load(std::memory_order_relaxed)) return false;
  return queue_.Push(std::move(measurement));
}

void EstimatorPipeline::Start() {
  if (running_.exchange(true)) return;
  thread_ = std::thread(&EstimatorPipeline::RunLoop, this);
  FG_INFO("EstimatorPipeline | started, backend={}", estimator_->Info().name);
}

void EstimatorPipeline::Stop() {
  if (!running_.exchange(false)) return;
  queue_.RequestShutdown();
  if (thread_.joinable()) thread_.join();
  FG_INFO("EstimatorPipeline | stopped");
}

bool EstimatorPipeline::IsRunning() const {
  return running_.load(std::memory_order_relaxed);
}

void EstimatorPipeline::Reset() {
  estimator_->Reset();
  NotifyReset();
}

const INavigationEstimator& EstimatorPipeline::Estimator() const {
  return *estimator_;
}

// ── Worker thread ─────────────────────────────────────────────────────────────

void EstimatorPipeline::RunLoop() {
  while (true) {
    auto measurement_opt = queue_.Pop();
    if (!measurement_opt) break;

    const auto report = estimator_->AddMeasurement(*measurement_opt);

    if (report.result == EstimatorUpdateResult::Accepted) {
      auto state = estimator_->LatestState();
      if (state) {
        NotifyObservers(
            std::make_shared<const core::NavigationState>(std::move(*state)));
      }
    }
  }
}

void EstimatorPipeline::NotifyObservers(
    std::shared_ptr<const core::NavigationState> state) {
  for (auto* observer : observers_) {
    observer->OnNavigationState(state);
  }
}

void EstimatorPipeline::NotifyReset() {
  for (auto* observer : observers_) {
    observer->OnEstimatorReset();
  }
}

}  // namespace falconguide::estimation
