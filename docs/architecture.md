# FalconGuide — System Architecture

## 1. Target Hardware

FalconGuide is designed to run on production SOM (System-on-Module) computers with the following specifications:

| Parameter        | Minimum          | Expected         |
|-----------------|------------------|------------------|
| CPU cores       | 4                | 8                |
| Clock speed     | 1.9 GHz          | 2.5 GHz          |
| RAM             | 32 GB            | 64 GB            |
| OS              | Linux (real-time kernel recommended) | |
| Architecture    | ARM64 / x86-64   |                  |

With 4–8 cores available, FalconGuide is designed around a **multi-threaded pipeline** that keeps the navigation estimator on a dedicated core, sensor I/O on separate threads, and downstream consumers (control, telemetry, recording) fully decoupled.

---

## 2. Core Architecture

The system is structured in four layers:

```
┌─────────────────────────────────────────────────────────────────┐
│  Sensor Drivers  (one thread per sensor or sensor group)        │
│  IMU @ 400 Hz · GNSS @ 10 Hz · Baro · Mag · DVL · etc.        │
└───────────────────────────┬─────────────────────────────────────┘
                            │  SensorMeasurement variants
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  MeasurementQueue  (MPSC lock-free ring buffer)                 │
│  Decouples sensor threads from the estimator thread             │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  EstimatorPipeline  (single dedicated thread)                   │
│  · Drains MeasurementQueue                                      │
│  · Calls INavigationEstimator::AddMeasurement()                 │
│  · After each aiding update: dispatches NavigationState         │
│    to all registered INavigationObserver instances              │
└───────────────────────────┬─────────────────────────────────────┘
                            │  NavigationState  (read-only, shared_ptr)
              ┌─────────────┼─────────────┐
              ▼             ▼             ▼
        ControlLoop    Telemetry      DataRecorder
        (10–100 Hz)    (UDP/ROS2)     (HDF5 / bag)
```

### Why a single estimator thread?

The EKF and UKF state machines are **not designed for concurrent access**. Serialising all measurement processing through one thread eliminates lock contention and makes timing deterministic. On a 4-core SOM at 1.9 GHz this thread consumes roughly 15–25 % of one core at full sensor rate (400 Hz IMU + 4 aiding sensors). On an 8-core 2.5 GHz machine the overhead drops below 10 %.

---

## 3. Observer Pattern

Downstream consumers implement `INavigationObserver`:

```cpp
class INavigationObserver {
public:
  virtual ~INavigationObserver() = default;

  // Called by EstimatorPipeline after every aiding update.
  // Implementations must be non-blocking.  Heavy work (file I/O,
  // network serialisation) must be offloaded to an internal queue.
  virtual void OnNavigationState(
      std::shared_ptr<const core::NavigationState> state) = 0;

  // Called when the estimator resets or loses initialization.
  virtual void OnEstimatorReset() {}
};
```

`EstimatorPipeline` holds a list of observers and calls `OnNavigationState()` synchronously after each update **on the estimator thread**. Observers must return immediately — any blocking work goes into their own worker thread.

### Observer dispatch rate

| Trigger                    | Typical rate         |
|---------------------------|----------------------|
| IMU-only propagation       | Not dispatched       |
| After aiding update        | 1–50 Hz (sensor-dependent) |
| Forced periodic dispatch   | Configurable (e.g. 10 Hz fallback) |

---

## 4. Memory Budget

64 GB RAM is far larger than navigation state requires. The dominant allocations are:

| Component                       | Memory              |
|--------------------------------|---------------------|
| EKF state (15-state, double)   | < 1 KB              |
| UKF sigma points (31×15)       | < 4 KB              |
| IMU ring buffer (2000 samples) | ~200 KB             |
| spdlog async queue             | 8 KB – 4 MB (configurable) |
| HDF5 / bag recorder buffer     | 32–256 MB (configurable) |

The remaining RAM is available for sensor fusion buffers, map data, or co-located processes (perception, planning).

---

## 5. Thread Map

| Thread                  | Pinned core (suggestion) | Priority       |
|------------------------|--------------------------|----------------|
| IMU driver             | Core 0                   | RT SCHED_FIFO 80 |
| GNSS / other sensors   | Core 1                   | RT SCHED_FIFO 70 |
| EstimatorPipeline      | Core 2                   | RT SCHED_FIFO 75 |
| Telemetry / recording  | Core 3+                  | SCHED_OTHER     |
| Logger (spdlog async)  | Core 3+                  | SCHED_OTHER     |

On a 4-core SOM all RT threads fit within cores 0–2, leaving core 3 for OS + logging + non-RT consumers. On an 8-core SOM the layout can be relaxed or replicated for redundant estimator instances.

---

## 6. Estimator Interface Summary

```
INavigationEstimator
├── EkfEstimator    — Error-state EKF, 15-state (+ optional clock/baro bias)
└── UkfEstimator   — Merwe-sigma UKF, same 15-state error space

Both support:
  AddMeasurement(SensorMeasurement)  → MeasurementUpdateReport
  ProcessUntil(Timestamp)            → force IMU propagation
  RegisterMeasurementModel(model)    → plug-in sensor models
  Reset() / IsInitialized()
  LatestState()                      → std::optional<NavigationState>
```

Measurement models are registered at startup and dispatched via `CanHandle()`. Adding a new sensor type requires only a new model class — no changes to the estimator core.

---

## 7. Logging

FalconGuide uses **spdlog** with three configurable sinks:

| Sink            | Purpose                              | Default        |
|----------------|--------------------------------------|----------------|
| Console (color) | Development and debug                | Enabled        |
| Rotating file   | Persistent on-vehicle log            | Configurable   |
| UDP network     | Real-time monitoring (GCS / laptop)  | Configurable   |

Log level is tunable at runtime via `falconguide::log::SetLevel()`. In production, `Info` level is recommended; `Debug` level adds per-measurement lines and may produce high log volume at full sensor rate.

Key log events:

| Level    | Event                                          |
|---------|------------------------------------------------|
| `INFO`  | Filter initialized (LTP origin coordinates)    |
| `INFO`  | Filter reset                                   |
| `WARN`  | IMU out-of-order                               |
| `WARN`  | GNSS init failed (NoFix / invalid)             |
| `WARN`  | Large correction > 10 m (possible outlier)     |
| `WARN`  | Cholesky decomposition failed, regularizing    |
| `WARN`  | No measurement model matched                   |
| `DEBUG` | Per-measurement accept / reject with model name and correction norm |

---

## 8. Build Configuration

```cmake
cmake -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFALCON_GUIDE_BUILD_TESTS=ON

cmake --build build -j$(nproc)
```

Dependencies are fetched automatically via CMake `FetchContent` if not found on the system:

| Dependency | Version  | Source          |
|-----------|----------|-----------------|
| Eigen     | 3.4.0    | GitLab          |
| spdlog    | 1.14.1   | GitHub          |
