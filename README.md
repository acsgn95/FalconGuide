# FalconGuide

FalconGuide is a C++ Visual-Inertial GNSS Navigation System intended to run both
offline on datasets and online on production compute platforms.

The initial architecture separates sensor data models from estimation backends:

- `core`: type-safe coordinate frames, timestamps, and sensor measurement types.
  Current sensor models cover IMU, magnetometer, barometer, radar altimeter,
  wheel odometry, airspeed, range finder, optical flow, DVL, echo sounder,
  external odometry, GNSS, and multi-camera metadata for odometry and
  geo-reference workflows. The core also defines the navigation output state,
  quality metrics, and per-sensor health/contribution status.
- `sensors`: dataset/live sensor adapters. Planned.
- `estimation`: EKF, Ceres sliding-window, and GTSAM factor-graph backends. Planned.
- `estimation`: backend-neutral estimator interface for EKF, Ceres sliding-window,
  GTSAM factor-graph, and custom solvers. Concrete backends are planned.
- `apps`: offline replay tools and live runtime nodes. Planned.

## Build

Windows development is expected to use vcpkg. Install Visual Studio Build Tools
with the C++ workload, install vcpkg, then set `VCPKG_ROOT` before configuring:

```powershell
.\scripts\dev_windows.ps1
```

On Windows ARM machines, `windows-vcpkg` builds an x64 binary by default because
Visual Studio Build Tools commonly installs the x64 target first. For native
ARM64 builds, install the MSVC ARM64 target tools and run:

```powershell
.\scripts\dev_windows.ps1 -Preset windows-arm64-vcpkg
```

Linux can use system packages first:

```bash
./scripts/dev_linux.sh linux-system
```

If system packages are not available on Linux, use the FetchContent fallback:

```bash
./scripts/dev_linux.sh linux-fetchcontent
```

For production builds on Linux-based SOM targets, use the release preset:

```bash
./scripts/dev_linux.sh linux-production
```
