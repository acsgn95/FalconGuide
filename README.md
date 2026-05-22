# FalconGuide

FalconGuide is a C++20 visual-inertial GNSS navigation system designed for both
offline dataset replay and online runtime use on production compute platforms.

The project is organized around strongly typed sensor and navigation models,
backend-neutral estimator interfaces, and interchangeable estimation backends.

## Project Status

FalconGuide is under active development. The current codebase includes:

- `core`: coordinate frames, timestamps, math helpers, navigation state, sensor
  measurement types, sensor calibration data, and health/quality reporting.
- `estimation`: backend-neutral estimator interfaces, measurement queues,
  navigation orchestration, EKF and UKF backends, and Ceres/GTSAM backend
  scaffolding.
- `io`: CSV measurement reading and NMEA output helpers.
- `app`: dataset/session configuration, replay-oriented application helpers,
  and IPC server scaffolding.
- `logger`: project logging configuration and wrapper utilities.

The GTSAM backend is currently kept out of CI builds with
`FALCONGUIDE_ENABLE_GTSAM=OFF` until the backend implementation and public header
shape are aligned.

## Requirements

Linux development expects:

- CMake 3.23 or newer
- Ninja
- a C++20 compiler
- Eigen3
- Ceres Solver, when `FALCONGUIDE_ENABLE_CERES=ON`
- nlohmann/json
- Doxygen, Graphviz, Java, and PlantUML for documentation generation
- Python and pre-commit for local repository checks

On Ubuntu, the core CI-style dependencies are:

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  ninja-build \
  libeigen3-dev \
  libceres-dev \
  nlohmann-json3-dev
```

## Build

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

The CI-compatible local configure/build/test flow is:

```bash
cmake -S . --preset linux-system -DFALCONGUIDE_ENABLE_GTSAM=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build build/linux-system
ctest --test-dir build/linux-system --output-on-failure
```

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

## Code Quality

FalconGuide uses pre-commit for repository hygiene:

- whitespace, final newline, YAML/JSON, merge-conflict, symlink, and private-key
  checks
- clang-format for C/C++ files
- cmake-format for CMake files
- codespell for common spelling mistakes

Install and run the hooks locally with:

```bash
python -m pip install pre-commit
pre-commit install
pre-commit run --all-files
```

Formatting is controlled by `.clang-format`, `.cmake-format.yaml`, and
`.editorconfig`.

## Documentation

Public APIs are documented with Doxygen comments. Generate the HTML
documentation with:

```bash
doxygen Doxyfile
```

The generated output is written to `build/doxygen/html` and should not be
committed.

When configured with documentation support, CMake also exposes a `docs` target:

```bash
cmake -S . --preset linux-system -DFALCON_GUIDE_BUILD_DOCS=ON -DFALCONGUIDE_ENABLE_GTSAM=OFF
cmake --build build/linux-system --target docs
```

## Versioning

`VERSION` is the single source of truth for the project version. CMake reads it
at configure time and uses it as `project(FalconGuide VERSION ...)`.

Validate the current version with:

```bash
./scripts/check_version.sh
```

Bump the version with:

```bash
./scripts/version_bump.sh patch
./scripts/version_bump.sh minor
./scripts/version_bump.sh major
```

FalconGuide follows semantic versioning in the form `MAJOR.MINOR.PATCH`.

## GitHub Workflows

The repository includes GitHub Actions workflows under `.github/workflows`:

- `CI`: runs pre-commit, configures the project, builds, and runs tests.
- `Docs`: generates Doxygen HTML and uploads it as an Actions artifact.

Pull requests should use the included template and should mention the checks
that were run locally. Issue templates are available for bug reports, feature
requests, and documentation problems.

## Contributing

See `CONTRIBUTING.md` for the expected local checks and documentation rules.

## Security

See `SECURITY.md` for vulnerability reporting guidance.
