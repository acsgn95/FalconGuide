# Contributing

Thanks for contributing to FalconGuide.

## Development Checks

Before opening a pull request, run:

```bash
pre-commit run --all-files
cmake -S . --preset linux-system -DFALCONGUIDE_ENABLE_GTSAM=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build build/linux-system
ctest --test-dir build/linux-system --output-on-failure
doxygen Doxyfile
```

## Versioning

`VERSION` is the single source of truth for the project version. Use semantic
versioning in the form `MAJOR.MINOR.PATCH`.

## Documentation

Public APIs should have Doxygen comments. Generated documentation is produced
with:

```bash
doxygen Doxyfile
```

The generated output lives under `build/doxygen` and should not be committed.
