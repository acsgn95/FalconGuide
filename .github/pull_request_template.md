## Summary

-

## Verification

- [ ] `pre-commit run --all-files`
- [ ] `cmake -S . --preset linux-system -DFALCONGUIDE_ENABLE_GTSAM=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5`
- [ ] `cmake --build build/linux-system`
- [ ] `ctest --test-dir build/linux-system --output-on-failure`
- [ ] `doxygen Doxyfile`

## Notes

- [ ] Documentation updated
- [ ] Version impact considered
- [ ] Breaking changes documented
