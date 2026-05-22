#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version_file="${project_root}/VERSION"

if [[ ! -f "${version_file}" ]]; then
    echo "VERSION file not found" >&2
    exit 1
fi

version="$(tr -d '[:space:]' <"${version_file}")"
semver_regex='^([0-9]+)\.([0-9]+)\.([0-9]+)(-[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?(\+[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?$'

if [[ ! "${version}" =~ ${semver_regex} ]]; then
    echo "Invalid semantic version: ${version}" >&2
    echo "Expected MAJOR.MINOR.PATCH, optionally with prerelease/build metadata." >&2
    exit 1
fi

tag="v${version}"
if git -C "${project_root}" rev-parse "${tag}" >/dev/null 2>&1; then
    echo "Tag already exists: ${tag}" >&2
    exit 1
fi

echo "FalconGuide version is valid: ${version}"
