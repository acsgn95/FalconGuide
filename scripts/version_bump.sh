#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version_file="${project_root}/VERSION"
bump_type="${1:-patch}"

case "${bump_type}" in
    patch | minor | major) ;;
    *)
        echo "Usage: $0 [patch|minor|major]" >&2
        exit 2
        ;;
esac

if [[ ! -f "${version_file}" ]]; then
    echo "VERSION file not found" >&2
    exit 1
fi

current="$(tr -d '[:space:]' <"${version_file}")"
if [[ ! "${current}" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)$ ]]; then
    echo "Cannot bump non-basic semantic version: ${current}" >&2
    exit 1
fi

major="${BASH_REMATCH[1]}"
minor="${BASH_REMATCH[2]}"
patch="${BASH_REMATCH[3]}"

case "${bump_type}" in
    patch)
        patch=$((patch + 1))
        ;;
    minor)
        minor=$((minor + 1))
        patch=0
        ;;
    major)
        major=$((major + 1))
        minor=0
        patch=0
        ;;
esac

next="${major}.${minor}.${patch}"
printf '%s\n' "${next}" >"${version_file}"

echo "Version bumped: ${current} -> ${next}"
