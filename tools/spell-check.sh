#!/bin/bash
# shellcheck disable=SC2046
set -euo pipefail
IFS=$'\n\t'
cd "$(dirname "$0")"/..

# Usage:
#    ./tools/spell-check.sh

if [[ -n "$(git ls-files '*Cargo.toml')" ]]; then
    dependencies=''
    for manifest_path in $(git ls-files '*Cargo.toml'); do
        if [[ "${manifest_path}" != "Cargo.toml" ]] && ! grep -Eq '\[workspace\]' "${manifest_path}"; then
            continue
        fi
        metadata=$(cargo metadata --format-version=1 --all-features --no-deps --manifest-path "${manifest_path}")
        for id in $(jq <<<"${metadata}" '.workspace_members[]'); do
            dependencies+=$'\n'
            dependencies+=$(jq <<<"${metadata}" ".packages[] | select(.id == ${id})" | jq -r '.dependencies[].name')
        done
    done
    words=''
    # shellcheck disable=SC2001
    for word in $(sed <<<"${dependencies}" 's/[0-9_-]/\n/g' | LC_ALL=C sort -f -u | (grep -E '.{4,}' || true)); do
        # Skip if the word is contained in other dictionaries.
        if ! npx cspell trace "${word}" 2>/dev/null | (grep -v -E '/(project-dictionary|rust-dependencies)\.txt' || true) | grep -Eq "^${word} \* [0-9A-Za-z_-]+\* "; then
            words+=$'\n'
            words+="${word}"
        fi
    done
fi
cat >.github/.cspell/rust-dependencies.txt <<EOF
// This file is @generated by $(basename "$0").
// It is not intended for manual editing.
EOF
if [[ -n "${words:-}" ]]; then
    echo "${words}" >>.github/.cspell/rust-dependencies.txt
fi

npx cspell --no-progress $(git ls-files)

# Check duplications in dictionary
for dictionary in .github/.cspell/*.txt; do
    if [[ "${dictionary}" == .github/.cspell/project-dictionary.txt ]]; then
        continue
    fi
    dup=$(sed '/^$/d' .github/.cspell/project-dictionary.txt "${dictionary}" | LC_ALL=C sort -f | uniq -d -i | (grep -v '//.*' || true))
    if [[ -n "${dup}" ]]; then
        echo "error: duplicated words in dictionaries; please remove the following words from .github/.cspell/project-dictionary.txt"
        echo "======================================="
        echo "${dup}"
        echo "======================================="
    fi
done

# Fail CI if .github/.cspell/rust-dependencies.txt needs to be updated.
if [[ -n "${CI:-}" ]]; then
    if ! git --no-pager diff --exit-code .github/.cspell/rust-dependencies.txt; then
        echo "error: please update .github/.cspell/rust-dependencies.txt"
    fi
fi