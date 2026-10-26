#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
zephyr_repo="${root}/../zephyr"
check_only=0

usage() {
  cat <<'EOF'
Usage: apply-zephyr-patches.sh [--zephyr <checkout>] [--check]

Options:
  --zephyr <checkout>  Zephyr checkout (default: ../zephyr)
  --check              Check applicability without modifying the checkout
  -h, --help           Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --zephyr)
      zephyr_repo="$2"
      shift 2
      ;;
    --check)
      check_only=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! git -C "${zephyr_repo}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Not a Git checkout: ${zephyr_repo}" >&2
  exit 1
fi

apply_one() {
  local patch="$1"

  if git -C "${zephyr_repo}" apply --reverse --check "${patch}" >/dev/null 2>&1; then
    echo "already applied: $(basename "${patch}")"
    return
  fi

  git -C "${zephyr_repo}" apply --check "${patch}"
  if [[ "${check_only}" -eq 1 ]]; then
    echo "applicable: $(basename "${patch}")"
    return
  fi

  git -C "${zephyr_repo}" apply "${patch}"
  echo "applied: $(basename "${patch}")"
}

shopt -s nullglob
patches=("${root}"/patches/zephyr/*.diff)
shopt -u nullglob

if [[ ${#patches[@]} -eq 0 ]]; then
  echo "No Zephyr patches found under ${root}/patches/zephyr" >&2
  exit 1
fi

for patch in "${patches[@]}"; do
  apply_one "${patch}"
done
