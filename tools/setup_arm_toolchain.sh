#!/usr/bin/env bash
# Fixed bare-metal compiler distribution, restored by Actions cache when available.
set -euo pipefail
version=13.2.1-1.1
install_dir="$PWD/.toolchains/arm"
if [[ ! -x "$install_dir/bin/arm-none-eabi-gcc" ]]; then
  archive="xpack-arm-none-eabi-gcc-${version}-linux-x64.tar.gz"
  base="https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v${version}"
  scratch_dir="$(mktemp -d)"
  trap 'rm -rf "$scratch_dir"' EXIT
  curl --fail --location --retry 2 --retry-max-time 240 --connect-timeout 20 --max-time 180 "$base/$archive" -o "$scratch_dir/$archive"
  curl --fail --location --retry 2 --retry-max-time 60 --connect-timeout 20 --max-time 30 "$base/$archive.sha" -o "$scratch_dir/$archive.sha"
  (cd "$scratch_dir" && sha256sum --check "$archive.sha")
  mkdir -p "$install_dir"
  tar -xzf "$scratch_dir/$archive" --strip-components=1 -C "$install_dir"
fi
[[ "$("$install_dir/bin/arm-none-eabi-gcc" -dumpmachine)" == arm-none-eabi ]]
"$install_dir/bin/arm-none-eabi-gcc" --version
echo "$install_dir/bin" >> "$GITHUB_PATH"
