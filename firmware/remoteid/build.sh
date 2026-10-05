#!/bin/bash
# Build ArduRemoteID (pinned tag) for the ESP32C3_DEV target with the heartbeat-state patch,
# in a work directory under this folder. Needs: git, curl, python3 with `uv` (or edit the pip
# line), ~1 GB of disk, and on Apple silicon Rosetta (the pinned esp32 2.0.3 core ships
# x86_64 macOS toolchains). Output: .work/ArduRemoteID/RemoteIDModule/ArduRemoteID-ESP32C3_DEV.bin
# (flash at 0x0) and ArduRemoteID_ESP32C3_DEV_OTA.bin (web update, needs LOCK_LEVEL -1 since it
# is not signed with an ArduPilot key).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
TAG=${ARID_TAG:-v1.14}
WORK=$HERE/.work
mkdir -p "$WORK"
export HOME=$WORK/home                      # keeps arduino-cli's cores and libraries out of ~/
export ARDUINO_DIRECTORIES_DATA=$HOME/.arduino15 ARDUINO_DIRECTORIES_USER=$HOME/Arduino
mkdir -p "$HOME"
if [ ! -d "$WORK/venv" ]; then
  uv venv -q "$WORK/venv"
  uv pip install -q -p "$WORK/venv/bin/python" 'empy==3.3.4' pymavlink dronecan pexpect future pyserial
fi
export PATH=$WORK/venv/bin:$PATH
if [ ! -d "$WORK/ArduRemoteID" ]; then
  git clone -q --branch "$TAG" --depth 1 --recurse-submodules --shallow-submodules \
      https://github.com/ArduPilot/ArduRemoteID.git "$WORK/ArduRemoteID"
  (cd "$WORK/ArduRemoteID" && git apply "$HERE/heartbeat-state.patch" && git diff --stat)
fi
cd "$WORK/ArduRemoteID"
mkdir -p bin
if [ ! -x bin/arduino-cli ]; then
  case "$(uname -s)-$(uname -m)" in
    Darwin-arm64) F=arduino-cli_0.27.1_macOS_ARM64.tar.gz ;;
    Darwin-*)     F=arduino-cli_0.27.1_macOS_64bit.tar.gz ;;
    *)            F=arduino-cli_0.27.1_Linux_64bit.tar.gz ;;
  esac
  curl -sfL "https://downloads.arduino.cc/arduino-cli/$F" | tar xz -C bin
fi
./scripts/regen_headers.sh
./scripts/add_libraries.sh
cd RemoteIDModule
make setup
make esp32c3dev
ls -l ArduRemoteID-ESP32C3_DEV.bin ArduRemoteID_ESP32C3_DEV_OTA.bin
shasum -a 256 ArduRemoteID-ESP32C3_DEV.bin
