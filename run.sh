#!/bin/bash

# PlatformIO build and upload script for Teensy 4.0
# Usage: ./run.sh or source run.sh then type: run, build, upload, monitor, etc.

run() {
    echo "Building and uploading to Teensy 4.0..."
    pio run -t upload
}

build() {
    echo "Building..."
    pio run
}

upload() {
    echo "Uploading to Teensy 4.0..."
    pio run -t upload
}

monitor() {
    echo "Opening serial monitor (115200 baud)..."
    pio device monitor -b 115200
}

buildmon() {
    echo "Building, uploading, and opening monitor..."
    pio run -t upload && pio device monitor -b 115200
}

clean() {
    echo "Cleaning build artifacts..."
    pio run -t clean
}

help() {
    echo "Teensy 4.0 Development Commands:"
    echo "  run       - Build and upload"
    echo "  build     - Compile only"
    echo "  upload    - Upload to board"
    echo "  monitor   - Open serial monitor"
    echo "  buildmon  - Build, upload, and monitor"
    echo "  clean     - Clean build artifacts"
    echo "  help      - Show this message"
}

# If script is executed directly, run the build/upload
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    run
fi
