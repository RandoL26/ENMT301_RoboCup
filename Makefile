.PHONY: build upload monitor clean help all test

# PlatformIO Makefile for Teensy 4.0 Project

help:
	@echo "Available commands:"
	@echo "  make build    - Compile the code"
	@echo "  make upload   - Build and upload to Teensy"
	@echo "  make monitor  - Open serial monitor (115200 baud)"
	@echo "  make build-monitor - Build, upload, and monitor"
	@echo "  make clean    - Remove build artifacts"
	@echo "  make all      - Build everything"
	@echo ""
	@echo "Quick aliases:"
	@echo "  make b        - Build"
	@echo "  make u        - Upload"
	@echo "  make m        - Monitor"

build:
	pio run

upload: build
	pio run -t upload

monitor:
	pio device monitor -b 115200

build-monitor: upload
	pio device monitor -b 115200

clean:
	pio run -t clean

all: clean build

# Quick aliases
b: build
u: upload
m: monitor
bm: build-monitor
