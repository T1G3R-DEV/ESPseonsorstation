#!/bin/bash

# Detect the ESP32 USB port (assumes it's the only /dev/ttyUSB* device)
PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -n 1)

if [ -z "$PORT" ]; then
  echo "❌ No ESP32 board found on /dev/ttyUSB*"
  exit 1
fi

echo "✅ Found ESP32 on $PORT"

# Set permissions
echo "🔧 Setting permissions on $PORT"
sudo chmod 666 "$PORT"

# Set your sketch folder
SKETCH_PATH="."

# Use generic ESP32 Dev Module
FQBN="esp32:esp32:esp32"

# Compile the sketch
echo "⚙️ Compiling..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH_PATH" #-e --optimize-for-debug
if [ $? -ne 0 ]; then
  echo "❌ Compilation failed."
  exit 1
fi

# Upload to board
echo "🚀 Uploading..."
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_PATH"
