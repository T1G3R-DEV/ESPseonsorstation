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

echo "🖥️ Opening serial monitor..."
#arduino-cli monitor -p "$PORT"
arduino-cli monitor -p "$PORT" -c baudrate=115200
