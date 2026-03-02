#!/bin/bash
# Toggle the backpack scanner service on/off.
# Designed to be launched from a desktop shortcut or physical GPIO button.

SERVICE="backpack-scanner.service"

if systemctl is-active --quiet "$SERVICE"; then
    echo "Stopping backpack scanner..."
    sudo systemctl stop "$SERVICE"
    echo ""
    echo "✓ Stopped."
else
    echo "Starting backpack scanner..."
    sudo systemctl start "$SERVICE"
    # Wait a moment for Python to start and print status
    sleep 2
    echo ""
    if systemctl is-active --quiet "$SERVICE"; then
        echo "✓ Started. Connect phone to BackpackScanner WiFi → http://10.42.0.1:5000"
    else
        echo "✗ Failed to start. Check log below:"
    fi
fi

echo ""
echo "--- Service log (last 15 lines) ---"
journalctl -u "$SERVICE" -n 15 --no-pager
echo ""
echo "Press Enter to close."
read
