#!/bin/bash

# Make sure to export the wifi username and password as env vars before running!
# Run as root and on boot!
iwctl known-networks $BB_SSID forget
iwctl station wlan0 scan
iwctl station wlan0 get-networks
iwctl --passphrase $WIFI_PASS station wlan0 connect $BB_SSID

