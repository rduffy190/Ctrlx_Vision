#!/usr/bin/env bash
set -e
sudo snapcraft clean
sudo snapcraft --use-lxd --build-for=arm64