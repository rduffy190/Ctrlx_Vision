#!/usr/bin/env bash
set -e
sudo snapcraft clean
sudo snapcraft pack --build-for arm64 --verbosity=verbose