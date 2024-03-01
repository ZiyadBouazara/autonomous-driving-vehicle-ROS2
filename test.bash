#!/bin/bash

packages="design3_system perception planning control"

colcon test --packages-select $packages --event-handlers=console_direct+ --pytest-with-coverage
colcon test-result --all
colcon coveragepy-result --packages-select $packages --verbose --coverage-report-args -m