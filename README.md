# [Cornell Autonomous Sailboat Project Team](https://cusail.com/): `boat` Repository
This repository contains everything that makes CUSail's boat autonomous: the code that runs on the boat itself, and the
setup, tooling, and tests that it depends on.

---

## Repository Structure
There are two main directories in this repository, which contain the code for each of our two on-board computers.
- The `jetson/` directory is the ROS 2 workspace containing the code that runs on our NVIDIA Jetson Orin Nano. See
  [jetson/setup/README.md](jetson/setup/README.md) for a guide on how to begin developing in this codebase.
- The `teensy/` directory contains the firmware for our Teensy 4.0 microcontroller. See [teensy/README.md](teensy/README.md) for
  information on how this codebase is organized.

## Continuous Integration (CI)
Changes to files in the `jetson/` directory run [Jetson CI](.github/workflows/jetson.yml).
Changes to files in the `teensy/` directory run [Teensy CI](.github/workflows/teensy.yml).
