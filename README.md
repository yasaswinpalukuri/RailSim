# RailSim

A small command-line railway signaling simulator in C++17. Work in progress;
the full README (architecture, design decisions, limitations) is written in step 5.

## Build and test

Requires CMake 3.16+ and a C++17 compiler. GoogleTest is downloaded at configure time.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Run

```bash
./build/src/railsim scenarios/simple_line.txt
```

Prints the track network. Add two station names to get the shortest route between them:

```bash
./build/src/railsim scenarios/simple_line.txt West East
```

## Status

- [x] Step 1: build skeleton, CI, track graph, config loader
- [x] Step 2: route planner (Dijkstra)
- [ ] Step 3: event logger and interlocking
- [ ] Step 4: PID speed controller and train
- [ ] Step 5: simulator, integration tests, full README
