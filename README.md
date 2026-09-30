# RailSim

A small command-line railway signaling simulator in C++17. Trains run on a
network of track blocks; a route planner suggests where to go, an interlocking
decides what is allowed, a PID controller drives each train, and every decision
is written to an event log that can be used to trace why something happened.

The safety rule the whole project is built around: **two trains are never in,
and never hold, the same block.**

## Build and test

Requires CMake 3.16+ and a C++17 compiler. GoogleTest is downloaded at configure time.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Run

```bash
./build/src/railsim show  scenarios/simple_line.txt
./build/src/railsim route scenarios/simple_line.txt West East
./build/src/railsim run   scenarios/two_trains_conflict.txt
./build/src/railsim run   scenarios/passing_loop.txt log.csv
```

- `show` prints the network.
- `route` prints the shortest route between two stations.
- `run` simulates the trains in the file. The event log goes to the given file,
  or to standard output if none is given. The exit code is 0 only if every train
  arrived and no safety violation was recorded.

### Scenario file

One statement per line; `#` starts a comment.

```
block   <name> <length_m> [out_of_service]
link    <block> <block>
station <station_name> <block>
train   <from_station> <to_station> <cruise_speed_mps>
```

Names must be declared before they are used. Trains are called T1, T2, ... in
file order. A file with errors is rejected as a whole, and every error is
reported with its line number.

### Reading the event log

The log is CSV: `tick,train,block,event,reason`. These lines are from
`two_trains_conflict.txt`, where two trains want the same junction block `J`:

```
tick,train,block,event,reason
0,T1,W1,route_planned,W1>J>M>E1
0,T1,J,reserved,
0,T2,W2,route_planned,W2>J>M>E2
0,T2,J,reserve_rejected,reserved_by_other_train
1,T2,W2,no_route,no_usable_route_to_destination
54,T1,J,entered,
...
134,T2,J,reserved,
```

To answer "why did T2 not move until tick 134?", read upwards from that line:
its request for `J` was refused at tick 0 because T1 already held it, and it had
no other route.

## Architecture

```
scenario file
     |
 config_loader ---> TrackGraph <------------------+
                        |                         |
                  route_planner              Interlocking ---> EventLogger ---> CSV
                        |                         ^                 ^
                        +-------> Simulator ------+-----------------+
                                      |
                                    Train ---> PidController
```

| Component | Files | Responsibility |
|---|---|---|
| Track graph | `track_graph` | Blocks, links and stations as an adjacency list. Static infrastructure only. |
| Config loader | `config_loader` | Parses and validates a scenario file. All or nothing. |
| Route planner | `route_planner` | Dijkstra by block length, skipping unusable blocks. Advisory. |
| Interlocking | `interlocking` | Owns reservations and occupancy. The only safety authority. |
| Event logger | `event_logger` | Records every event in memory and as CSV. |
| PID controller | `pid_controller` | Clamped PID with anti-windup. Pure arithmetic. |
| Train | `train` | Speed, braking curve, emergency brake. Knows nothing about the track. |
| Simulator | `simulator` | Fixed time-step loop that connects the parts and checks the safety rule. |

All logic is in the static library `railsim_core`. The executable and the tests
link against it, so the tests exercise exactly the code that ships.

## Design decisions

- **Adjacency list, not a matrix.** Track is sparse (two or three links per
  block) and Dijkstra only asks for a block's neighbours.
- **Strong id types.** `BlockId` and `TrainId` are separate structs, so passing
  one where the other is expected does not compile.
- **`std::optional` for expected failures, exceptions for caller bugs.** An
  unknown station name or "no route" is a normal outcome and is part of the
  return type. An invalid id or a non-positive time step can only come from a
  bug, so it throws.
- **The planner is advisory; the interlocking is the authority.** A route is
  only correct at the moment it is computed. Safety never depends on it: every
  move is checked again by the interlocking when it happens.
- **Deny by default.** The interlocking grants a request only after every check
  has passed, and performs all checks before changing any state.
- **One holder per block.** Reservation and occupation share a single `holder`
  field, so two trains in one block cannot be represented.
- **Two layers of speed supervision.** The PID follows a braking curve that
  stops the train with the service brake. Independently, an emergency brake
  takes over if the train can no longer stop in time, and stays on until
  standstill. It does not go through the PID.
- **Unknown means danger.** A NaN distance to the danger point is treated as
  "not clear".
- **Determinism.** Trains are processed in id order, route ties resolve the same
  way every time, and time is a parameter, not a clock. The same input always
  gives the same log, which makes a failure reproducible from its log.
- **The log is flushed per event.** Slower, but the log is complete up to the
  last event if the program stops unexpectedly.

## Verification

- **Unit tests** for each component, including invalid input.
- **Property-style test** of the interlocking: 20,000 random requests with a
  fixed seed, checking the safety rule after each one.
- **Integration tests** that run full scenarios: two trains competing for one
  block, opposing trains using a passing loop, a head-on deadlock, four trains
  on a ring, and a block closed in front of a train.
- **Three independent checks of the safety rule** in the integration tests: the
  interlocking's own state, the simulator's per-tick check of train positions,
  and a replay of the event log that uses only `entered` and `left` events.
- **CI** on Ubuntu builds with warnings as errors, runs the tests, and runs
  clang-tidy and cppcheck.

## Known limitations

- **Deadlock is possible.** Two trains facing each other on a single line both
  wait forever. This is safe but not live; the run ends with `timeout` events.
  A real system reserves a complete route at once and locks the direction of a
  single-line section.
- **Trains have no length.** A train is in exactly one block and frees the block
  behind it the moment it crosses the boundary.
- **One block of lookahead.** A train holds only the next block, which limits
  its speed on short blocks.
- **Conservative planning.** A train does not set off while any block on its
  only route is occupied, even a distant one.
- **Arrived trains stay on the network** and keep occupying their destination.
- **Blocks must be longer than twice the safety margin** (40 m by default);
  the simulator refuses a network that has a shorter one.
- **One speed profile.** No gradients, no per-block speed limits, and PID gains
  tuned by trial for the default train.
- **Links have no direction and no points (switches).** Any linked block can be
  entered from any other.
- **No escaping in the CSV log.** Names containing commas would break it.
- This is a teaching project. It follows ideas from railway signaling but is not
  built to any safety standard.
