# Learning log

## Session 1 (2026-09-30): skeleton, track graph, config loader

- **Built:** CMake project (core library, thin CLI, test target), GoogleTest via
  FetchContent, CI workflow, `TrackGraph`, plain-text track file loader.
- **Decisions:** adjacency list over matrix (sparse graph, Dijkstra iterates
  neighbours); `BlockId` as a strong type that is also a vector index;
  `std::optional` for expected failures (unknown name, bad input) and exceptions
  only for caller bugs (invalid id); loader is all-or-nothing and collects every
  error with its line number.
- **CMake ideas:** targets carry their own include paths and flags; warnings live
  on an interface target so third-party code is not affected.
- **Open:** install CMake locally; confirm CI is green after the first push.

## Session 2 (2026-09-30): CI fixes, route planner

- **Built:** `find_route` (Dijkstra over block lengths) and a CLI mode that prints
  the route between two stations.
- **Decisions:** Dijkstra over BFS (blocks have different lengths) and over A*
  (no coordinates for a heuristic, tiny graph); lazy deletion in
  `std::priority_queue` instead of decrease-key; availability passed in as a
  predicate so the planner does not depend on the interlocking; start block is
  never filtered; ties resolve the same way every run.
- **CI lessons:** clang-tidy cannot see through `ok()` to an optional check;
  `enum class` defaults to `int`; `main`'s signature needs a cppcheck suppression.
- **Open:** still no local CMake, clang-tidy, or cppcheck; CI is the only full check.

## Session 3 (2026-09-30): event logger, interlocking

- **Built:** `EventLogger` (in-memory record plus CSV sink) and `Interlocking`
  (reserve, enter, release, with a logged reason for every rejection).
- **Decisions:** one `holder` field per block so two trains in a block cannot be
  represented; deny by default; `MoveResult` enum instead of bool or strings;
  the planner is advisory and the interlocking is the authority, so `enter`
  re-checks everything; logger does not own its stream (the caller's
  `std::ofstream` is the RAII owner) and flushes every line.
- **Testing idea:** a seeded random test fires 20,000 requests and checks the
  safety rule after each one, not only the cases I thought of.
- **Open:** head-on trains can deadlock; trains are treated as one block long.

## Session 4 (2026-09-30): PID controller, train model

- **Built:** `PidController` (clamped output, anti-windup, derivative on the
  measurement) and `Train` (speed, braking curve, latched emergency brake).
- **Decisions:** the train knows nothing about the track and is told only the
  distance to the danger point; two layers, where the PID follows a braking
  curve and the emergency brake is an independent backstop; "not clear" logic so
  a NaN distance counts as danger; time passed in as `dt`, so tests need no clock.
- **CI lesson:** clang-tidy's optional check cannot follow an optional that is
  reassigned inside a loop; the route rebuild now walks plain `BlockId`s.
- **Open:** gains were tuned by trial for one train; trains have no length.

## Session 5 (2026-09-30): simulator, integration tests, README

- **Built:** `Simulator` (fixed time-step loop), `train` lines in scenario files,
  CLI commands `show`, `route`, `run`, two multi-train scenarios, integration
  tests, full README.
- **Decisions:** movement authority ends at the far end of the last block the
  train holds; the route is locked once the next block is held; a refused block
  is avoided when replanning and asked for only once; the simulator checks the
  safety rule itself each tick, independently of the interlocking.
- **Testing idea:** replay the event log using only `entered` and `left` events
  as a third, independent check, and test that checker against a fabricated bad log.
- **Found while testing:** a train starting in its destination drove off before
  stopping (fixed: target speed is zero inside the destination block); my first
  four-train ring setup was a genuine gridlock, which the design cannot resolve.
- **Open:** deadlock avoidance, train length, more than one block of lookahead.
