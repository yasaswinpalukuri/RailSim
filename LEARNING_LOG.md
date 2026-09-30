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
