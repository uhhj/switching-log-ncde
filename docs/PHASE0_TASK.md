# Phase 0 active task

- Main task: Cable Constrained Passage.
- Geometry: converging funnel followed by a narrow throat.
- Initialization: constraint-consistent canonical cable spawn.
- Controller: deterministic lateral alignment, insertion and hold.
- Oracle signals: simulator-native fixture contact, force, relative tangential velocity and progress.
- Mode representation: hierarchical contact, friction and jam states plus transition events.

The previous open two-wall channel is closed and is not an active benchmark.
Every evaluated branch starts from one contact-free PyBullet snapshot. No cable
teleportation, fixture movement, constraint rebuilding or parameter change is
allowed after that snapshot.
