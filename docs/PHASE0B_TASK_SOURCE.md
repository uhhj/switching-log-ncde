# Task source

This simulator task is a minimal adaptation of fixture-based DLO manipulation,
not a geometry-exact reproduction of a prior setup.

Primary task motivation:
K. Chen et al., "Contact-aware Shaping and Maintenance of Deformable Linear
Objects With Fixtures," arXiv:2307.10153, 2023.

That work performs cable routing with environmental fixtures and a clip-fixing
skill that establishes contact and pushes a cable into a clip while using
force/visual information.

Related benchmark-level task family:
"WireCraft: A Simulation Benchmark for Industrial DLO Manipulation,"
arXiv:2606.18097, 2026, including connector insertion, clip routing and
channel seating.

Our Phase 0B uses a simplified two-wall PyBullet fixture solely to create
physically realized contact transitions suitable for dynamics study. It is not
claimed to reproduce either prior hardware or simulator exactly.
