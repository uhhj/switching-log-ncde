# Phase 0B Fixture Task

Phase 0B qualifies a minimal two-wall channel as a source of physically realized hybrid cable dynamics. One cable endpoint is grasped and moved to a contact-free staging state. Four scripted branches then start from the same PyBullet snapshot: centered nominal insertion, an identical nominal repeat, a lateral slide probe, and a larger-offset jam probe.

The branch names describe interventions, not modes. The simulator logs only cable-to-fixture contact points, forces, tangential speeds, command context, and geometry. Offline analysis derives free, contact-transition, slip-like, stick-like, and jam-like behavior from those raw channels. Plane contact is excluded.

The fixture is a surrogate consisting of two static PyBullet boxes. It is adapted from fixture-based cable clip-fixing and channel-seating tasks in prior DLO manipulation work; it is not a geometry-exact reproduction of any prior apparatus or simulator.

The legacy joint-space endpoint acquisition can sweep the cable through a fixture placed from the reset pose. Cable-wall collision is therefore disabled only while constructing the grasp/staging precondition. A fixed two-stage procedure repositions the walls along the grasped endpoint's current tangent beyond the forward projection of every current bead plus the configured entry clearance, then executes a precise endpoint stretch. This is a deterministic adaptation of the source task's stretch precondition, not a geometry search. Collision is restored before the fixed settling interval and common snapshot. It remains enabled throughout every experimental branch; a snapshot is rejected unless the restored fixture reports zero cable contact.

Phase 0B is successful only if same-condition repeat drift is low, nominal insertion remains executable, real fixture contact is sustained, slide probes naturally realize slip, jam probes naturally realize jam, and the expected contact transitions occur across fixed seeds. These thresholds are internal smoke gates rather than field standards.
