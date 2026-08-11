# Phase 0S-SOFA Environment Evidence

## Verdict

PHASE0S_SOFA_ENGINEERING_BLOCKED

## Runtime discovery

- CPU-only: yes (`CUDA_VISIBLE_DEVICES` was empty; no SofaCUDA preset used).
- `runSofa`: unavailable on the server.
- existing BeamAdapter/SofaPython3 installation: none found under the checked
  system and workspace locations.
- system Python: 3.8.10.
- SOFA source: official `v26.06.00` tag, commit
  `7c18e95d5c5f2839079892c69e7d89a313c79603`, cloned at
  `/root/workspace/third_party/sofa-v26.06`.
- SOFA build metadata: the matching source provides Pixi/CMake presets and a
  CPU-capable `standard` environment with SofaPython3 support.

## Provisioning attempt

Pixi 0.76.2 was installed from its official installer. Two attempts to resolve
the official `standard` environment ran for more than fifty minutes in total.
They downloaded and unpacked hundreds of MiB of dependencies, but never
produced an executable environment Python, `runSofa`, or a loadable
BeamAdapter runtime. The remaining live resolver was stopped after the second
attempt so it would not consume the shared server indefinitely.

## Unrun checks

- official free-beam example: not run (no runner)
- official collision-beam example: not run (no runner)
- custom beam smoke: not run (no verified API/runtime)

## Fact

The required CPU SOFA + BeamAdapter + SofaPython3 runtime was not available on
the provisioned server, and the official v26.06.00 dependency path did not
complete to a usable runner.

## Inference

No scientific statement about BeamAdapter mechanics, Oracle calibration,
capability, or the unified passage task is justified.

## Next action

Repair the SOFA/Pixi runtime provisioning only, then rerun the unchanged
environment smoke.
