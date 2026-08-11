# Phase 0S-SOFA Environment Evidence

## Verdict

ENVIRONMENT_PASS

## Runtime

- CPU-only: yes.  Every smoke command used `runSofa -g batch` in Docker
  without a GPU device request, `SofaCUDA`, or a CUDA preset.
- Official binary: `SOFA_v26.06.00_Linux_Python3.12.zip`, unpacked at
  `/root/workspace/third_party/SOFA_v26.06.00_Linux` on the execution host.
- Runner: `/sofa/bin/runSofa` from that unmodified release, executed in the
  local CPU-only image `sofa-v2606-python312-runner:ubuntu24`
  (`sha256:1046e3369c588223988f51938ff7ef8a83522e38aba6acba37b9bb640b590796`).
  The image provides Ubuntu 24.04's compatible glibc, Python 3.12 runtime,
  NumPy, and required OpenGL shared libraries; the SOFA release directory is
  mounted read-only at `/sofa`.
- SOFA source provenance: official `v26.06.00` tag, commit
  `7c18e95d5c5f2839079892c69e7d89a313c79603`.
- BeamAdapter: release-bundled plugin, commit
  `cac4005bd7c9f266c5c775ab88cebbd7c7fa83c7` (detached `origin/v26.06`).
- SofaPython3: release-bundled plugin, commit
  `727ce05f6956369a04f67e3f98da73de33f37a4a` (v26.06), using Python 3.12.3
  in the runtime image.

The Ubuntu 20.04 host cannot directly execute the official binary because it
lacks the release's required newer glibc/libstdc++ ABI.  The compatibility
container is therefore an execution adapter, not a replacement build and not
a Pixi-provisioned environment.

## Official example smoke

All commands used the final image, mounted the official release read-only,
and ran headlessly for exactly 20 simulation steps.

| Official scene | Command suffix | Result |
| --- | --- | --- |
| `plugins/BeamAdapter/examples/SingleBeam.scn` | `runSofa -g batch -n 20 SingleBeam.scn` | PASS; BeamAdapter loaded; 20 iterations; exit 0 |
| `plugins/BeamAdapter/examples/SingleBeamDeploymentCollision.scn` | `runSofa -g batch -n 20 SingleBeamDeploymentCollision.scn` | PASS; BeamAdapter and collision/deployment components loaded; 20 iterations; exit 0 |
| `plugins/BeamAdapter/examples/python3/SingleBeam.py` | `runSofa -l SofaPython3 -g batch -n 20 SingleBeam.py` | PASS; SofaPython3 and BeamAdapter loaded; 20 iterations; exit 0 |

Final logs contain no `ERROR` or `NaN` entries.  The XML free and collision
examples are the two required BeamAdapter smoke cases; the Python scene is an
additional executable check that SofaPython3 is functional, rather than merely
present on disk.

## Fact

An official, release-matched SOFA v26.06.00 CPU runtime with executable
`runSofa`, BeamAdapter, and SofaPython3 now exists on the server, and the two
official BeamAdapter examples complete their 20-step headless smoke runs.

## Inference

The environment gate is cleared.  This establishes only simulator and plugin
executability; it provides no evidence yet about contact-regime capability,
Oracle calibration, or the constrained-passage task.

## Next action

Run the unchanged one-shot SOFA Oracle calibration before adding any custom
contact-regime or passage scene.
