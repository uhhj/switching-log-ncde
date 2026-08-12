# Phase 0S-SOFA Native Contact Bridge R1-B

## Verdict

PHASE0S_SOFA_NATIVE_CONTACT_BRIDGE_PASS

## Runtime

- SOFA: official v26.06.00 Linux binary
- BeamAdapter / SofaPython3: release-bundled v26.06
- CPU-only: yes

## Plugin build

- CMake source: `native-contact-bridge/CMakeLists.txt`
- SOFA prefix: `/root/workspace/third_party/SOFA_v26.06.00_Linux`
- library: `/root/workspace/switching-log-ncde/simulator-spikes/sofa-beam-spike/native-contact-bridge/build/lib/libNativeContactBridge.so`
- full SOFA rebuild: no
- SofaPython3 rebuild: no

## Native source

- `NarrowPhaseDetection::getDetectionOutputs()`: yes
- exported: `DetectionOutput::elem`, `id`, `point[2]`, `normal`, and raw `value`
- `DetectionOutput::value` is reported only as a raw detection value.

## Stage-A smoke

- requested / completed steps: 20 / 20
- bridge ready frames: 20
- frame serial min/max: 1 / 20
- native contact frames / total / max per frame: 20 / 120 / 6
- beam primitive ID range: 0 / 6
- fixture primitive ID range: 0 / 1
- normal norm min/median/max: 1.0 / 1.0 / 1.0
- DetectionOutput value min/max: -0.001 / -0.0006241090541037936
- failures: []

## Proxy policy

- nearest-plane formal contact / velocity: no / no
- endpoint formal velocity: no
- LCP vector as formal contact identity: no
- stdout parsed for formal contact: no

## Fact

The bridge read only the official v26.06 narrow-phase DetectionOutput map and exported ordinary SOFA Data.

## Inference

The native-contact observability path is validated; controlled contact mechanics remain untested.

## Unknown

Stages B-F, Oracle calibration, held-out capability, and unified passage remain untested.

## Next action

Resume Phase 0S-SOFA Controlled Contact Construction R1 Revised at Stage B: run the anchored-free reaction-contamination control using NativeContactBridge as the formal native contact identity, then stop for review before Stage C if the LCP reaction proxy is contaminated.
