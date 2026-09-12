# 2027 Repository Cleanup Checklist

This checklist tracks removal of 2026 experiments and obsolete supporting files while preserving useful test infrastructure. Complete cleanup in small, buildable commits. Do not combine cleanup with the QuestNav 3 migration or localization redesign.

## 1. Retire the unfinished web interfaces

- [ ] Confirm the custom driver dashboard and touchscreen operator interface are no longer wanted.
- [ ] Remove `src/main/deploy/dashboard/`.
- [ ] Remove `src/main/deploy/operator/`.
- [ ] Remove `TouchscreenInterface` and its `RobotContainer` wiring.
- [ ] Remove NetworkTables publications used only by the custom web interfaces.
- [ ] Remove camera-stream configuration used only by the custom web interfaces.
- [ ] Preserve AdvantageKit, AdvantageScope, SmartDashboard, Elastic, auto selection, and AutoPilot behavior.
- [ ] Build, test, and commit this removal independently.

## 2. Delete unused deploy artifacts

- [ ] Confirm QuestNav 3 uses its headset-configured field layout.
- [ ] Delete `src/main/deploy/example.txt`.
- [ ] Delete `src/main/deploy/frc2025r2.fmap`.
- [ ] Delete `src/main/deploy/2026-rebuilt-andymark.json` after the QuestNav field-layout confirmation.
- [ ] Build and verify the expected roboRIO deploy contents.

## 3. Replace obsolete controls documentation

- [ ] Delete or archive `docs/DriverControls.html`, which documents the 2026 robot.
- [ ] Replace the nearly empty `docs/CONTROLS.md` when the 2027 control scheme is ready.
- [ ] Keep one maintainable 2027 controls source instead of parallel hand-maintained formats.

## 4. Clean non-code files out of the Java source tree

- [ ] Review `src/main/java/frc/robot/generated/Through360data - Sheet1.csv`.
- [ ] Review the generated controller JPG files.
- [ ] Review `src/main/java/frc/robot/generated/logFileSample.txt`.
- [ ] Decide whether `generate_controller_maps.py` remains useful.
- [ ] Move retained documentation tooling outside `src/main/java`; delete obsolete inputs and outputs.
- [ ] Preserve `TunerConstants.java`.
- [ ] Verify non-code source-tree files are not entering the robot JAR.

## 5. Replace 2026 PathPlanner assets with neutral tests

- [ ] Create a straight-translation test.
- [ ] Create a rotation test.
- [ ] Create a known-pose-seed test.
- [ ] Create a route-provider-to-AutoPilot handoff test.
- [ ] Create a Blue/Red alliance-flip regression test.
- [ ] Confirm `PoseInitializer` no longer depends on a removed 2026 auto.
- [ ] Delete obsolete 2026 autos, paths, and the 2026 navgrid only after replacement coverage exists.

## 6. Audit obsolete vendordeps and copied vendor sources

- [ ] Determine whether `Phoenix5-5.36.0.json` is still required.
- [ ] Determine whether `libgrapplefrc2026.json` is still required.
- [ ] Determine why `REVLib.json` and `REVLibJNI` remain in the CTRE-first codebase.
- [ ] Document why the copied sources under top-level `com/` were added.
- [ ] Remove dependencies and workarounds one at a time, building and testing after each removal.

## 7. Remove Limelight support

- [ ] Complete basic QuestNav 3 validation before changing other localization inputs.
- [ ] Remove `LimelightHelpers` and `LimelightCamera`.
- [ ] Remove Limelight constants, stream setup, dashboard entries, and subsystem construction.
- [ ] Remove Limelight-only fusion paths without starting PhotonVision localization work.
- [ ] Build, test degraded localization behavior, and commit independently.

## 8. Review small configuration artifacts

- [ ] Delete `.SysId/sysid.json` if WPILib does not use or regenerate the empty file.
- [ ] Retain `simgui-ds.json` if it supports the planned simulation workflow; otherwise delete it.
- [ ] Validate `docs/SYSID_SETUP.md` against current bindings and retain it if accurate.

## Recommended Order

1. Prove QuestNav 3 builds, communicates, and produces observable pose data.
2. Remove the custom web interfaces and their Java plumbing.
3. Delete stale deploy files and obsolete documentation.
4. Reorganize or delete generated controller assets.
5. Add neutral navigation tests, then remove 2026 PathPlanner assets.
6. Audit vendordeps and copied vendor sources.
7. Remove Limelight comprehensively.
