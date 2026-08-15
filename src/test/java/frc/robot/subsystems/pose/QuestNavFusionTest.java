package frc.robot.subsystems.pose;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

// Unit tests for the pure teleport-threshold math in QuestNavFusion.
// No robot, hardware, or Logger needed - exceedsTeleportThreshold() is a plain function.
class QuestNavFusionTest {

  private static final double TRANS_LIMIT_M = 0.6;
  private static final double ROT_LIMIT_RAD = 1.4;

  @Test
  void passesWhenBothErrorsWellUnderLimit() {
    assertFalse(QuestNavFusion.exceedsTeleportThreshold(0.1, 0.2, TRANS_LIMIT_M, ROT_LIMIT_RAD));
  }

  @Test
  void failsWhenTranslationErrorExceedsLimit() {
    assertTrue(QuestNavFusion.exceedsTeleportThreshold(0.8, 0.0, TRANS_LIMIT_M, ROT_LIMIT_RAD));
  }

  @Test
  void failsWhenRotationErrorExceedsLimit() {
    assertTrue(QuestNavFusion.exceedsTeleportThreshold(0.0, 1.8, TRANS_LIMIT_M, ROT_LIMIT_RAD));
  }

  @Test
  void passesExactlyAtLimit() {
    // Boundary is strict ">" in the production code, so equal-to-limit should pass.
    assertFalse(QuestNavFusion.exceedsTeleportThreshold(TRANS_LIMIT_M, ROT_LIMIT_RAD, TRANS_LIMIT_M, ROT_LIMIT_RAD));
  }

  @Test
  void failsJustOverLimit() {
    assertTrue(QuestNavFusion.exceedsTeleportThreshold(TRANS_LIMIT_M + 0.001, 0.0, TRANS_LIMIT_M, ROT_LIMIT_RAD));
  }

  // --- exceedsImpliedVelocityThreshold ---

  private static final double MAX_SPEED_MPS = 7.15;
  private static final double MAX_OMEGA_RAD_PER_SEC = 16.9;

  @Test
  void impliedVelocityPassesWhenUnderMax() {
    assertFalse(QuestNavFusion.exceedsImpliedVelocityThreshold(2.0, 1.0, MAX_SPEED_MPS, MAX_OMEGA_RAD_PER_SEC));
  }

  @Test
  void impliedVelocityFailsWhenSpeedOverMax() {
    assertTrue(QuestNavFusion.exceedsImpliedVelocityThreshold(9.0, 0.0, MAX_SPEED_MPS, MAX_OMEGA_RAD_PER_SEC));
  }

  @Test
  void impliedVelocityFailsWhenOmegaOverMax() {
    assertTrue(QuestNavFusion.exceedsImpliedVelocityThreshold(0.0, 20.0, MAX_SPEED_MPS, MAX_OMEGA_RAD_PER_SEC));
  }

  @Test
  void impliedVelocityPassesExactlyAtMax() {
    // Same strict ">" semantics as the absolute teleport gate - equal-to-max passes.
    assertFalse(QuestNavFusion.exceedsImpliedVelocityThreshold(
        MAX_SPEED_MPS, MAX_OMEGA_RAD_PER_SEC, MAX_SPEED_MPS, MAX_OMEGA_RAD_PER_SEC));
  }

  // --- withinVelocityGate ---

  private static final double MAX_LINEAR_MPS = 1.8;
  private static final double MAX_ANGULAR_RAD_PER_SEC = 3.0;

  @Test
  void velocityGatePassesWhenBothUnderMax() {
    assertTrue(QuestNavFusion.withinVelocityGate(0.5, 0.5, MAX_LINEAR_MPS, MAX_ANGULAR_RAD_PER_SEC));
  }

  @Test
  void velocityGateFailsWhenLinearOverMax() {
    assertFalse(QuestNavFusion.withinVelocityGate(2.5, 0.0, MAX_LINEAR_MPS, MAX_ANGULAR_RAD_PER_SEC));
  }

  @Test
  void velocityGateFailsWhenAngularOverMax() {
    assertFalse(QuestNavFusion.withinVelocityGate(0.0, 4.0, MAX_LINEAR_MPS, MAX_ANGULAR_RAD_PER_SEC));
  }

  @Test
  void velocityGatePassesExactlyAtMax() {
    // Production code uses "<=" here, unlike the strict "<" tolerance check in PoseValidator -
    // equal-to-max still counts as within the gate.
    assertTrue(QuestNavFusion.withinVelocityGate(MAX_LINEAR_MPS, MAX_ANGULAR_RAD_PER_SEC, MAX_LINEAR_MPS, MAX_ANGULAR_RAD_PER_SEC));
  }
}
