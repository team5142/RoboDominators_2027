package frc.robot.subsystems.pose;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

// Unit tests for the pure alignment-tolerance math in PoseValidator.
class PoseValidatorTest {

  private static final double POS_TOLERANCE_M = 0.15;
  private static final double ROT_TOLERANCE_DEG = 5.0;

  @Test
  void alignedWhenBothErrorsWellUnderTolerance() {
    assertTrue(PoseValidator.isWithinTolerance(0.02, 1.0, POS_TOLERANCE_M, ROT_TOLERANCE_DEG));
  }

  @Test
  void notAlignedWhenPositionErrorExceedsTolerance() {
    assertFalse(PoseValidator.isWithinTolerance(0.30, 0.0, POS_TOLERANCE_M, ROT_TOLERANCE_DEG));
  }

  @Test
  void notAlignedWhenRotationErrorExceedsTolerance() {
    assertFalse(PoseValidator.isWithinTolerance(0.0, 8.0, POS_TOLERANCE_M, ROT_TOLERANCE_DEG));
  }

  @Test
  void notAlignedExactlyAtTolerance() {
    // Production code uses strict "<", so a value equal to the tolerance is NOT aligned.
    // Opposite boundary behavior from QuestNavFusion's teleport gate (which uses ">"), worth
    // contrasting when reading both tests.
    assertFalse(PoseValidator.isWithinTolerance(POS_TOLERANCE_M, ROT_TOLERANCE_DEG, POS_TOLERANCE_M, ROT_TOLERANCE_DEG));
  }
}
