package frc.robot.commands.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.commands.drive.DriveWithJoysticks.DesaturatedSpeeds;
import org.junit.jupiter.api.Test;

// Unit tests for the pure stick-shaping math in DriveWithJoysticks.
class DriveWithJoysticksTest {

  private static final double DELTA = 1e-9;

  // --- shapeAxis ---

  @Test
  void shapeAxisLeavesZeroAndFullStickUnchanged() {
    assertEquals(0.0, DriveWithJoysticks.shapeAxis(0.0), DELTA);
    assertEquals(1.0, DriveWithJoysticks.shapeAxis(1.0), DELTA);
    assertEquals(-1.0, DriveWithJoysticks.shapeAxis(-1.0), DELTA);
  }

  @Test
  void shapeAxisReducesPartialStickAndPreservesSign() {
    // 0.5^1.5 = 0.353553...
    assertEquals(0.35355339059, DriveWithJoysticks.shapeAxis(0.5), 1e-8);
    assertEquals(-0.35355339059, DriveWithJoysticks.shapeAxis(-0.5), 1e-8);
  }

  // --- desaturate ---

  @Test
  void desaturateLeavesInputsUnchangedWhenUnderMagnitudeOne() {
    DesaturatedSpeeds result = DriveWithJoysticks.desaturate(0.3, 0.3, 0.3);

    assertEquals(0.3, result.x(), DELTA);
    assertEquals(0.3, result.y(), DELTA);
    assertEquals(0.3, result.omega(), DELTA);
  }

  @Test
  void desaturateScalesDownProportionallyWhenOverMagnitudeOne() {
    DesaturatedSpeeds result = DriveWithJoysticks.desaturate(1.0, 1.0, 1.0);

    // x and y should still be equal to each other, and omega equal to x - scaling is uniform.
    assertEquals(result.x(), result.y(), DELTA);
    assertEquals(result.x(), result.omega(), DELTA);

    // The scaled combined magnitude should be brought back down to exactly 1.0.
    double translationMagnitude = Math.hypot(result.x(), result.y());
    double combinedMagnitude = Math.hypot(translationMagnitude, result.omega());
    assertEquals(1.0, combinedMagnitude, 1e-9);
  }
}
