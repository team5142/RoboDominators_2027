package frc.robot.commands.drive;

import static frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// Joystick teleoperated driving - runs as default command on DriveSubsystem
// Applies deadbands, a response curve, and slew rate limiting to prevent wheel twitching
public class DriveWithJoysticks extends Command {
  private final DriveSubsystem driveSubsystem;
  private final RobotState robotState;
  private final DoubleSupplier xSupplier; // Forward/back joystick axis
  private final DoubleSupplier ySupplier; // Left/right joystick axis
  private final DoubleSupplier omegaSupplier; // Rotation joystick axis
  private final BooleanSupplier fieldRelativeSupplier; // Field-relative vs robot-relative toggle
  private final BooleanSupplier precisionModeSupplier; // Slow mode toggle

  // Rate-limits x/y/omega so stick noise and hard reversals ramp instead of jumping instantly -
  // prevents wheels from micro-steering on tiny stick movements. Omega uses its own (looser)
  // rate since rotation is expected to feel snappier than translation.
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(TRANSLATION_SLEW_RATE_PER_SEC);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(TRANSLATION_SLEW_RATE_PER_SEC);
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(ROTATION_SLEW_RATE_PER_SEC);

  public DriveWithJoysticks(
      DriveSubsystem driveSubsystem,
      RobotState robotState,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fieldRelativeSupplier,
      BooleanSupplier precisionModeSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.robotState = robotState;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.fieldRelativeSupplier = fieldRelativeSupplier;
    this.precisionModeSupplier = precisionModeSupplier;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    if (robotState.isOperatorDriveLockout()) {
      driveSubsystem.drive(0.0, 0.0, 0.0, true);
      return;
    }

    // Apply deadband to ignore stick drift
    double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), JOYSTICK_DEADBAND);
    double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), JOYSTICK_DEADBAND);
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), JOYSTICK_DEADBAND);

    // x^1.5 curve: more responsive than squaring at low speeds, less twitchy than linear
    x = shapeAxis(x);
    y = shapeAxis(y);
    omega = shapeAxis(omega);

    // Rate-limit x/y/omega so stick noise and hard reversals ramp instead of jumping instantly.
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    omega = omegaLimiter.calculate(omega);

    // Desaturate combined translation + rotation to prevent module over-speed
    DesaturatedSpeeds desaturated = desaturate(x, y, omega);
    x = desaturated.x();
    y = desaturated.y();
    omega = desaturated.omega();

    // Convert normalized inputs to m/s and rad/s
    double xMetersPerSec = x * MAX_TRANSLATION_SPEED_MPS;
    double yMetersPerSec = y * MAX_TRANSLATION_SPEED_MPS;
    double omegaRadPerSec = omega * MAX_ANGULAR_SPEED_RAD_PER_SEC;

    // Apply speed scaling based on precision mode
    boolean precision = precisionModeSupplier.getAsBoolean();
    double translationScale = precision ? PRECISION_SPEED_SCALE     : NORMAL_SPEED_SCALE;
    double rotationScale    = precision ? PRECISION_ROTATION_SCALE  : NORMAL_ROTATION_SCALE;

    // Send drive command
    driveSubsystem.drive(
        xMetersPerSec     * translationScale,
        yMetersPerSec     * translationScale,
        omegaRadPerSec    * rotationScale,
        fieldRelativeSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0, true); // Stop robot when command ends
  }

  // x^1.5 response curve, pulled out so it can be unit tested independent of a controller.
  static double shapeAxis(double raw) {
    return Math.copySign(Math.pow(Math.abs(raw), 1.5), raw);
  }

  // Result of desaturate() - x/y/omega scaled down together so combined magnitude stays <= 1.0.
  record DesaturatedSpeeds(double x, double y, double omega) {}

  // Scales x/y/omega down together if their combined magnitude exceeds 1.0, so translation
  // and rotation never combine to command a module speed above what it can actually do.
  static DesaturatedSpeeds desaturate(double x, double y, double omega) {
    double translationMagnitude = Math.hypot(x, y);
    double combinedMagnitude = Math.hypot(translationMagnitude, omega);

    if (combinedMagnitude > 1.0) {
      double scale = 1.0 / combinedMagnitude; // Scale down to keep magnitude at 1.0
      return new DesaturatedSpeeds(x * scale, y * scale, omega * scale);
    }
    return new DesaturatedSpeeds(x, y, omega);
  }
}
