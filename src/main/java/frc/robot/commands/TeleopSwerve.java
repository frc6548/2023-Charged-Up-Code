package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier leftBumper;
  private BooleanSupplier rightBumper;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  /**
   * The constructor initializes the class variables.
   * 
   * @param s_Swerve
   * @param translationSup
   * @param strafeSup
   * @param rotationSup
   * @param autoCenter
   * @param robotCentricSup
   */
  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier leftBumper,
      BooleanSupplier rightBumper) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.leftBumper = leftBumper;
    this.rightBumper = rightBumper;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal = translationLimiter
        .calculate(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double strafeVal = strafeLimiter
        .calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double rotationVal = rotationLimiter
        .calculate(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));

    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(
          leftBumper.getAsBoolean() ? 
          Constants.SwerveConstants.maxSpeedMaxLimit : 
          rightBumper.getAsBoolean() ? 
          Constants.SwerveConstants.maxSpeedMinLimit : 
          Constants.SwerveConstants.maxSpeed),
        rotationVal * (leftBumper.getAsBoolean() ? 
          Constants.SwerveConstants.maxAngularVelocityMaxLimit : 
          rightBumper.getAsBoolean() ? 
          Constants.SwerveConstants.maxAngularVelocityMinLimit : 
          Constants.SwerveConstants.maxAngularVelocity),
        !robotCentricSup.getAsBoolean(),
        false);
  }
}