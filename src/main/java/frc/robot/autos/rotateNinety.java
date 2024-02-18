package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class rotateNinety extends Command{
  private Swerve s_Swerve;

  public rotateNinety(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }
  @Override
  public void initialize() {
    s_Swerve.drive(new Translation2d(0, .1), 0, true, false);
  }
  
  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}