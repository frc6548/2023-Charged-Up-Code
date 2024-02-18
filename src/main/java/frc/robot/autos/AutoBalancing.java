package frc.robot.autos;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoBalancing extends Command {
  private Swerve s_Swerve;
  
  private double currentAngle;
  private double drivePower;

  public AutoBalancing(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.currentAngle = s_Swerve.getRoll().getDegrees();

    drivePower = .025 * currentAngle;
    if(Math.abs(currentAngle) < .001){
      drivePower = 0;
    }
      
    s_Swerve.drive(new Translation2d(-drivePower, 0), 0, true, true);
    
    System.out.println("Current Angle: " + currentAngle);
    System.out.println("Drive Power: " + drivePower);
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(0, .3), 0, true, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}