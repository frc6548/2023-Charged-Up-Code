package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmCommand extends Command {
    private final Arm armSubsystem;
    private final double speed;

    public ArmCommand(Arm armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.speed = speed;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ArmCommand started!");
    }

    @Override
    public void execute() {
        armSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotor(0);
        System.out.println("ArmCommand ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
