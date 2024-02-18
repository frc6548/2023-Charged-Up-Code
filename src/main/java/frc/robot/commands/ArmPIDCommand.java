package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmPIDCommand extends Command {
    private final Arm armSubsystem;
    private final PIDController pidController;

    public ArmPIDCommand(Arm armSubsystem, double setpoint) {
        this.armSubsystem = armSubsystem;
        this.pidController = new PIDController(//
        ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        pidController.setSetpoint(setpoint);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ArmPIDCmd started!");
        pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(armSubsystem.getEncoder());
        armSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotor(0);
        System.out.println("ArmPIDCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}