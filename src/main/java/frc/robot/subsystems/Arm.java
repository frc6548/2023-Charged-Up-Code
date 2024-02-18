package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private final CANSparkMax armMotor = new CANSparkMax(ArmConstants.armMotorId, MotorType.kBrushless);
    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Position", armEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Velocity", armEncoder.getVelocity());
    }

    public void setMotor(double speed) {
        armMotor.set(speed);
    }

    public double getEncoder() {
        return armEncoder.getPosition();
    }

    public void resetEncoder() {
        armEncoder.setPosition(0);
        armMotor.setIdleMode(IdleMode.kBrake);
    }
}