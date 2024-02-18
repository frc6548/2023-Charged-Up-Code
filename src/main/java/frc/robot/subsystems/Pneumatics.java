package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final DoubleSolenoid ds_pinch = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        PneumaticsConstants.kPinchSolenoidPorts[0], 
        PneumaticsConstants.kPinchSolenoidPorts[1]);
    private final DoubleSolenoid ds_arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        PneumaticsConstants.kArmSolenoidPorts[0], 
        PneumaticsConstants.kArmSolenoidPorts[1]);

        public void armToggle() {
            ds_arm.toggle();
        }
        public void armForward() {
            ds_arm.set(Value.kForward);
        }
        public void armReverse() {
            ds_arm.set(Value.kReverse);
        }

        public void pinchToggle() {
            ds_pinch.toggle();
        }
        public void pinchForward() {
            ds_pinch.set(Value.kForward);
        }
        public void pinchReverse() {
            ds_pinch.set(Value.kReverse);
        }

        public void compressorEnable() {
            m_compressor.enableDigital();
        }

        public void compressorDisable() {
            m_compressor.disable();
        }

        public void ConfigurePneumatics() {
            pinchForward();
            armForward();
        }
}
