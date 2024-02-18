package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
   
    public void ConfigureLEDs() {
        m_led = new AddressableLED(8);
        m_ledBuffer = new AddressableLEDBuffer(35);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    } 
    
    public void grayLED() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 60, 60, 60);
        }
       m_led.setData(m_ledBuffer);
    }
    public void whiteLED() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 255, 255);
        }
       m_led.setData(m_ledBuffer);
    }
    public void blueLED() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 255);
        }
       m_led.setData(m_ledBuffer);
    }
    public void greenLED() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 255, 0);
        }
       m_led.setData(m_ledBuffer);
    }
    public void turnOffLED() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
       m_led.setData(m_ledBuffer);
    }
}