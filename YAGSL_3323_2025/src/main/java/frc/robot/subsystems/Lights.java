package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
    import edu.wpi.first.wpilibj.AddressableLEDBuffer;
    import edu.wpi.first.wpilibj.DriverStation;
    import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
    

 public class Lights extends SubsystemBase {
    

        
        private final AddressableLED m_led;
        private final AddressableLEDBuffer m_ledBuffer;
    
        public Lights (int port){
            // PWM port 0
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(port);
    
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());
    
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    
        // Set default color to purple
        var alliance = DriverStation.getAlliance();
        if (alliance != null) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                setSolidColor(227, 5, 5);
            } else {
                setSolidColor(62, 62, 255);
            }
        }
    
    
    
        }
    
        public void setSolidColor (int red, int green, int blue){
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                
                m_ledBuffer.setRGB(i, red, green, blue);
               
             }

             
             m_led.setData(m_ledBuffer);
        }
        public void setStripedColor (int red, int green, int blue){
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                
                m_ledBuffer.setRGB(i, red, green, blue);
               
             }

             
             m_led.setData(m_ledBuffer);
        }
    }
    

