package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
    boolean compressorOn = false;
    boolean extended = false;

    Solenoid extendSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    Solenoid retractSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    
  /** Creates a new ExampleSubsystem. */
  public PneumaticsSubsystem() {
    //Basic Compressor information
    

    //pcmCompressor.enableDigital();
    //pcmCompressor.disable();

    //  boolean enabled = pcmCompressor.enabled(); 
    //boolean pressureSwitch = pcmCompressor.getPressureSwitchValue(); //Checking pressure
    //double current = pcmCompressor.getCompressorCurrent();

    //Basic Solenoid information
    
    //Solenoid exampleSolenoidPH = new Solenoid(PneumaticsModuleType.REVPH, 1);



    }
    //} 
  
  public void toggleCompressor(){
    if (compressorOn == true){
        pcmCompressor.disable();
        compressorOn = false;
    }
    else{
        pcmCompressor.enableDigital();
        compressorOn = true;
    }
  
  }
  public void toggleExtend(){
    retractSolenoidPCM.set(false);
    extendSolenoidPCM.set(true);

    //extendSolenoidPCM.set(false);
  }

  public void toggleRetract(){
    extendSolenoidPCM.set(false);
    retractSolenoidPCM.set( true);
    //retractSolenoidPCM.set( false);
  }

  public void toggleExtendandRetract(){
    if (extended){
      toggleRetract();
    }
    else {
      toggleExtend();
    }
    extended =! extended;
  }

 

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
