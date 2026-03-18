package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
//import frc.robot.generated.TunerConstants.Constants;


public class ClimberSubsystem extends SubsystemBase{
     // Climber Motor Controllers

  private SparkMax m_climber; // NEO motor
  //private SparkMaxConfig motorConfig;

  /** Subsystem for controlling the climber */
  public ClimberSubsystem() {


    // Setup using SparkMax motor controller
    m_climber = new SparkMax(TunerConstants.CLIMBER_CAN_ID, MotorType.kBrushless);
    
    //motorConfig = new SparkMaxConfig();



    // Reverse it if needed
    // m_climber.setInverted(Constants.CLIMBER_INVERT);

    // Put the default speed on SmartDashboard
    // SmartDashboard.putNumber("Climber Speed", Constants.CLIMBER_SPEED);
  }

  /* Set power to the climber motor */
  public void setPower(double power) {
 

    // Spark Max set method
    m_climber.set(power);
  }

  /*public double getPower(){
    return m_climber.getEncoder();
  }*/

  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {}
}
    

