package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
//import com.revrobotics.spark.config.SparkMaxConfig;

//import frc.robot.generated.TunerConstants.Constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.generated.TunerConstants.Constants;
import frc.robot.generated.TunerConstants.ShooterSubsystemConstants;

public final class Configs {
   //private static final double nominalVoltage = 12.0;


   

public static final class IntakeSubsystem{
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig conveyorConfig = new SparkFlexConfig();

    static {

 // Configure basic settings of the intake motor
      intakeConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.5)
        .smartCurrentLimit(40);

      // Configure basic settings of the conveyor motor
      conveyorConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.5)
        .smartCurrentLimit(40);
      


    }




}
 public static final class ShooterSubsystemConfigs {
    public static final SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    public static final SparkFlexConfig flywheelFollowerConfig = new SparkFlexConfig();
    public static final SparkFlexConfig feederConfig = new SparkFlexConfig();

    static {
      // Configure basic setting of the flywheel motors
      flywheelConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .closedLoopRampRate(1.0)
        .openLoopRampRate(1.0)
        .smartCurrentLimit(80);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      flywheelConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          // .p(0.0002)
          .p(0.0)
          .outputRange(-1, 1);

      flywheelConfig.closedLoop.feedForward.kV(0.00187);

      flywheelConfig.closedLoop
        .maxMotion
          // Set MAXMotion parameters for MAXMotion Velocity control
          .cruiseVelocity(5000)
          .maxAcceleration(10000)
          .allowedProfileError(1);

      // Constants.NeoMotorConstants.kVortexKv is in rpm/V. feedforward.kV is in V/rpm sort we take
      // the reciprocol.
     // flywheelConfig.closedLoop
      //  .feedForward.kV(nominalVoltage / Constants.NeoMotorConstants.kVortexKv);

      // Configure the follower flywheel motor to follow the main flywheel motor
      flywheelFollowerConfig.apply(flywheelConfig)
       .follow(ShooterSubsystemConstants.kFlywheelMotorCanId, true);

      // Configure basic setting of the feeder motor
      feederConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(1.0)
        .smartCurrentLimit(60);
    }
  }


}
