package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.Constants.IntakeSetpoints;

public class IntakeSubsystem extends SubsystemBase {

    private SparkFlex intakeMotor = new SparkFlex(TunerConstants.INTAKE_CAN_ID, MotorType.kBrushless);

    private SparkFlex conveyorMotor = new SparkFlex(TunerConstants.INTAKECONVEYOR_CAN_ID, MotorType.kBrushless);

    public IntakeSubsystem() {

    intakeMotor.configure(
        Configs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    conveyorMotor.configure(
        Configs.IntakeSubsystem.conveyorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    System.out.println("---> IntakeSubsystem intialized");

    }

    private void setIntakePower(double power) {
       intakeMotor.set(power);
    }

    private void setConveyorPower(double power){
       conveyorMotor.set(power);
    }
 
    public Command runIntakeCommand() {
      return this.startEnd(
         () -> {
           this.setIntakePower(IntakeSetpoints.kIntake);
           this.setConveyorPower(TunerConstants.Constants.ConveyorSetpoints.kIntake);
         }, () -> {
           this.setIntakePower(0.0);
           this.setConveyorPower(0.0);
        }).withName("Intaking");

    }

    public Command runExtakeCommand() {
      return this.startEnd(
          () -> {
            this.setIntakePower(IntakeSetpoints.kExtake);
            this.setConveyorPower(TunerConstants.Constants.ConveyorSetpoints.kExtake);
          }, () -> {
            this.setIntakePower(0.0);
            this.setConveyorPower(0.0);
          }).withName("Extaking");

    }
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Intake | Intake | Applied Output", intakeMotor.getAppliedOutput());
      SmartDashboard.putNumber("Intake | Conveyor | Applied Output", conveyorMotor.getAppliedOutput());
      SmartDashboard.putNumber("Intake in Amps| Intake | Applied Output AMPS", intakeMotor.getOutputCurrent());
      SmartDashboard.putNumber("Intake Conveyor in Amps | Applied Output", conveyorMotor.getOutputCurrent());

    }




    }
      
