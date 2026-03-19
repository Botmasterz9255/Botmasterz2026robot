package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Autos {

    private static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private static final SwerveRequest idle = new SwerveRequest.Idle();
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    public static Command refuelAtOutpost_Right(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake, ShooterSubsystem shooter) {
        // Starting on right side of field, in front of bump.
        // 1. Turn to face hub
        // 2. Shoot
        // 3. Drive to outpost, turn around to face outpost
        // 4. Wait while intaking.
        // 5. Drive back to hub, turning to face it.
        // 6. Shoot.
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

            // Drive left to hub.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0)
                    .withVelocityY(0.3)
                    .withRotationalRate(0)
            )
            .withTimeout(1.5),
            drivetrain.applyRequest(() -> brake).withTimeout(0.1),

            // Shoot.
            Commands.parallel(
                shooter.runShooterCommand().withTimeout(3.5),
                intake.runIntakeCommand().withTimeout(3.5)
            ),

            // Drive back to outpost.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-0.3)
                    .withVelocityY(-0.3)
                    .withRotationalRate(0.3)
            )
            .withTimeout(3.0),
            drivetrain.applyRequest(() -> brake).withTimeout(0.1),

            // Wait for fuel at outpost and intake.
            intake.runIntakeCommand().withTimeout(2.5),

            // Drive forward to hub.
            Commands.parallel(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(0.3)
                        .withVelocityY(0.3)
                        .withRotationalRate(0.3)
                ),
                intake.runIntakeCommand() // while intaking.
            ).withTimeout(3.0),
            drivetrain.applyRequest(() -> brake).withTimeout(0.1),

            // Shoot.
            Commands.parallel(
                shooter.runShooterCommand().withTimeout(5),
                intake.runIntakeCommand().withTimeout(5)
            ),

            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }

    /*
    public static Command turnToAngle(CommandSwerveDrivetrain drivetrain, Rotation2d desiredHeading) {
        Rotation2d currentHeading = drivetrain.getPose().getRotation();
        currentHeading = Rotation2d.fromRadians(MathUtil.angleModulus(currentHeading.getRadians()));
        desiredHeading = Rotation2d.fromRadians(MathUtil.angleModulus(desiredHeading.getRadians()));

        
        double error = desiredHeading.getDegrees() - currentHeading.getDegrees();
        final double rotationSpeed;
        if (Math.abs(error) < 2.0) {
            rotationSpeed = 0;
        } else if (error > 0 || error <= 180) {
            rotationSpeed = 0.25;
        } else {
            rotationSpeed = -0.25;
        }
        
        return drivetrain.applyRequest(() ->
            drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed)
        );
    }
    */

    public static Command testDrive(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake, ShooterSubsystem shooter) {
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> Autos.idle)
        );
    }

    /*
    public static Command testTurnToAngle(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake, ShooterSubsystem shooter) {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(

            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            Commands.repeatingSequence(
                Autos.turnToAngle(drivetrain, Rotation2d.fromDegrees(30))
            ),
            //drivetrain.applyRequest(() -> Autos.turnToAngle(() -> drivetrain.getPose().getRotation(), Rotation2d.fromDegrees(30))),
            //drivetrain.applyRequest(() -> Autos.turnToAngle(() -> Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(30))),

            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
    */
}
