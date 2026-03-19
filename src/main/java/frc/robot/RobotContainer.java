// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Autos;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    private final CommandXboxController joystick1 = new CommandXboxController(0);

    private final CommandXboxController joystick2 = new CommandXboxController(1);

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    // Uncomment these if you want to test out slew rate limiters on the drivetrain. Note that these will limit the maximum acceleration of the robot, which may make it feel less responsive, but can help with traction and make the robot easier to control.
    // private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0); // Limit acceleration to 3 m/s^2
    // private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0); // Limit acceleration to 3 m/s^2

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        autoChooser.addOption("Refuel At Outpost Right", Autos.refuelAtOutpost_Right(drivetrain, intake, shooter));
        autoChooser.setDefaultOption("Shoot Preloads", 
        Commands.parallel(
            shooter.runShooterCommand().withTimeout(10),
            intake.runIntakeCommand().withTimeout(10)
        ));

        NamedCommands.registerCommand("shoot fuel", shooter.runShooterCommand());

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick1.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )

            // We can test this if we need it to help smooth out the drive train
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(limiterY.calculate((Math.abs(joystick1.getLeftY()) * joystick1.getLeftY()))) // Drive forward with negative Y (forward)g
            //         .withVelocityY(limiterX.calculate((Math.abs(joystick1.getLeftX()) * joystick1.getLeftX()))) // Drive left with negative X (left)
            //         .withRotationalRate(-(Math.abs(joystick1.getRightX()) * joystick1.getRightX())) // Drive counterclockwise with negative X (left)
            // )
            
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick1.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick1.getLeftY(), -joystick1.getLeftX()))
        ));

        joystick1.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick1.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick1.back().and(joystick1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick1.back().and(joystick1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick1.start().and(joystick1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick1.start().and(joystick1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick1.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        joystick2.a().whileTrue(intake.runIntakeCommand());
        joystick2.rightTrigger().whileTrue(shooter.runFlywheelCommand());
        joystick2.y().whileTrue(intake.runIntakeCommand());
        joystick2.y().whileTrue(shooter.runShooterCommand());
        joystick2.b().whileTrue(intake.runExtakeCommand());
        joystick2.rightBumper().whileTrue(shooter.runShooterCommandFar());
        joystick2.rightBumper().whileTrue(intake.runIntakeCommand());
      

        // Reset the field-centric heading on left bumper press.
        joystick1.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
