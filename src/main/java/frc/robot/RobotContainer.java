// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;



public class RobotContainer {
    

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(Constants.MaxSpeed * 0.1)
            .withRotationalDeadband(Constants.MaxAngularRate * 0.1);

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController coJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();



    private final Drive driveSystem = new Drive(drivetrain);
    private final Intake intakeSystem = new Intake();
    private final Elevator elevatorSystem = new Elevator();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Constants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * Constants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> driveRobotCentric.withVelocityX(-joystick.getLeftY() * Constants.MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * Constants.MaxSpeed)
            .withRotationalRate(-joystick.getRightX() * Constants.MaxAngularRate)
        ));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.x().onTrue(driveSystem.pathRelative(5, 5));
        joystick.y().onTrue(new InstantCommand(()->{
            driveSystem.cancelLastPath();
        }).andThen(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Constants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * Constants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        ));

        // InstantCommand intakeDefault = new InstantCommand(()->{intakeSystem.setIntakePower(coJoystick.getRightY()*Constants.JOYSTICK_CORAL_MULTIPLIER);});
        // InstantCommand intakeFast = new InstantCommand(()->{intakeSystem.setIntakePower(coJoystick.getRightY());});
        // intakeDefault.addRequirements(intakeSystem);
        // intakeFast.addRequirements(intakeSystem);


        // intakeSystem.setDefaultCommand(intakeDefault);
        // coJoystick.leftBumper().whileTrue(intakeFast);

        // elevatorSystem.setDefaultCommand(new InstantCommand(() -> {
        //     elevatorSystem.setSpeed(coJoystick.getRightTriggerAxis()-coJoystick.getLeftTriggerAxis());
        // }));
        
        // coJoystick.b().whileTrue(new InstantCommand(() -> {
        //     elevatorSystem.setSpeedNoLimit(coJoystick.getRightTriggerAxis()-coJoystick.getLeftTriggerAxis());
        // }));
        // DELET LATER

        // Assuming intakeSystem and coJoystick are already defined and initialized

        // Define commands for the intake system with addRequirements
        InstantCommand intakeDefault = new InstantCommand(() -> {
            intakeSystem.setIntakePower(coJoystick.getRightY() * Constants.JOYSTICK_CORAL_MULTIPLIER);
        });
        InstantCommand intakeFast = new InstantCommand(() -> {
            intakeSystem.setIntakePower(coJoystick.getRightY());
        });
        intakeDefault.addRequirements(intakeSystem);
        intakeFast.addRequirements(intakeSystem);

        intakeSystem.setDefaultCommand(intakeDefault);
        coJoystick.leftBumper().whileTrue(intakeFast);

        // Define commands for the elevator system with addRequirements
        InstantCommand elevatorDefault = new InstantCommand(() -> {
            elevatorSystem.setSpeed(coJoystick.getRightTriggerAxis() - coJoystick.getLeftTriggerAxis());
        });
        InstantCommand elevatorNoLimit = new InstantCommand(() -> {
            elevatorSystem.setSpeedNoLimit(coJoystick.getRightTriggerAxis() - coJoystick.getLeftTriggerAxis());
        });
        elevatorDefault.addRequirements(elevatorSystem);
        elevatorNoLimit.addRequirements(elevatorSystem);

        elevatorSystem.setDefaultCommand(elevatorDefault);
        coJoystick.b().whileTrue(elevatorNoLimit);


       

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
