// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.AllianceStationID;
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
import frc.robot.subsystems.MessageListener;



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



    private final Drive driveSystem = new Drive(drivetrain, drive, joystick);
    private final Intake intakeSystem = new Intake();
    private final Elevator elevatorSystem = new Elevator();
    private final MessageListener messageListenerSystem = new MessageListener();

    public RobotContainer() {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureBindings() {
        driveSystem.setDrivetrainDefaultCommand(joystick); 

        joystick.rightBumper().whileTrue(driveSystem.driveRobotCentric(joystick));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        if (Robot.isSimulation()) {
            joystick.x().onTrue(driveSystem.pathRelative(1, 1, 0)); // Random test path
        } else {
            joystick.x().onTrue(driveSystem.pathAprilTag(messageListenerSystem.getAprilTagPIDReading()));
        }
        
        joystick.y().onTrue(new InstantCommand(()->{
            driveSystem.cancelLastPath();
        }).andThen(
            driveSystem.driveFieldCentric(joystick)
        ));


        // intakeSystem.setDefaultCommand(intakeDefault);
        // coJoystick.leftBumper().whileTrue(intakeFast);

        // elevatorSystem.setDefaultCommand(new InstantCommand(() -> {
        //     elevatorSystem.setSpeed(coJoystick.getRightTriggerAxis()-coJoystick.getLeftTriggerAxis());
        // }));
        
        // coJoystick.b().whileTrue(new InstantCommand(() -> {
        //     elevatorSystem.setSpeedNoLimit(coJoystick.getRightTriggerAxis()-coJoystick.getLeftTriggerAxis());
        // }));
        // DELET LATER

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

        Constants.imu.reset();
        Constants.imu.setYaw(180);
    }

    public Command getAutonomousCommand() {
        AllianceStationID allianceStationID = DriverStation.getRawAllianceStation();
        boolean simpleauto = false;
        if (simpleauto) {
            return driveSystem.pathRelative(-1, 0, 0);
        } else if (allianceStationID.equals(AllianceStationID.Blue3)) {
            return new PathPlannerAuto("Blue Coral");
        } else {
            return null;
        }
    }

    public Drive getDriveSubsystem(){
        return driveSystem;
    }
}
