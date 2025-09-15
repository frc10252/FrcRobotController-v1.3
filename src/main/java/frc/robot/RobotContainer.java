// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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

import java.util.*;

public class RobotContainer {
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    private final Orchestra orchestra = new Orchestra();

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController coJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final AllianceStationID allianceStationID = DriverStation.getRawAllianceStation();
    private final boolean isRed;

    private final Drive driveSystem;
    private final Intake intakeSystem;
    private final Elevator elevatorSystem;
    private final MessageListener messageListenerSystem;

    public RobotContainer() {
        driveSystem = new Drive(drivetrain, joystick);
        intakeSystem = new Intake();
        elevatorSystem = new Elevator();
        messageListenerSystem = new MessageListener();
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        orchestra.addInstrument(new TalonFX(1));
        orchestra.addInstrument(new TalonFX(2));
        orchestra.addInstrument(new TalonFX(3));
        orchestra.addInstrument(new TalonFX(4));
        orchestra.addInstrument(new TalonFX(5));
        orchestra.addInstrument(new TalonFX(6));
        orchestra.addInstrument(new TalonFX(7));
        orchestra.addInstrument(new TalonFX(8));

        orchestra.loadMusic("rushe.chrp");

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRed = alliance.get() == DriverStation.Alliance.Red;
        } else {
            isRed = true; // Default fallback value
        }
    }

    private void configureBindings() {
        driveSystem.setDrivetrainDefaultCommand(joystick);

        joystick.rightBumper().whileTrue(driveSystem.driveRobotCentric(joystick));
        joystick.rightBumper().onFalse(new InstantCommand(() -> {
            driveSystem.resetFacingAngle();
        }));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        if (Robot.isSimulation()) {
            // joystick.x().onTrue(driveSystem.pathRelative(0,0,1)); // Random test path
            joystick.x().onTrue(
                    new InstantCommand(() -> {
                        driveSystem.pathRelative(1, 1, Math.toRadians(90)).schedule();
                        // driveSystem.driveToPose(new Pose2d(4,4,new Rotation2d(0))).schedule();
                        // driveSystem.driveToBlueAlgae(2).schedule();
                    }, driveSystem));
        } else {
            // joystick.x().onTrue(driveSystem.pathAprilTag(messageListenerSystem.getAprilTagPIDReading()));
            // joystick.x().onTrue(driveSystem.pathRelative(1, 1, 0));
            // joystick.x().onTrue(
                    // new InstantCommand(() -> {
                    //     driveSystem.pathRelative(1, 1, Math.toRadians(90)).schedule();
                    // }, driveSystem));
                    joystick.x().onTrue(
                        new PathPlannerAuto("Test Rotation") // Runs a PathPlanner auto when X is pressed
                    );
        }

        joystick.povDown().onTrue(new InstantCommand(() -> {
            orchestra.play();
        }));
        joystick.povRight().onTrue(new InstantCommand(() -> {
            orchestra.stop();
        }));

        joystick.y().onTrue(driveSystem.runOnce(() -> {
            driveSystem.cancelLastPath();
        }).andThen(
                driveSystem.driveFieldCentric(joystick)
        // driveSystem.driveFieldCentricFacingAngle(joystick)
        ));

        // intakeSystem.setDefaultCommand(intakeDefault);
        // coJoystick.leftBumper().whileTrue(intakeFast);

        // elevatorSystem.setDefaultCommand(new InstantCommand(() -> {
        // elevatorSystem.setSpeed(coJoystick.getRightTriggerAxis()-coJoystick.getLeftTriggerAxis());
        // }));

        // coJoystick.b().whileTrue(new InstantCommand(() -> {
        // elevatorSystem.setSpeedNoLimit(coJoystick.getRightTriggerAxis()-coJoystick.getLeftTriggerAxis());
        // }));
        // DELET LATER

        intakeSystem.setDefaultCommand(intakeSystem.run(() -> {
            intakeSystem.setIntakePower(coJoystick.getRightY() * Constants.JOYSTICK_CORAL_MULTIPLIER);
        }));
        coJoystick.leftBumper().whileTrue(intakeSystem.run(() -> {
            intakeSystem.setIntakePower(coJoystick.getRightY());
        }));

        elevatorSystem.setDefaultCommand(elevatorSystem.run(() -> {
            elevatorSystem.setSpeed(coJoystick.getRightTriggerAxis() - coJoystick.getLeftTriggerAxis());
        }));
        coJoystick.b().whileTrue(elevatorSystem.run(() -> {
            elevatorSystem.setSpeedNoLimit(coJoystick.getRightTriggerAxis() - coJoystick.getLeftTriggerAxis());
        }));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.povUp().onTrue(driveSystem.runOnce(() -> {
            driveSystem.setStartingPose();
        }));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        driveSystem.setStartingPose();

        Constants.imu.reset();

        Constants.imu.setYaw(0);
    }

    public Command getAutonomousCommand() {
        boolean simpleauto = false;
        if (simpleauto) {
            return driveSystem.pathRelative(-1, 0, 0);
        } else if (allianceStationID.equals(AllianceStationID.Blue3)) {
            return new PathPlannerAuto("Blue Coral").andThen(() -> {
                driveSystem.resetFacingAngle();
            });
        } else {
            return null;
        }
    }

    public void resetDriveFacingAngle() {
        driveSystem.resetFacingAngle();
    }

    public Drive getDriveSubsystem() {
        return driveSystem;
    }

    public MessageListener getMessageListener() {
        return messageListenerSystem;
    }
}
