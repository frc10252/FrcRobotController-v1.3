package frc.robot.subsystems;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.util.AprilTagPIDReading;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public class Drive extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private Command lastPath;

    private SwerveModule[] modules = new SwerveModule[4];
    private final Field2d field = new Field2d();
    private final Pigeon2 gyro = new Pigeon2(Constants.pigeonID);

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(Constants.MaxSpeed * 0.1)
            .withRotationalDeadband(Constants.MaxAngularRate * 0.1);

    private final CommandXboxController joystick;

    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
    .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.2) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);;

    private final SwerveRequest.FieldCentricFacingAngle driveFieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.MaxSpeed * 0.1)
            .withRotationalDeadband(Constants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public double targetDrivetrainAngle;
    
    public Drive(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
        this.joystick = joystick;

        modules = drivetrain.getModules();
        kinematics = drivetrain.getKinematics();
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

        this.drivetrain = drivetrain;

        driveFieldCentricFacingAngle.HeadingController.setPID(5,0.1,0.02);
        targetDrivetrainAngle = Constants.imu.getYaw().getValueAsDouble();

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(this::getPose, this::resetPose, this::getSpeeds, this::driveRobotRelative, 
                    new PPHolonomicDriveController(
                        Constants.translationConstants, 
                        Constants.rotationConstants),
                    config, 
                    () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SFIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

    public void setStartingPose() {
        AllianceStationID aid = DriverStation.getRawAllianceStation();
        if (aid.equals(AllianceStationID.Red1)) {
            resetPose(new Pose2d(10.425, 1.881, new Rotation2d(0)));
        } else if (aid.equals(AllianceStationID.Red2)) {
            resetPose(new Pose2d(10.425, 3.956, new Rotation2d(0)));
        } else if (aid.equals(AllianceStationID.Red3)) {
            resetPose(new Pose2d(10.425, 6.169, new Rotation2d(0)));
        } else if (aid.equals(AllianceStationID.Blue1)) {
            resetPose(new Pose2d(7.130, 6.169, new Rotation2d(0)));
        } else if (aid.equals(AllianceStationID.Blue2)) {
            resetPose(new Pose2d(7.130, 3.956, new Rotation2d(0)));
        } else if (aid.equals(AllianceStationID.Blue3)) {
            resetPose(new Pose2d(7.130, 1.881, new Rotation2d(0)));
        }

    }

    public void periodic(){

        odometry.update(gyro.getRotation2d(), getPositions());

        field.setRobotPose(getPose());

        targetDrivetrainAngle -= joystick.getRightX()*0.1;
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
    }


    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition(false);
        }
        return positions;
    }
    
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getCurrentState();
        }
        return states;
    }

    public void setDrivetrainDefaultCommand(CommandXboxController joystick) {
        drivetrain.setDefaultCommand(
            driveFieldCentric(joystick)
            //driveFieldCentricFacingAngle(joystick)
        );
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        double targetPowerX = targetSpeeds.vxMetersPerSecond/Constants.MaxSpeed;
        double targetPowerY = targetSpeeds.vyMetersPerSecond/Constants.MaxSpeed;
        double targetRotationalPower = targetSpeeds.omegaRadiansPerSecond;

        drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(targetPowerX)
        .withVelocityY(targetPowerY)
        .withRotationalRate(-targetRotationalPower))
        //TODO: CHANGE BACK ASAP!!!! also btw pathplanner explodes wo this change
        .schedule();
    }

    public Command driveRobotCentric(CommandXboxController joystick) {
        return drivetrain.applyRequest(() -> driveRobotCentric.withVelocityX(-joystick.getLeftY())
            .withVelocityY(-joystick.getLeftX())
            .withRotationalRate(-joystick.getRightX() * Constants.MaxAngularRate));
    }

    public void resetFacingAngle() {
        targetDrivetrainAngle = Math.toRadians(Constants.imu.getYaw().getValueAsDouble());
    }


    public void cancelLastPath() {
        resetFacingAngle();
        if (lastPath != null) lastPath.cancel();
    }

    public Command driveFieldCentric(CommandXboxController joystick) {
        return drivetrain.applyRequest(() ->
            driveFieldCentric.withVelocityX(joystick.getLeftY()) // Drive forward with negative Y (forward)
                .withVelocityY(joystick.getLeftX()) // Drive left with negative X (left)
                .withRotationalRate(joystick.getRightX() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
    }

    public Command driveFieldCentricFacingAngle(CommandXboxController joystick) {
        return drivetrain.applyRequest(() ->
            driveFieldCentricFacingAngle.withVelocityX(joystick.getLeftY()) // Drive forward with negative Y (forward)
                .withVelocityY(joystick.getLeftX()) // Drive left with negative X (left)
                .withTargetDirection(new Rotation2d(targetDrivetrainAngle))
                );
    }

    /**
     * 
     * @param targetX
     * @param targetY
     * @param targetRotation in radians!!
     * @return
     */
    public Command pathRelative(double targetX, double targetY, double targetRotation) {
        System.out.println("path relative with target: " + targetX + ", " + targetY + ", " + targetRotation);
        System.out.println("current pose: " + getPose());
        
        Pose2d currentPose = getPose();

        Translation2d localOffset = new Translation2d(targetX, targetY);
        Translation2d fieldOffset = localOffset.rotateBy(currentPose.getRotation ());
        
        // Pose2d startPose = new Pose2d(
        //     currentPose.getTranslation(),
        //     currentPose.getRotation()
        // );

        Pose2d startPose = currentPose;

        Rotation2d endHeading = currentPose.getRotation().plus(new Rotation2d(targetRotation));

        Pose2d endPose = new Pose2d(
            currentPose.getTranslation().plus(fieldOffset),
            endHeading
        );
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);
        // PathPlannerPath path = new PathPlannerPath(
        //     waypoints,
        //     new PathConstraints(
        //         0.5, 
        //         0.5,
        //         Units.degreesToRadians(360), 
        //         Units.degreesToRadians(540)
        //     ),
        //     null, // Ideal starting state can be null for on-the-fly paths
        //     new GoalEndState(0.0, endHeading) // Final heading matches endPos heading
        // );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(
                0.5, 
                0.5,
                Units.degreesToRadians(360), 
                Units.degreesToRadians(90)
            ),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, endHeading) // Final heading matches endPos heading
        );

        path.preventFlipping = true;

        cancelLastPath();
        lastPath = AutoBuilder.followPath(path);
        
        return lastPath;

    } 

    public Command driveToPose(Pose2d endPose){
        Pose2d startPose = getPose();
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);
        Rotation2d endHeading = endPose.getRotation();

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(
                0.5, 
                0.5,
                Units.degreesToRadians(0), 
                Units.degreesToRadians(0)
            ),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, endHeading) // Final heading matches endPos heading
        );

        path.preventFlipping = true;

        cancelLastPath();
        lastPath = AutoBuilder.followPath(path);
        
        return lastPath;
    }

    public Command driveToBlueAlgae(int position)
    {
        
        Pose2d[] positionList = new Pose2d[]{
            new Pose2d(2.92, 4.06, new Rotation2d(Math.toDegrees(0))),
            new Pose2d(3.68, 2.66, new Rotation2d(Math.toDegrees(60))),
            new Pose2d(5.28, 2.66, new Rotation2d(Math.toDegrees(120))),
            new Pose2d(6.11, 4.02, new Rotation2d(Math.toDegrees(180))),
            new Pose2d(5.29, 5.35, new Rotation2d(Math.toDegrees(240))),
            new Pose2d(3.7, 5.41, new Rotation2d(Math.toDegrees(300)))
            
        };

        return driveToPose(positionList[position]);
    }

    public Command pathAprilTag(AprilTagPIDReading reading) {
        return pathRelative(reading.getMetersX(), reading.getMetersY(), reading.getTagRotation());
    }
}
