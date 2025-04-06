package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.generated.Constants;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
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

public class Drive implements Subsystem {
    private CommandSwerveDrivetrain drivetrain;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private Command lastPath;

    private SwerveModule[] modules = new SwerveModule[4];
    private final Field2d field = new Field2d();
    private final Pigeon2 gyro = new Pigeon2(Constants.pigeonId);

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(Constants.MaxSpeed * 0.1)
            .withRotationalDeadband(Constants.MaxAngularRate * 0.1);
    
    public Drive(CommandSwerveDrivetrain drivetrain) {
        modules = drivetrain.getModules();
        kinematics = drivetrain.getKinematics();
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
        this.drivetrain = drivetrain;
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

    public void periodic(){

        odometry.update(gyro.getRotation2d(), getPositions());

        field.setRobotPose(getPose());
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

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        double targetPowerX = targetSpeeds.vxMetersPerSecond/Constants.MaxSpeed;
        double targetPowerY = targetSpeeds.vyMetersPerSecond/Constants.MaxSpeed;
        double targetRotationalPower = targetSpeeds.omegaRadiansPerSecond;

        drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(targetPowerX)
        .withVelocityY(targetPowerY)
        .withRotationalRate(targetRotationalPower))
        .schedule();
    }

    public void cancelLastPath() {
        if (lastPath != null) lastPath.cancel();
    }

    public Command pathRelative(double targetX, double targetY) {
        Pose2d currentPose = getPose();
        Rotation2d targetRotation = currentPose.getRotation();

        Translation2d localOffset = new Translation2d(targetX, targetY);
        Translation2d fieldOffset = localOffset.rotateBy(currentPose.getRotation());
        
        Pose2d startPose = new Pose2d(
            currentPose.getTranslation(),
            currentPose.getRotation()
        );

        Rotation2d endHeading = currentPose.getRotation(); // Todo: add offset feature

        Pose2d endPose = new Pose2d(
            currentPose.getTranslation().plus(fieldOffset),
            endHeading
        );
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(
                4.0, 
                4.0,
                Units.degreesToRadians(360), 
                Units.degreesToRadians(540)
            ),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, endHeading) // Final heading matches endPos heading
        );

        path.preventFlipping = true;

        cancelLastPath();
        lastPath = AutoBuilder.followPath(path);
        
        return lastPath;

    }
}
