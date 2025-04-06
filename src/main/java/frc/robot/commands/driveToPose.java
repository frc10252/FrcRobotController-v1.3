package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class driveToPose extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentric drive;

    public driveToPose(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
        this.drivetrain = drivetrain;
        this.drive = drive;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
    }


    
}
