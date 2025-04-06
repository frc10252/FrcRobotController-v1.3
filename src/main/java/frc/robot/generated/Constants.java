package frc.robot.generated;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;

import static edu.wpi.first.units.Units.*;


public class Constants {
    public static final int pigeonId = 15;

    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0); //original p 5
    public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0); //original p 5
}
