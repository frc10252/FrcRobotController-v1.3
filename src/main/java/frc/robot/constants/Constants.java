package frc.robot.constants;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.*;


public class Constants {
    public static final int pigeonID = 15;
    public static final Pigeon2 imu = new Pigeon2(Constants.pigeonID);

    public static final int ELEVATOR_NEO_CAN_ID_1 = 23;
    public static final int ELEVATOR_NEO_CAN_ID_2 = 22;
    public static final int INTAKE_NEO_WHEEL_CAN_ID = 24; 

    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    public static final PIDConstants translationConstants = new PIDConstants(7, 0.0001, 0.01); //original p 5; Recalibrated for smoothness: (7, 0.0001, 0.01)
    public static final PIDConstants rotationConstants = new PIDConstants(5, 0.0001, 0.045); //original p 5; Recalibrated for smoothness: (5, 0.0001, 0.045)
    //Network
    public static final int IP_ADDRESS_LISTEN_PORT = 1234;
    public static final String LISTEN_IP_ADDRESS = "0.0.0.0";

    //Motors + IDs
    public static SparkMax elevatorNeo1 = new SparkMax(ELEVATOR_NEO_CAN_ID_1, MotorType.kBrushless);
    public static SparkMax elevatorNeo2 = new SparkMax(ELEVATOR_NEO_CAN_ID_2, MotorType.kBrushless);

    public static SparkMax intakeNeoWheel = new SparkMax(INTAKE_NEO_WHEEL_CAN_ID, MotorType.kBrushless);

    public static final double JOYSTICK_YAW_MULTIPLIER = 4;
    public static final double JOYSTICK_ELEVATOR_MULTIPLIER =-0.5;
    public static final double ELEVATOR_SETPOINT_CONSTANT = 0.5;

    public static final double MAX_ELEVATOR_POSITION = -2.6; // 3.2 normal
    public static final double MIN_ELEVATOR_POSITION = 0;

    public static final double ELEVATOR_SPEED_LIMIT_OFFSET = 0.6;
    public static final double ELEVATOR_SPEED_LIMIT_MULTIPLIER = 0.3;
    public static final double ELEVATOR_OFFSET_POS = 0;

    public static final double JOYSTICK_CORAL_MULTIPLIER = 0.2;
}
