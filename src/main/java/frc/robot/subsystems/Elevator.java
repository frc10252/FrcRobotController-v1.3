package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorNeo1;
    private final SparkMax elevatorNeo2;
    private double power;
    private double neoOffset;
    public static double elevatorOffset = 0;


    public Elevator() {
        System.out.println("Initializing Elevator subsystem...");
        elevatorNeo1 = Constants.elevatorNeo1;
        elevatorNeo2 = Constants.elevatorNeo2;


        SparkMaxConfig elevatorNeo1Config = new SparkMaxConfig();
        elevatorNeo1Config.inverted(true);

        elevatorNeo1.configure(elevatorNeo1Config, null, null);
    }

    public void setSpeed(double power){
        this.power = power * Constants.JOYSTICK_ELEVATOR_MULTIPLIER;
        double position = getPosition() - elevatorOffset;

        // Limits
        if (this.power < 0 && position <= Constants.MAX_ELEVATOR_POSITION) this.power = 0;
        if (this.power > 0 && position >= Constants.MIN_ELEVATOR_POSITION) this.power = 0;

        // Speed Limiting when near the limits
        if (this.power < 0 && position <= Constants.MAX_ELEVATOR_POSITION + Constants.ELEVATOR_SPEED_LIMIT_OFFSET)
         this.power = Math.max(Constants.JOYSTICK_ELEVATOR_MULTIPLIER*Constants.ELEVATOR_SPEED_LIMIT_MULTIPLIER, this.power);
         
        if (this.power > 0 && position >= Constants.MIN_ELEVATOR_POSITION-Constants.ELEVATOR_SPEED_LIMIT_OFFSET)
         this.power = Math.min(-Constants.JOYSTICK_ELEVATOR_MULTIPLIER*Constants.ELEVATOR_SPEED_LIMIT_MULTIPLIER, this.power);



        elevatorNeo1.set(this.power);
        elevatorNeo2.set(this.power);
    }

    public void setSpeedNoLimit(double power){
        this.power = power*Constants.JOYSTICK_ELEVATOR_MULTIPLIER;
        elevatorNeo1.set(this.power);
        elevatorNeo2.set(this.power);
    }

    public double getPosition() {
        return elevatorNeo2.getEncoder().getPosition();
    }

    public double getVelocity() {
        return elevatorNeo2.getEncoder().getVelocity();
    }

    public double getOffset(){
        return neoOffset;
    }


    public void stop() {
        elevatorNeo1.stopMotor();
        elevatorNeo2.stopMotor();
    }

    public void stopGently(){
        elevatorNeo1.set(0);
        elevatorNeo2.set(0);

    }


}
