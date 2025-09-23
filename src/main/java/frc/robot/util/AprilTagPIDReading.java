package frc.robot.util;

import java.net.DatagramPacket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import frc.robot.constants.Constants;


public class AprilTagPIDReading {
    private double pidX;
    private double pidZ;
    private double pidYaw;
    private double metersX;
    private double metersY;
    private double tagRotation;
    private double scalingXCoef;
    private double scalingYCoef;
    private double scalingRotationCoef;

    public AprilTagPIDReading(double pidX, double pidZ, double pidYaw, double metersX, double metersY, double tagRotation) {
        this.pidX = pidX;
        this.pidZ = pidZ;
        this.pidYaw = pidYaw;
        this.metersX = metersX;
        this.metersY = metersY;
        this.tagRotation = tagRotation;
        this.scalingXCoef = Constants.AprilTagXScalingCoefficient;
        this.scalingYCoef = Constants.AprilTagYScalingCoefficient;
        this.scalingRotationCoef = Constants.AprilTagRotationCoefficient;
    }

    public static AprilTagPIDReading decompress(DatagramPacket packet) {
        double pidX = 0.0, pidZ = 0.0, pidYaw = 0.0, metersX = 0.0, metersY = 0.0, tagRotation = 0.0;

        ByteBuffer buffer = ByteBuffer.wrap(packet.getData());

        buffer.order(ByteOrder.LITTLE_ENDIAN);

        pidX = buffer.getFloat();
        pidZ = buffer.getFloat();
        pidYaw = buffer.getFloat();
        metersX = buffer.getFloat();
        metersY = buffer.getFloat();
        tagRotation = buffer.getFloat();

        return new AprilTagPIDReading(pidX, pidZ, pidYaw, metersX, metersY, tagRotation);
    }

    public double getPidX() {
        return pidX;
    }

    public double getPidZ() {
        return pidZ;
    }

    public double getPidYaw() {
        return pidYaw;
    }

    public double getMetersX() {
        return metersX*scalingXCoef;
    }

    public double getMetersY() {
        return metersY*scalingYCoef;
    }

    public double getTagRotation() {
        return tagRotation*scalingRotationCoef;
    }

    @Override
    public String toString() {
        return "AprilTagPIDReading [pidX=" + pidX + ", pidZ=" + pidZ + 
               ", pidYaw=" + pidYaw + ", metersX=" + metersX + 
               ", metersY=" + metersY + ", tagRotation=" + tagRotation + "]";
    }
}
