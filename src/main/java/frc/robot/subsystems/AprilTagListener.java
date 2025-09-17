package frc.robot.subsystems;

import java.io.*;
import java.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

class AprilTagData {
    private float constant1;
    private float constant2;
    private float constant3;
    private float xOffset;
    private float zOffset;
    private float pitchDegrees;

    public AprilTagData(float constant1, float constant2, float constant3,
            float xOffset, float zOffset, float pitchDegrees) {
        this.constant1 = constant1;
        this.constant2 = constant2;
        this.constant3 = constant3;
        this.xOffset = xOffset;
        this.zOffset = zOffset;
        this.pitchDegrees = pitchDegrees;
    }

    // Getters
    public float getConstant1() {
        return constant1;
    }

    public float getConstant2() {
        return constant2;
    }

    public float getConstant3() {
        return constant3;
    }

    public float getXOffset() {
        return xOffset;
    }

    public float getZOffset() {
        return zOffset;
    }

    public float getPitchDegrees() {
        return pitchDegrees;
    }

    // Get all values as an array
    public float[] toArray() {
        return new float[] { constant1, constant2, constant3, xOffset, zOffset, pitchDegrees };
    }

    // Check if data represents no detection (all zeros)
    public boolean isNoDetection() {
        return constant1 == 0.0f && constant2 == 0.0f && constant3 == 0.0f &&
                xOffset == 0.0f && zOffset == 0.0f && pitchDegrees == 0.0f;
    }

    @Override
    public String toString() {
        if (isNoDetection()) {
            return "AprilTagData{No detection}";
        }
        return String.format("AprilTagData{constants=[%.1f, %.1f, %.1f], x=%.2f, z=%.2f, pitch=%.1f°}",
                constant1, constant2, constant3, xOffset, zOffset, pitchDegrees);
    }
}

public class AprilTagListener {
    private static final int PORT = 1234;
    private DatagramSocket socket;
    AprilTagData lastTagData;

    public AprilTagListener(){
        // Bind to the specified port
        try {
            socket = new DatagramSocket(PORT);
        } catch (SocketException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        System.out.println("AprilTagListener listening on port " + PORT);


        Thread listenerThread = new Thread(() -> {
            AprilTagListener listener = null;
            try {
                listener = new AprilTagListener();
    
                System.out.println("Waiting for AprilTag data...");
                while (true) {
                    AprilTagData tagData = listener.receiveTagData();
                    lastTagData = tagData;
                    System.out.println("Received: " + tagData);
                }
            } catch (Exception e) {
                System.err.println("Error: " + e.getMessage());
                e.printStackTrace();
            } finally {
                if (listener != null) {
                    listener.close();
                }
            }
        });
        listenerThread.start();
    }

    public AprilTagData receiveTagData() throws IOException {
        // Buffer to receive 6 floats (4 bytes each) = 24 bytes
        byte[] buffer = new byte[24];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

        // Receive the packet
        socket.receive(packet);

        // Parse the data
        return parseTagData(buffer);
    }

    private AprilTagData parseTagData(byte[] data) {
        // Try little endian first (most common on x86 systems)
        ByteBuffer buffer = ByteBuffer.wrap(data);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        // Extract the 6 float values
        float constant1 = buffer.getFloat();
        float constant2 = buffer.getFloat();
        float constant3 = buffer.getFloat();
        float xOffset = buffer.getFloat();
        float zOffset = buffer.getFloat();
        float pitchDegrees = buffer.getFloat();

        // Validate the data - if we get NaN or infinite values, try big endian
        if (Float.isNaN(constant1) || Float.isInfinite(constant1)) {
            buffer.order(ByteOrder.BIG_ENDIAN);
            buffer.rewind();
            constant1 = buffer.getFloat();
            constant2 = buffer.getFloat();
            constant3 = buffer.getFloat();
            xOffset = buffer.getFloat();
            zOffset = buffer.getFloat();
            pitchDegrees = buffer.getFloat();
        }

        return new AprilTagData(constant1, constant2, constant3, xOffset, zOffset, pitchDegrees);
    }

    public void close() {
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }

    // public static void main(String[] args) {
    // AprilTagListener listener = null;
    // try {
    // listener = new AprilTagListener();

    // System.out.println("Waiting for AprilTag data...");
    // while (true) {
    // AprilTagData tagData = listener.receiveTagData();
    // System.out.println("Received: " + tagData);
    // }
    // } catch (Exception e) {
    // System.err.println("Error: " + e.getMessage());
    // e.printStackTrace();
    // } finally {
    // if (listener != null) {
    // listener.close();
    // }
    // }
    // }

    
}
