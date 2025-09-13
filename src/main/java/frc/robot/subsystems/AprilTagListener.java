package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagListener extends SubsystemBase {
    private static final int UDP_PORT = 1234;
    private static final int BUFFER_SIZE = 1024;
    private static final int SOCKET_TIMEOUT_MS = 10;
    
    private DatagramSocket socket;
    private Thread udpListenerThread;
    private volatile boolean isRunning = false;
    
    // Thread-safe storage for the latest AprilTag data
    private final AtomicReference<AprilTagData> latestTagData = new AtomicReference<>();
    
    public AprilTagListener() {
        try {
            socket = new DatagramSocket(UDP_PORT);
            socket.setSoTimeout(SOCKET_TIMEOUT_MS);
            startUdpListener();
        } catch (SocketException e) {
            DriverStation.reportError("Failed to create UDP socket on port " + UDP_PORT, e.getStackTrace());
        }
    }
    
    private void startUdpListener() {
        isRunning = true;
        udpListenerThread = new Thread(this::udpListenerLoop, "AprilTagUDPListener");
        udpListenerThread.setDaemon(true);
        udpListenerThread.start();
    }
    
    private void udpListenerLoop() {
        byte[] buffer = new byte[BUFFER_SIZE];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        
        while (isRunning && !Thread.currentThread().isInterrupted()) {
            try {
                socket.receive(packet);
                String data = new String(packet.getData(), 0, packet.getLength());
                parseAprilTagData(data);
            } catch (SocketTimeoutException e) {
                // Timeout is expected, continue listening
            } catch (IOException e) {
                if (isRunning) {
                    DriverStation.reportError("UDP receive error", e.getStackTrace());
                }
            }
        }
    }
    
    private void parseAprilTagData(String data) {
        try {
            // Expected format: 6 float values - "0.0,0.0,0.0,xOffset,zOffset,pitchAngle"
            String[] parts = data.trim().split(",");
            if (parts.length >= 6) {
                // First 3 values are placeholders (0,0,0)
                float placeholder1 = Float.parseFloat(parts[0]);
                float placeholder2 = Float.parseFloat(parts[1]);
                float placeholder3 = Float.parseFloat(parts[2]);
                
                // Meaningful data: x offset, z offset, pitch angle
                float xOffset = Float.parseFloat(parts[3]);
                float zOffset = Float.parseFloat(parts[4]);
                float pitchAngle = Float.parseFloat(parts[5]);
                
                AprilTagData tagData = new AprilTagData(
                    xOffset,
                    zOffset,
                    pitchAngle,
                    System.currentTimeMillis()
                );
                
                latestTagData.set(tagData);
            }
        } catch (NumberFormatException e) {
            DriverStation.reportWarning("Invalid AprilTag data format: " + data, false);
        }
    }
    
    /**
     * Gets the latest AprilTag data received from the Raspberry Pi
     * @return AprilTagData object or null if no data received
     */
    public AprilTagData getLatestTagData() {
        return latestTagData.get();
    }
    
    /**
     * Checks if AprilTag data is fresh (received within the last 500ms)
     * @return true if data is fresh, false otherwise
     */
    public boolean hasRecentData() {
        AprilTagData data = latestTagData.get();
        return data != null && (System.currentTimeMillis() - data.timestamp) < 500;
    }
    
    /**
     * Gets the x offset from the AprilTag
     * @return x offset in meters or 0.0 if no recent data
     */
    public float getXOffset() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.xOffset : 0.0f;
    }
    
    /**
     * Gets the z offset from the AprilTag
     * @return z offset in meters or 0.0 if no recent data
     */
    public float getZOffset() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.zOffset : 0.0f;
    }
    
    /**
     * Gets the pitch angle from the AprilTag
     * @return pitch angle in degrees or 0.0 if no recent data
     */
    public float getPitchAngle() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.pitchAngle : 0.0f;
    }
    
    @Override
    public void close() {
        isRunning = false;
        if (udpListenerThread != null) {
            udpListenerThread.interrupt();
        }
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }
    
    /**
     * Data class to hold AprilTag information
     */
    public static class AprilTagData {
        public final float xOffset;
        public final float zOffset;
        public final float pitchAngle;
        public final long timestamp;
        
        public AprilTagData(float xOffset, float zOffset, float pitchAngle, long timestamp) {
            this.xOffset = xOffset;
            this.zOffset = zOffset;
            this.pitchAngle = pitchAngle;
            this.timestamp = timestamp;
        }
    }
}

package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagListener extends SubsystemBase {
    private static final int UDP_PORT = 1234;
    private static final int BUFFER_SIZE = 1024;
    private static final int SOCKET_TIMEOUT_MS = 10;
    
    private DatagramSocket socket;
    private Thread udpListenerThread;
    private volatile boolean isRunning = false;
    
    // Thread-safe storage for the latest AprilTag data
    private final AtomicReference<AprilTagData> latestTagData = new AtomicReference<>();
    
    public AprilTagListener() {
        try {
            socket = new DatagramSocket(UDP_PORT);
            socket.setSoTimeout(SOCKET_TIMEOUT_MS);
            startUdpListener();
        } catch (SocketException e) {
            DriverStation.reportError("Failed to create UDP socket on port " + UDP_PORT, e.getStackTrace());
        }
    }
    
    private void startUdpListener() {
        isRunning = true;
        udpListenerThread = new Thread(this::udpListenerLoop, "AprilTagUDPListener");
        udpListenerThread.setDaemon(true);
        udpListenerThread.start();
    }
    
    private void udpListenerLoop() {
        byte[] buffer = new byte[BUFFER_SIZE];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        
        while (isRunning && !Thread.currentThread().isInterrupted()) {
            try {
                socket.receive(packet);
                String data = new String(packet.getData(), 0, packet.getLength());
                parseAprilTagData(data);
            } catch (SocketTimeoutException e) {
                // Timeout is expected, continue listening
            } catch (IOException e) {
                if (isRunning) {
                    DriverStation.reportError("UDP receive error", e.getStackTrace());
                }
            }
        }
    }
    
    private void parseAprilTagData(String data) {
        try {
            // Expected format: 6 float values - "0.0,0.0,0.0,xOffset,zOffset,pitchAngle"
            String[] parts = data.trim().split(",");
            if (parts.length >= 6) {
                // First 3 values are placeholders (0,0,0)
                float placeholder1 = Float.parseFloat(parts[0]);
                float placeholder2 = Float.parseFloat(parts[1]);
                float placeholder3 = Float.parseFloat(parts[2]);
                
                // Meaningful data: x offset, z offset, pitch angle
                float xOffset = Float.parseFloat(parts[3]);
                float zOffset = Float.parseFloat(parts[4]);
                float pitchAngle = Float.parseFloat(parts[5]);
                
                AprilTagData tagData = new AprilTagData(
                    xOffset,
                    zOffset,
                    pitchAngle,
                    System.currentTimeMillis()
                );
                
                latestTagData.set(tagData);
            }
        } catch (NumberFormatException e) {
            DriverStation.reportWarning("Invalid AprilTag data format: " + data, false);
        }
    }
    
    /**
     * Gets the latest AprilTag data received from the Raspberry Pi
     * @return AprilTagData object or null if no data received
     */
    public AprilTagData getLatestTagData() {
        return latestTagData.get();
    }
    
    /**
     * Checks if AprilTag data is fresh (received within the last 500ms)
     * @return true if data is fresh, false otherwise
     */
    public boolean hasRecentData() {
        AprilTagData data = latestTagData.get();
        return data != null && (System.currentTimeMillis() - data.timestamp) < 500;
    }
    
    /**
     * Gets the x offset from the AprilTag
     * @return x offset in meters or 0.0 if no recent data
     */
    public float getXOffset() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.xOffset : 0.0f;
    }
    
    /**
     * Gets the z offset from the AprilTag
     * @return z offset in meters or 0.0 if no recent data
     */
    public float getZOffset() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.zOffset : 0.0f;
    }
    
    /**
     * Gets the pitch angle from the AprilTag
     * @return pitch angle in degrees or 0.0 if no recent data
     */
    public float getPitchAngle() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.pitchAngle : 0.0f;
    }
    
    public void close() {
        isRunning = false;
        if (udpListenerThread != null) {
            udpListenerThread.interrupt();
        }
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }
    
    /**
     * Data class to hold AprilTag information
     */
    public static class AprilTagData {
        public final float xOffset;
        public final float zOffset;
        public final float pitchAngle;
        public final long timestamp;
        
        public AprilTagData(float xOffset, float zOffset, float pitchAngle, long timestamp) {
            this.xOffset = xOffset;
            this.zOffset = zOffset;
            this.pitchAngle = pitchAngle;
            this.timestamp = timestamp;
        }
    }
}
