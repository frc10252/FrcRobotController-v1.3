package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.List;
import java.util.ArrayList;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagListener extends SubsystemBase {
    // Configuration constants
    private static final int UDP_PORT = 1234;
    private static final int BUFFER_SIZE = 2048;
    private static final int SOCKET_TIMEOUT_MS = 10;
    private static final int MAX_HISTORY_SIZE = 50;
    private static final double DATA_FRESHNESS_THRESHOLD_MS = 500.0;
    private static final double POSE_ESTIMATION_TIMEOUT_MS = 1000.0;
    private static final int FILTER_WINDOW_SIZE = 5;
    
    // Networking
    private DatagramSocket socket;
    private Thread udpListenerThread;
    private volatile boolean isRunning = false;
    
    // Data storage
    private final AtomicReference<AprilTagData> latestTagData = new AtomicReference<>();
    private final ConcurrentLinkedQueue<AprilTagData> dataHistory = new ConcurrentLinkedQueue<>();
    private final Map<Integer, AprilTagData> tagDataById = new ConcurrentHashMap<>();
    
    // Filtering and smoothing
    private final MedianFilter xOffsetFilter = new MedianFilter(FILTER_WINDOW_SIZE);
    private final MedianFilter zOffsetFilter = new MedianFilter(FILTER_WINDOW_SIZE);
    private final LinearFilter pitchAngleFilter = LinearFilter.movingAverage(FILTER_WINDOW_SIZE);
    
    // Statistics and monitoring
    private long totalPacketsReceived = 0;
    private long totalPacketsDropped = 0;
    private long lastPacketTime = 0;
    private double averageLatency = 0.0;
    private final Timer connectionTimer = new Timer();
    
    // NetworkTables publishers
    private final NetworkTable table;
    private final DoublePublisher xOffsetPub;
    private final DoublePublisher zOffsetPub;
    private final DoublePublisher pitchAnglePub;
    private final DoublePublisher latencyPub;
    private final BooleanPublisher connectedPub;
    private final StringPublisher statusPub;
    
    // Event callbacks
    private final List<Consumer<AprilTagData>> dataCallbacks = new ArrayList<>();
    private final List<Consumer<Boolean>> connectionCallbacks = new ArrayList<>();
    
    public AprilTagListener() {
        // Initialize NetworkTables
        table = NetworkTableInstance.getDefault().getTable("AprilTagListener");
        xOffsetPub = table.getDoubleTopic("xOffset").publish();
        zOffsetPub = table.getDoubleTopic("zOffset").publish();
        pitchAnglePub = table.getDoubleTopic("pitchAngle").publish();
        latencyPub = table.getDoubleTopic("latency").publish();
        connectedPub = table.getBooleanTopic("connected").publish();
        statusPub = table.getStringTopic("status").publish();
        
        connectionTimer.start();
        
        try {
            socket = new DatagramSocket(UDP_PORT);
            socket.setSoTimeout(SOCKET_TIMEOUT_MS);
            startUdpListener();
            statusPub.set("Initialized");
        } catch (SocketException e) {
            DriverStation.reportError("Failed to create UDP socket on port " + UDP_PORT, e.getStackTrace());
            statusPub.set("Socket Error");
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
                long receiveStartTime = System.currentTimeMillis();
                socket.receive(packet);
                long receiveTime = System.currentTimeMillis();
                
                String data = new String(packet.getData(), 0, packet.getLength());
                parseAprilTagData(data, receiveTime, receiveStartTime);
                
                totalPacketsReceived++;
                lastPacketTime = receiveTime;
                
                // Update connection status
                updateConnectionStatus(true);
                
            } catch (SocketTimeoutException e) {
                // Check for connection timeout
                if (System.currentTimeMillis() - lastPacketTime > POSE_ESTIMATION_TIMEOUT_MS) {
                    updateConnectionStatus(false);
                }
            } catch (IOException e) {
                if (isRunning) {
                    DriverStation.reportError("UDP receive error", e.getStackTrace());
                    totalPacketsDropped++;
                    updateConnectionStatus(false);
                }
            }
        }
    }
    
    private void parseAprilTagData(String data, long receiveTime, long startTime) {
        try {
            String[] parts = data.trim().split(",");
            
            // Enhanced format support: "tagId,x,y,z,roll,pitch,yaw,confidence,distance"
            if (parts.length >= 9) {
                int tagId = Integer.parseInt(parts[0]);
                double x = Double.parseDouble(parts[1]);
                double y = Double.parseDouble(parts[2]);
                double z = Double.parseDouble(parts[3]);
                double roll = Double.parseDouble(parts[4]);
                double pitch = Double.parseDouble(parts[5]);
                double yaw = Double.parseDouble(parts[6]);
                double confidence = Double.parseDouble(parts[7]);
                double distance = Double.parseDouble(parts[8]);
                
                // Calculate latency
                double latency = receiveTime - startTime;
                updateAverageLatency(latency);
                
                // Apply filtering
                double filteredX = xOffsetFilter.calculate(x);
                double filteredZ = zOffsetFilter.calculate(z);
                double filteredPitch = pitchAngleFilter.calculate(pitch);
                
                AprilTagData tagData = new AprilTagData(
                    tagId, x, y, z, roll, pitch, yaw,
                    filteredX, filteredZ, filteredPitch,
                    confidence, distance, receiveTime, latency
                );
                
                // Store data
                latestTagData.set(tagData);
                tagDataById.put(tagId, tagData);
                addToHistory(tagData);
                
                // Publish to NetworkTables
                publishToNetworkTables(tagData);
                
                // Trigger callbacks
                triggerDataCallbacks(tagData);
                
            } else if (parts.length >= 6) {
                // Legacy format support
                parseLegacyFormat(parts, receiveTime, startTime);
            }
        } catch (NumberFormatException e) {
            DriverStation.reportWarning("Invalid AprilTag data format: " + data, false);
            totalPacketsDropped++;
        }
    }
    
    private void parseLegacyFormat(String[] parts, long receiveTime, long startTime) {
        try {
            double xOffset = Double.parseDouble(parts[3]);
            double zOffset = Double.parseDouble(parts[4]);
            double pitchAngle = Double.parseDouble(parts[5]);
            
            double latency = receiveTime - startTime;
            updateAverageLatency(latency);
            
            double filteredX = xOffsetFilter.calculate(xOffset);
            double filteredZ = zOffsetFilter.calculate(zOffset);
            double filteredPitch = pitchAngleFilter.calculate(pitchAngle);
            
            AprilTagData tagData = new AprilTagData(
                -1, xOffset, 0, zOffset, 0, pitchAngle, 0,
                filteredX, filteredZ, filteredPitch,
                1.0, Math.sqrt(xOffset*xOffset + zOffset*zOffset), receiveTime, latency
            );
            
            latestTagData.set(tagData);
            addToHistory(tagData);
            publishToNetworkTables(tagData);
            triggerDataCallbacks(tagData);
            
        } catch (NumberFormatException e) {
            throw e; // Re-throw to be caught by parent
        }
    }
    
    // ========== Data Access Methods ==========
    
    public AprilTagData getLatestTagData() {
        return latestTagData.get();
    }
    
    public AprilTagData getTagDataById(int tagId) {
        return tagDataById.get(tagId);
    }
    
    public List<AprilTagData> getDataHistory() {
        return new ArrayList<>(dataHistory);
    }
    
    public boolean hasRecentData() {
        AprilTagData data = latestTagData.get();
        return data != null && (System.currentTimeMillis() - data.timestamp) < DATA_FRESHNESS_THRESHOLD_MS;
    }
    
    public boolean hasRecentDataForTag(int tagId) {
        AprilTagData data = tagDataById.get(tagId);
        return data != null && (System.currentTimeMillis() - data.timestamp) < DATA_FRESHNESS_THRESHOLD_MS;
    }
    
    // ========== Position and Orientation Methods ==========
    
    public double getXOffset() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.filteredX : 0.0;
    }
    
    public double getYOffset() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.y : 0.0;
    }
    
    public double getZOffset() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.filteredZ : 0.0;
    }
    
    public double getPitchAngle() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.filteredPitch : 0.0;
    }
    
    public double getRollAngle() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.roll : 0.0;
    }
    
    public double getYawAngle() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.yaw : 0.0;
    }
    
    public double getDistance() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.distance : 0.0;
    }
    
    public double getConfidence() {
        AprilTagData data = latestTagData.get();
        return data != null ? data.confidence : 0.0;
    }
    
    // ========== Pose Estimation Methods ==========
    
    public Pose2d getRobotPose2d() {
        AprilTagData data = latestTagData.get();
        if (data == null || !hasRecentData()) return null;
        
        return new Pose2d(
            new Translation2d(data.filteredX, data.filteredZ),
            new Rotation2d(Math.toRadians(data.yaw))
        );
    }
    
    public Pose3d getRobotPose3d() {
        AprilTagData data = latestTagData.get();
        if (data == null || !hasRecentData()) return null;
        
        return new Pose3d(
            new Translation3d(data.filteredX, data.y, data.filteredZ),
            new Rotation3d(
                Math.toRadians(data.roll),
                Math.toRadians(data.filteredPitch),
                Math.toRadians(data.yaw)
            )
        );
    }
    
    public Transform3d getTransformToTag() {
        AprilTagData data = latestTagData.get();
        if (data == null || !hasRecentData()) return null;
        
        return new Transform3d(
            new Translation3d(data.x, data.y, data.z),
            new Rotation3d(
                Math.toRadians(data.roll),
                Math.toRadians(data.pitch),
                Math.toRadians(data.yaw)
            )
        );
    }
    
    // ========== Statistics and Monitoring ==========
    
    public long getTotalPacketsReceived() {
        return totalPacketsReceived;
    }
    
    public long getTotalPacketsDropped() {
        return totalPacketsDropped;
    }
    
    public double getPacketLossRate() {
        long total = totalPacketsReceived + totalPacketsDropped;
        return total > 0 ? (double) totalPacketsDropped / total : 0.0;
    }
    
    public double getAverageLatency() {
        return averageLatency;
    }
    
    public boolean isConnected() {
        return hasRecentData();
    }
    
    public double getConnectionUptime() {
        return connectionTimer.get();
    }
    
    // ========== Event Callbacks ==========
    
    public void addDataCallback(Consumer<AprilTagData> callback) {
        dataCallbacks.add(callback);
    }
    
    public void addConnectionCallback(Consumer<Boolean> callback) {
        connectionCallbacks.add(callback);
    }
    
    public void removeDataCallback(Consumer<AprilTagData> callback) {
        dataCallbacks.remove(callback);
    }
    
    public void removeConnectionCallback(Consumer<Boolean> callback) {
        connectionCallbacks.remove(callback);
    }
    
    // ========== Utility Methods ==========
    
    public void resetStatistics() {
        totalPacketsReceived = 0;
        totalPacketsDropped = 0;
        averageLatency = 0.0;
        connectionTimer.reset();
        connectionTimer.start();
    }
    
    public void clearHistory() {
        dataHistory.clear();
        tagDataById.clear();
    }
    
    // ========== Private Helper Methods ==========
    
    private void addToHistory(AprilTagData data) {
        dataHistory.offer(data);
        while (dataHistory.size() > MAX_HISTORY_SIZE) {
            dataHistory.poll();
        }
    }
    
    private void updateAverageLatency(double newLatency) {
        if (totalPacketsReceived == 1) {
            averageLatency = newLatency;
        } else {
            averageLatency = (averageLatency * 0.9) + (newLatency * 0.1);
        }
    }
    
    private void updateConnectionStatus(boolean connected) {
        connectedPub.set(connected);
        if (connected) {
            statusPub.set("Connected");
        } else {
            statusPub.set("Disconnected");
        }
        
        // Trigger connection callbacks
        for (Consumer<Boolean> callback : connectionCallbacks) {
            try {
                callback.accept(connected);
            } catch (Exception e) {
                DriverStation.reportError("Error in connection callback", e.getStackTrace());
            }
        }
    }
    
    private void publishToNetworkTables(AprilTagData data) {
        xOffsetPub.set(data.filteredX);
        zOffsetPub.set(data.filteredZ);
        pitchAnglePub.set(data.filteredPitch);
        latencyPub.set(data.latency);
    }
    
    private void triggerDataCallbacks(AprilTagData data) {
        for (Consumer<AprilTagData> callback : dataCallbacks) {
            try {
                callback.accept(data);
            } catch (Exception e) {
                DriverStation.reportError("Error in data callback", e.getStackTrace());
            }
        }
    }
    
    @Override
    public void periodic() {
        // Update SmartDashboard with key metrics
        SmartDashboard.putBoolean("AprilTag/Connected", isConnected());
        SmartDashboard.putNumber("AprilTag/PacketsReceived", totalPacketsReceived);
        SmartDashboard.putNumber("AprilTag/PacketLoss%", getPacketLossRate() * 100);
        SmartDashboard.putNumber("AprilTag/AvgLatency", averageLatency);
        
        AprilTagData data = latestTagData.get();
        if (data != null) {
            SmartDashboard.putNumber("AprilTag/Distance", data.distance);
            SmartDashboard.putNumber("AprilTag/Confidence", data.confidence);
            SmartDashboard.putNumber("AprilTag/TagID", data.tagId);
        }
    }
    
    public void close() {
        isRunning = false;
        if (udpListenerThread != null) {
            udpListenerThread.interrupt();
        }
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
        
        // Close NetworkTables publishers
        xOffsetPub.close();
        zOffsetPub.close();
        pitchAnglePub.close();
        latencyPub.close();
        connectedPub.close();
        statusPub.close();
        
        statusPub.set("Closed");
    }
    
    /**
     * Enhanced data class to hold comprehensive AprilTag information
     */
    public static class AprilTagData {
        // Raw data from vision system
        public final int tagId;
        public final double x, y, z;           // Position in 3D space
        public final double roll, pitch, yaw;  // Orientation angles
        
        // Filtered/processed data
        public final double filteredX, filteredZ, filteredPitch;
        
        // Metadata
        public final double confidence;        // Detection confidence (0-1)
        public final double distance;          // Distance to tag
        public final long timestamp;           // When data was received
        public final double latency;           // Network latency in ms
        
        public AprilTagData(int tagId, double x, double y, double z, 
                           double roll, double pitch, double yaw,
                           double filteredX, double filteredZ, double filteredPitch,
                           double confidence, double distance, long timestamp, double latency) {
            this.tagId = tagId;
            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            this.filteredX = filteredX;
            this.filteredZ = filteredZ;
            this.filteredPitch = filteredPitch;
            this.confidence = confidence;
            this.distance = distance;
            this.timestamp = timestamp;
            this.latency = latency;
        }
        
        // Legacy compatibility methods
        public float getXOffset() { return (float) filteredX; }
        public float getZOffset() { return (float) filteredZ; }
        public float getPitchAngle() { return (float) filteredPitch; }
        
        public boolean isHighConfidence() {
            return confidence > 0.8;
        }
        
        public boolean isCloseRange() {
            return distance < 2.0; // Less than 2 meters
        }
        
        public double getAge() {
            return System.currentTimeMillis() - timestamp;
        }
        
        @Override
        public String toString() {
            return String.format("AprilTag[ID=%d, pos=(%.2f,%.2f,%.2f), conf=%.2f, dist=%.2f]",
                tagId, x, y, z, confidence, distance);
        }
    }
}
