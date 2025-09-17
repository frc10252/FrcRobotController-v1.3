package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * AprilTagReceiver - Listens for UDP messages from AprilTag detection system
 * and parses them into structured data objects.
 */
public class AprilTagReceiver {
    
    private final int port;
    private DatagramSocket socket;
    private boolean running;
    private Thread listenerThread;
    
    public AprilTagReceiver(int port) {
        this.port = port;
    }
    
    /**
     * Starts listening for AprilTag messages
     * @throws SocketException if unable to bind to the port
     */
    public void start() throws SocketException {
        socket = new DatagramSocket(port);
        running = true;
        
        listenerThread = new Thread(() -> {
            byte[] buffer = new byte[24]; // 6 floats * 4 bytes each = 24 bytes
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
            
            while (running) {
                try {
                    socket.receive(packet);
                    
                    // Parse the received data
                    AprilTagData data = parseData(buffer);
                    
                    // Notify listeners
                    onDataReceived(data);
                } catch (IOException e) {
                    if (running) {
                        System.err.println("Error receiving packet: " + e.getMessage());
                    }
                }
            }
        });
        
        listenerThread.start();
        System.out.println("AprilTagReceiver listening on port " + port);
    }
    
    /**
     * Stops listening for messages
     */
    public void stop() {
        running = false;
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
        if (listenerThread != null) {
            try {
                listenerThread.join(1000); // Wait up to 1 second for thread to finish
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        System.out.println("AprilTagReceiver stopped");
    }
    
    /**
     * Parses the raw byte data into an AprilTagData object
     * @param buffer the raw byte data
     * @return parsed AprilTagData object
     */
    private AprilTagData parseData(byte[] buffer) {
        ByteBuffer bb = ByteBuffer.wrap(buffer);
        bb.order(ByteOrder.LITTLE_ENDIAN); // Python struct.pack uses host byte order, which is typically little-endian
        
        float[] values = new float[6];
        for (int i = 0; i < 6; i++) {
            values[i] = bb.getFloat();
        }
        
        return new AprilTagData(values[0], values[1], values[2], values[3], values[4], values[5]);
    }
    
    /**
     * Override this method to handle received data
     * @param data the parsed AprilTag data
     */
    protected void onDataReceived(AprilTagData data) {
        System.out.println("Received: " + data);
    }
    
    /**
     * AprilTagData - Represents the data sent by the AprilTag detection system
     */
    public static class AprilTagData {
        private final float field1;  // Always 0 in current implementation
        private final float field2;  // Always 0 in current implementation
        private final float field3;  // Always 0 in current implementation
        private final float xOffset;
        private final float zOffset;
        private final float pitchDegrees;
        
        public AprilTagData(float field1, float field2, float field3, 
                           float xOffset, float zOffset, float pitchDegrees) {
            this.field1 = field1;
            this.field2 = field2;
            this.field3 = field3;
            this.xOffset = xOffset;
            this.zOffset = zOffset;
            this.pitchDegrees = pitchDegrees;
        }
        
        // Getters
        public float getField1() { return field1; }
        public float getField2() { return field2; }
        public float getField3() { return field3; }
        public float getXOffset() { return xOffset; }
        public float getZOffset() { return zOffset; }
        public float getPitchDegrees() { return pitchDegrees; }
        
        // Convenience methods
        public boolean isTagDetected() {
            // When no tag is detected, all values except the first three are 0
            return xOffset != 0.0f || zOffset != 0.0f || pitchDegrees != 0.0f;
        }
        
        @Override
        public String toString() {
            if (isTagDetected()) {
                return String.format("AprilTagData{xOffset=%.2f, zOffset=%.2f, pitch=%.2f°}", 
                                   xOffset, zOffset, pitchDegrees);
            } else {
                return "AprilTagData{No tag detected}";
            }
        }
    }
    
    /**
     * Example usage
     */
    public static void main(String[] args) {
        int port = 1234; // Default port from the Python script
        
        AprilTagReceiver receiver = new AprilTagReceiver(port);
        
        // Override the data handling method
        receiver = new AprilTagReceiver(port) {
            @Override
            protected void onDataReceived(AprilTagData data) {
                if (data.isTagDetected()) {
                    System.out.println("Tag detected: X=" + data.getXOffset() + 
                                     "m, Z=" + data.getZOffset() + 
                                     "m, Pitch=" + data.getPitchDegrees() + "°");
                } else {
                    System.out.println("No tag detected");
                }
            }
        };
        
        try {
            receiver.start();
            
            // Keep the program running
            System.out.println("Press Enter to stop...");
            System.in.read();
            
        } catch (SocketException e) {
            System.err.println("Failed to start receiver: " + e.getMessage());
            return;
        } catch (IOException e) {
            // Ignore
        } finally {
            receiver.stop();
        }
    }
}