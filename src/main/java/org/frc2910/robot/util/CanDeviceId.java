package org.frc2910.robot.util;

public class CanDeviceId {
    private final int deviceNumber;
    private final String bus;

    public CanDeviceId(int deviceNumber, String bus) {
        this.deviceNumber = deviceNumber;
        this.bus = bus;
    }

    // Use the default bus name "rio".
    public CanDeviceId(int deviceNumber) {
        this(deviceNumber, "rio");
    }

    public int getDeviceNumber() {
        return deviceNumber;
    }

    public String getBus() {
        return bus;
    }

    public boolean equals(CanDeviceId other) {
        return other.deviceNumber == deviceNumber && other.bus.equals(bus);
    }

    @Override
    public String toString() {
        return "CanDeviceId(" + deviceNumber + ", " + bus + ")";
    }
}
