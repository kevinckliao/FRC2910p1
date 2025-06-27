package org.frc2910.robot.config;

import org.frc2910.robot.util.CanDeviceId;

public class PortConfiguration {
    public String CANBus;

    public CanDeviceId extensionOneID;
    public CanDeviceId extensionTwoID;
    public CanDeviceId extensionThreeID;

    public CanDeviceId shoulderOneID;
    public CanDeviceId shoulderTwoID;
    public CanDeviceId shoulderThreeID;

    public CanDeviceId wristID;

    public CanDeviceId climberIntakeID;
    public CanDeviceId carriageID;

    public CanDeviceId intakeTopMotorID;
    public CanDeviceId intakeLeftVerticalMotorID;
    public CanDeviceId intakeRightVerticalMotorID;
    public CanDeviceId intakeBackCANRangeID;
    public CanDeviceId intakeFrontCANRangeID;
    public CanDeviceId intakeFrontLeftCANRangeID;
    public CanDeviceId intakeFrontRightCANRangeID;

    public CanDeviceId candleID;

    public int neutralModeSwitchID;
    public int homeButtonID;

    public int canandcolorID;

    public PortConfiguration withCandleID(CanDeviceId candleID) {
        this.candleID = candleID;
        return this;
    }

    public PortConfiguration withCANBus(String CANBus) {
        this.CANBus = CANBus;
        return this;
    }

    public PortConfiguration withIntakeTopMotorID(CanDeviceId intakeTopMotorID) {
        this.intakeTopMotorID = intakeTopMotorID;
        return this;
    }

    public PortConfiguration withLeftIntakeVerticalMotorID(CanDeviceId leftIntakeVerticalMotorID) {
        this.intakeLeftVerticalMotorID = leftIntakeVerticalMotorID;
        return this;
    }

    public PortConfiguration withRightIntakeVerticalMotorID(CanDeviceId rightIntakeVerticalMotorID) {
        this.intakeRightVerticalMotorID = rightIntakeVerticalMotorID;
        return this;
    }

    public PortConfiguration withIntakeBackCANRangeID(CanDeviceId intakeBackCANRangeID) {
        this.intakeBackCANRangeID = intakeBackCANRangeID;
        return this;
    }

    public PortConfiguration withIntakeFrontCANRangeID(CanDeviceId intakeFrontCANRangeID) {
        this.intakeFrontCANRangeID = intakeFrontCANRangeID;
        return this;
    }

    public PortConfiguration withIntakeFrontRightCANRangeID(CanDeviceId intakeFrontRightCANRangeID) {
        this.intakeFrontRightCANRangeID = intakeFrontRightCANRangeID;
        return this;
    }

    public PortConfiguration withIntakeFrontLeftCANRangeID(CanDeviceId intakeFrontLeftCANRangeID) {
        this.intakeFrontLeftCANRangeID = intakeFrontLeftCANRangeID;
        return this;
    }

    public PortConfiguration withClimberIntakeID(CanDeviceId climberID) {
        this.climberIntakeID = climberID;
        return this;
    }

    public PortConfiguration withCarriageID(CanDeviceId carriageID) {
        this.carriageID = carriageID;
        return this;
    }

    public PortConfiguration withExtensionOneID(CanDeviceId extensionOneID) {
        this.extensionOneID = extensionOneID;
        return this;
    }

    public PortConfiguration withExtensionTwoID(CanDeviceId extensionTwoID) {
        this.extensionTwoID = extensionTwoID;
        return this;
    }

    public PortConfiguration withExtensionThreeID(CanDeviceId extensionThreeID) {
        this.extensionThreeID = extensionThreeID;
        return this;
    }

    public PortConfiguration withShoulderOneID(CanDeviceId shoulderOneID) {
        this.shoulderOneID = shoulderOneID;
        return this;
    }

    public PortConfiguration withShoulderTwoID(CanDeviceId shoulderTwoID) {
        this.shoulderTwoID = shoulderTwoID;
        return this;
    }

    public PortConfiguration withWristID(CanDeviceId wristID) {
        this.wristID = wristID;
        return this;
    }

    public PortConfiguration withShoulderThreeID(CanDeviceId shoulderThreeID) {
        this.shoulderThreeID = shoulderThreeID;
        return this;
    }

    public PortConfiguration withNeutralModeSwitchID(int neutralModeSwitchId) {
        this.neutralModeSwitchID = neutralModeSwitchId;
        return this;
    }

    public PortConfiguration withHomeButtonID(int homeButtonID) {
        this.homeButtonID = homeButtonID;
        return this;
    }

    public PortConfiguration withCanandcolorID(int canandcolorID) {
        this.canandcolorID = canandcolorID;
        return this;
    }
}
