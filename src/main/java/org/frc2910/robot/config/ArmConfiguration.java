package org.frc2910.robot.config;

public class ArmConfiguration {
    public double extensionkP;
    public double extensionkI;
    public double extensionkD;
    public double extensionkS;

    public double shoulderkP;
    public double shoulderkI;
    public double shoulderkD;
    public double shoulderkS;

    public double wristkP;
    public double wristkI;
    public double wristkD;
    public double wristkS;

    public ArmConfiguration withExtensionkP(double extensionkP) {
        this.extensionkP = extensionkP;
        return this;
    }

    public ArmConfiguration withExtensionkI(double extensionkI) {
        this.extensionkI = extensionkI;
        return this;
    }

    public ArmConfiguration withExtensionkD(double extensionkD) {
        this.extensionkD = extensionkD;
        return this;
    }

    public ArmConfiguration withExtensionkS(double extensionkS) {
        this.extensionkS = extensionkS;
        return this;
    }

    public ArmConfiguration withShoulderkP(double shoulderkP) {
        this.shoulderkP = shoulderkP;
        return this;
    }

    public ArmConfiguration withShoulderkI(double shoulderkI) {
        this.shoulderkI = shoulderkI;
        return this;
    }

    public ArmConfiguration withShoulderkD(double shoulderkD) {
        this.shoulderkD = shoulderkD;
        return this;
    }

    public ArmConfiguration withShoulderkS(double shoulderkS) {
        this.shoulderkS = shoulderkS;
        return this;
    }

    public ArmConfiguration withWristkP(double wristkP) {
        this.wristkP = wristkP;
        return this;
    }

    public ArmConfiguration withWristkI(double wristkI) {
        this.wristkI = wristkI;
        return this;
    }

    public ArmConfiguration withWristkD(double wristkD) {
        this.wristkD = wristkD;
        return this;
    }

    public ArmConfiguration withWristkS(double wristkS) {
        this.wristkS = wristkS;
        return this;
    }
}
