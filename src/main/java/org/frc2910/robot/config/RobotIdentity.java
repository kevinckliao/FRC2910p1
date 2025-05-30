package org.frc2910.robot.config;

import org.frc2910.robot.Robot;
import org.frc2910.robot.constants.Constants;
import org.frc2910.robot.util.MacAddressUtil;

import static org.frc2910.robot.util.MacAddressUtil.getMACAddress;

public enum RobotIdentity {
    TYPHOON,
    CYCLONE,
    LOKI,
    SPECTRE,
    SIMULATION,
    ;

    public static RobotIdentity getIdentity() {
        if (!Robot.isReal()) {
            return SIMULATION;
        } else {
            String mac = getMACAddress();
            if (!mac.isEmpty()) {
                if (mac.equals(MacAddressUtil.LOKI)) {
                    return LOKI;
                } else if (mac.equals(MacAddressUtil.CYCLONE)) {
                    return CYCLONE;
                } else if (mac.equals(MacAddressUtil.TYPHOON)) {
                    return TYPHOON;
                }
            }

            return SPECTRE;
        }
    }

    public static Constants.Mode getMode() {
        return switch (getIdentity()) {
            case SPECTRE, LOKI, CYCLONE, TYPHOON -> Robot.isReal() ? Constants.Mode.REAL : Constants.Mode.REPLAY;
            case SIMULATION -> Constants.Mode.SIM;
        };
    }
}
