package org.frc2910.robot.subsystems.canandcolor;

import org.littletonrobotics.junction.AutoLog;

public interface CanandcolorIO {
    default void updateInputs(CanandcolorIOInputs inputs) {}

    @AutoLog
    class CanandcolorIOInputs {
        public double red = 0;
        public double blue = 0;
    }
}
