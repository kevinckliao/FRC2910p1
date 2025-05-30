package org.frc2910.robot.subsystems.canandcolor;

import org.frc2910.robot.config.PortConfiguration;

public class CanandcolorIOHardware implements CanandcolorIO {

    public CanandcolorIOHardware(PortConfiguration configuration) {}

    @Override
    public void updateInputs(CanandcolorIOInputs inputs) {
        inputs.blue = 0.0;
        inputs.red = 0.0;
    }
}
