package org.frc2910.robot.subsystems.toggles;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import org.frc2910.robot.config.PortConfiguration;

public class TogglesIOHardware implements TogglesIO {
    private final DigitalInput neutralModeSwitch;
    private final DigitalInput brakeButton;

    public TogglesIOHardware(PortConfiguration configuration) {
        neutralModeSwitch = new DigitalInput(configuration.neutralModeSwitchID);
        brakeButton = new DigitalInput(configuration.homeButtonID);
    }

    @Override
    public void updateInputs(TogglesIOInputs inputs) {
        inputs.switchValue = neutralModeSwitch.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        inputs.isHomeButtonPressed = !brakeButton.get();
    }
}
