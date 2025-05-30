package org.frc2910.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj.util.Color;

/**
 * * LEDIO interface where you update the red, green, and blue LEDIO inputs
 * */
public interface LEDIO {
    /**
     * Sets the LEDs to a red, green, and blue value
     */
    default void setLEDs(Color color) {}

    default void setLEDs(int red, int green, int blue) {}

    default void setAnimation(Animation animation) {}

    default void clearAnimation() {}
}
