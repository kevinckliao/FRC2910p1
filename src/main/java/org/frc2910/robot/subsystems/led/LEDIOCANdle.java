package org.frc2910.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOCANdle implements LEDIO {

    private static final int COLOR_SCALAR = 255;
    private final CANdle candle;

    public LEDIOCANdle(int port, String canBus) {
        this.candle = new CANdle(port, canBus);
        candle.configLEDType(CANdle.LEDStripType.GRB);
    }

    @Override
    public void setLEDs(Color color) {
        candle.setLEDs((int) (color.red * COLOR_SCALAR), (int) (color.green * COLOR_SCALAR), (int)
                (color.blue * COLOR_SCALAR));
    }

    public void setLEDs(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
    }

    @Override
    public void setAnimation(Animation animation) {
        candle.animate(animation);
    }

    @Override
    public void clearAnimation() {
        candle.clearAnimation(0);
    }
}
