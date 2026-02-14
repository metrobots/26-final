package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

    /* =====================
     * CONFIG
     * ===================== */
    private static final int PWM_PORT = 0; // CHANGE THIS
    private static final int LED_COUNT = 150;
    private static final int MAX_BRIGHTNESS = 128; // cap current draw

    /* =====================
     * HARDWARE
     * ===================== */
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    /* =====================
     * STATE
     * ===================== */
    public enum Pattern {
        OFF,
        SOLID,
        RAINBOW,
        CYLON,
        COLOR_CHASE
    }

    private Pattern currentPattern = Pattern.OFF;
    private Color solidColor = Color.kBlack;

    // Animation state
    private int chaseIndex = 0;
    private int cylonIndex = 0;
    private int cylonDirection = 1;
    private int rainbowHue = 0;

    public Lights() {
        led = new AddressableLED(PWM_PORT);
        buffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        clear();
    }

    /* =====================
     * PERIODIC
     * ===================== */
    @Override
    public void periodic() {
        switch (currentPattern) {
            case SOLID:
                applySolid();
                break;

            case RAINBOW:
                applyRainbow();
                break;

            case CYLON:
                applyCylon();
                break;

            case COLOR_CHASE:
                applyColorChase();
                break;

            case OFF:
            default:
                // do nothing
                break;
        }
    }

    /* =====================
     * PUBLIC API (COMMANDS USE THESE)
     * ===================== */

    public void setSolid(Color color) {
        solidColor = color;
        currentPattern = Pattern.SOLID;
    }

    public void setPattern(Pattern pattern) {
        currentPattern = pattern;
    }

    public void off() {
        currentPattern = Pattern.OFF;
        clear();
    }

    /* =====================
     * PATTERN IMPLEMENTATIONS
     * ===================== */

    private void applySolid() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, scaleBrightness(solidColor));
        }
        led.setData(buffer);
    }

    // FastLED-style rainbow
    private void applyRainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (rainbowHue + (i * 180 / buffer.getLength())) % 180;
            buffer.setHSV(i, hue, 255, MAX_BRIGHTNESS);
        }
        rainbowHue = (rainbowHue + 3) % 180;
        led.setData(buffer);
    }

    // Cylon / Knight Rider sweep
    private void applyCylon() {
        buffer.setLED(cylonIndex, Color.kBlack);

        cylonIndex += cylonDirection;
        if (cylonIndex <= 0 || cylonIndex >= buffer.getLength() - 1) {
            cylonDirection *= -1;
        }

        buffer.setLED(cylonIndex, solidColor);
        led.setData(buffer);
    }

    // Color chase with multi-LED block (FastLED inspired)
    private void applyColorChase() {
        clearBuffer();

        for (int i = 0; i < 6; i++) {
            int index = chaseIndex + i;
            if (index < buffer.getLength()) {
                buffer.setLED(index, solidColor);
            }
        }

        chaseIndex++;
        if (chaseIndex >= buffer.getLength() - 6) {
            chaseIndex = 0;
        }

        led.setData(buffer);
    }

    /* =====================
     * HELPERS
     * ===================== */

    private void clear() {
        clearBuffer();
        led.setData(buffer);
    }

    private void clearBuffer() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }

    private Color scaleBrightness(Color color) {
        return new Color(
            color.red * MAX_BRIGHTNESS / 255.0,
            color.green * MAX_BRIGHTNESS / 255.0,
            color.blue * MAX_BRIGHTNESS / 255.0
        );
    }
}
