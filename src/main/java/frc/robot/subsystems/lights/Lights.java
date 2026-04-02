package frc.robot.subsystems.lights;

import java.util.ArrayList;
import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Lights extends SubsystemBase {

    /* =====================
     * CONFIG
     * ===================== */
    private static final int PWM_PORT = 0; // CHANGE THIS
    private static final int LED_COUNT = 95;


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
        COLOR_CHASE,
        CHASE_3324,
        METROBOTS_SCOLL,
        INPUT_SCROLL,
        SNAKE
    }

    private Pattern currentPattern = Pattern.SNAKE;
    private Color solidColor = Color.kBlue;
    AddressableLEDBufferView m_left;

    // Snake Variables
    private int snakeScore = 0;
    ArrayList<Integer> snakex = new ArrayList<Integer>();
    ArrayList<Integer> snakey = new ArrayList<Integer>();
    private int applex = 20;
    private int appley = 1;
    private int snakeDirection = 1; // 1 = Right, 2 = Left, 3 = Up, 4 = Down


    // Animation state
    private int chaseIndex = 33;
    private int cylonIndex = 0;
    private int cylonDirection = 1;
    private int rainbowHue = 0;

    enum RobotStatus {
        EXAMPLE (Color.kBlue),
        EMPTY (Color.kBlack); // No active status
        
        public final Color stateColor;

        private RobotStatus (Color stateColor) {
            this.stateColor = stateColor;
        }

        public Color getColor () {
            return this.stateColor;
        }
    }

    public RobotStatus status1 = RobotStatus.EMPTY;
    public RobotStatus status2 = RobotStatus.EMPTY;
    public RobotStatus status3 = RobotStatus.EMPTY;
    public RobotStatus status4 = RobotStatus.EMPTY;


    public Lights() {

        led = new AddressableLED(PWM_PORT);
        buffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        m_left = buffer.createView(0, 45);
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

            case CHASE_3324:
                apply3324Chase();
                break;

            case METROBOTS_SCOLL:
                brailleScroll("3324 Metrobots", solidColor);
                break;
            
            case SNAKE:
                snake();

            case OFF:
                // do nothing
                break;
        }
        applyStatusLights();
        led.setData(buffer);
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

    private void apply3324Chase() {
        clearBuffer();

        for (int i = 0; i < 14; i++) {
            int index = chaseIndex + i;
            if (index < buffer.getLength()) {
                buffer.createView(index, index + 2);
                buffer.createView(index + 4, index + 6);
                buffer.createView(index + 8, index + 9);
                buffer.createView(index + 11, index + 14);
            }
        }

        chaseIndex++;
        if (chaseIndex >= buffer.getLength() - 14) {
            chaseIndex = 0;
        }

        led.setData(buffer);
    }

    private void brailleScroll (String textInput, Color color) {
        clearBuffer();
        char[] charList = textInput.toLowerCase().toCharArray();
        int spaceCount = 0;
        for (int i = 0; i < textInput.length(); i++) {
            if (charList[i] != ' ') {
                brailleCharacter(chaseIndex + (i*4) + spaceCount, charList[i], color);
            } else {spaceCount++;}
            if (chaseIndex + (i*4) + spaceCount > 33) {
                i = textInput.length() + 1;
            }
        }
        chaseIndex--;
    }

    private void setStatusLight (RobotStatus desiredState, int lightSlot) {
        if (status1 == RobotStatus.EMPTY && lightSlot > 1) {
            lightSlot = 1;
        } else if (status2 == RobotStatus.EMPTY && lightSlot > 2) {
            lightSlot = 2;
        } else if (status3 == RobotStatus.EMPTY && lightSlot > 3) {
            lightSlot = 3;
        }
        switch (lightSlot) {
            case 1:
                status1 = desiredState;
                break; 
            case 2:
                status2 = desiredState;
                break;
            case 3:
                status3 = desiredState;
                break;
            case 4:
                status4 = desiredState;
                break;
        }
    }

    private void applyStatusLights () {
        if (status1.getColor() != Color.kBlack) {
            createRectangle(0, 2, status1.getColor());
            createRectangle(30, 32, status1.getColor());
        }
        if (status2.getColor() != Color.kBlack) {
            createRectangle(3, 5, status2.getColor());
            createRectangle(27, 29, status2.getColor());
        }
        if (status3.getColor() != Color.kBlack) {
            createRectangle(6, 8, status3.getColor());
            createRectangle(24, 26, status3.getColor());
        }
        if (status4.getColor() != Color.kBlack) {
            createRectangle(9, 11, status4.getColor());
            createRectangle(21, 23, status4.getColor());
        }
    }

    public void steerSnake (int direction) {
        switch (direction) {
            case 0:
                snakeScore = 0;
                snakeDirection = direction;
                break;
            case 1:
                if (snakeDirection != 2) {
                    snakeDirection = 1;
                }
                break;
            case 2:
                if (snakeDirection != 1) {
                    snakeDirection = 2;
                }
                break;
            case 3:
                if (snakeDirection != 4) {
                    snakeDirection = 3;
                }
                break;
            case 4:
                if (snakeDirection != 3) {
                    snakeDirection = 4;
                }
                break;
        }
    }
    
    private void snake () {
        Random rand = new Random(); 
        if (snakeScore == 96) {
            snakeDirection = 0;
        }
        switch (snakeDirection) {
            case 0:
                snakex.clear();
                snakey.clear();
                snakex.add(5);
                snakey.add(1);
                applex = 20;
                appley = 1;
                break;
            case 1:
                    snakex.add(snakex.get(snakex.size() - 1) + 1);
                    snakey.add(snakey.get(snakey.size() - 1));
                break;
            case 2:
                snakex.add(snakex.get(snakex.size() - 1) - 1);
                snakey.add(snakey.get(snakey.size() - 1));
                break;
            case 3:
                snakex.add(snakex.get(snakex.size() - 1));
                snakey.add(snakey.get(snakey.size() - 1) + 1);
                break;
            case 4:
                snakex.add(snakex.get(snakex.size() - 1));
                snakey.add(snakey.get(snakey.size() - 1) - 1);
                break;
        }
        if (snakex.get(snakex.size() - 1) == applex && snakey.get(snakey.size() - 1) == appley) {
            snakeScore++;
            applex = rand.nextInt(32);
            appley = rand.nextInt(3);
            for (int i = 0; i < snakex.size(); i++) {
                if (applex == snakex.get(i) && appley == snakey.get(i)) {
                    applex = rand.nextInt(32);
                    appley = rand.nextInt(3);
                    i = -1;
                }
            } 
        }
        for (int i = 0; i < snakex.size(); i++) {
            if (snakex.get(snakex.size() - 1) == snakex.get(i) && snakey.get(snakey.size() - 1) == snakey.get(i)) {
                snakeDirection = 0;
            }
        }
        if (snakeScore+1 > snakex.size()) {
            snakex.remove(0);
            snakey.remove(0);
        }
        for (int i = 0; i < snakex.size(); i++) {
            setCoord(snakex.get(i), snakey.get(i), Color.kGreen);
        }
        setCoord(applex, appley, Color.kRed);
    }

    // Coordinate Based Light Programs

    private void setCoord (int x, int y, Color color) {
        int rowLength = 32; 
        if (y == 0 && x <= rowLength) {
            buffer.setLED(LED_COUNT-x, color);
        } else if (y == 1 && x <= rowLength -2) {
            buffer.setLED(x+rowLength + 1, color);
        } else if (y == 2 && x <= rowLength) {
            buffer.setLED(rowLength-x, color);
        }
    }

    private void createRectangle (int x1, int x2, Color color) {
        int length = x2 - x1;
        for (int i = 0; i < length; i++) {
            setCoord(x1+i, 0, color);
            setCoord(x1+i, 1, color);
            setCoord(x1+i, 2, color);
        }
    }

    private void brailleCharacter (int leadx, char characterInput, Color color) {
        Boolean tl = false;
        Boolean tr = false;
        Boolean cl = false;
        Boolean cr = false;
        Boolean bl = false;
        Boolean br = false;

        switch (characterInput) {
            case 'a':
                tl = true; tr = false;
                cl = false; cr = false;
                bl = false; br = false;
                break;
            case 'b':
                tl = true; tr = false;
                cl = true; cr = false;
                bl = false; br = false;
                break;
            case 'c':
                tl = true; tr = true;
                cl = false; cr = false;
                bl = false; br = false;
                break;
            case 'd':
                tl = true; tr = true;
                cl = false; cr = true;
                bl = false; br = false;
                break;
            case 'e':
                tl = true; tr = false;
                cl = false; cr = true;
                bl = false; br = false;
                break;
            case 'f':
                tl = true; tr = true;
                cl = true; cr = false;
                bl = false; br = false;
                break;
            case 'g':
                tl = true; tr = true;
                cl = true; cr = true;
                bl = false; br = false;
                break;
            case 'h':
                tl = true; tr = false;
                cl = true; cr = true;
                bl = false; br = false;
                break;
            case 'i':
                tl = false; tr = true;
                cl = true; cr = false;
                bl = false; br = false;
                break;
            case 'j':
                tl = false; tr = true;
                cl = true; cr = true;
                bl = false; br = false;
                break;
            case 'k':
                tl = true; tr = false;
                cl = false; cr = false;
                bl = true; br = false;
                break;
            case 'l':
                tl = true; tr = false;
                cl = true; cr = false;
                bl = true; br = false;
                break;
            case 'm':
                tl = true; tr = true;
                cl = false; cr = false;
                bl = true; br = false;
                break;
            case 'n':
                tl = true; tr = true;
                cl = false; cr = true;
                bl = true; br = false;
                break;
            case 'o':
                tl = true; tr = false;
                cl = false; cr = true;
                bl = true; br = false;
                break;
            case 'p':
                tl = true; tr = true;
                cl = true; cr = false;
                bl = true; br = false;
                break;
            case 'q':
                tl = true; tr = true;
                cl = true; cr = true;
                bl = true; br = false;
                break;
            case 'r':
                tl = true; tr = false;
                cl = true; cr = true;
                bl = true; br = false;
                break;
            case 's':
                tl = false; tr = true;
                cl = true; cr = false;
                bl = true; br = false;
                break;
            case 't':
                tl = true; tr = true;
                cl = false; cr = true;
                bl = false; br = false;
                break;
            case 'u':
                tl = true; tr = false;
                cl = false; cr = false;
                bl = true; br = true;
                break;
            case 'v':
                tl = true; tr = false;
                cl = true; cr = false;
                bl = true; br = true;
                break;
            case 'w':
                tl = false; tr = true;
                cl = true; cr = true;
                bl = false; br = true;
                break;
            case 'x':
                tl = true; tr = true;
                cl = false; cr = false;
                bl = true; br = true;
                break;
            case 'y':
                tl = true; tr = true;
                cl = false; cr = true;
                bl = true; br = true;
                break;
            case 'z':
                tl = true; tr = false;
                cl = false; cr = true;
                bl = true; br = true;
                break;
            case '.':
                tl = false; tr = false;
                cl = true; cr = true;
                bl = false; br = true;
                break;
            case ',':
                tl = false; tr = false;
                cl = true; cr = false;
                bl = false; br = false;
                break;
            case '?':
                tl = false; tr = false;
                cl = true; cr = false;
                bl = true; br = true;
                break;
            case '!':
                tl = false; tr = false;
                cl = true; cr = true;
                bl = true; br = false;
                break;
            case '-':
                tl = false; tr = false;
                cl = false; cr = false;
                bl = true; br = true;
                break;
            case '#':
                tl = false; tr = true;
                cl = false; cr = true;
                bl = true; br = true;
                break;
            case '1':
                tl = true; tr = false;
                cl = false; cr = false;
                bl = false; br = false;
                break;
            case '2':
                tl = true; tr = false;
                cl = true; cr = false;
                bl = false; br = false;
                break;
            case '3':
                tl = true; tr = true;
                cl = false; cr = false;
                bl = false; br = false;
                break;
            case '4':
                tl = true; tr = true;
                cl = false; cr = true;
                bl = false; br = false;
                break;
            case '5':
                tl = true; tr = false;
                cl = false; cr = true;
                bl = false; br = false;
                break;
            case '6':
                tl = true; tr = true;
                cl = true; cr = false;
                bl = false; br = false;
                break;
            case '7':
                tl = true; tr = true;
                cl = true; cr = true;
                bl = false; br = false;
                break;
            case '8':
                tl = true; tr = false;
                cl = true; cr = true;
                bl = false; br = false;
                break;
            case '9':
                tl = false; tr = true;
                cl = true; cr = false;
                bl = false; br = false;
                break;
            case '0':
                tl = false; tr = true;
                cl = true; cr = true;
                bl = false; br = false;
                break;
        }

        if (tl) {
            setCoord(leadx, 2, color);
        } if (tr) {
            setCoord(leadx+1, 2, color);
        } if (cl) {
            setCoord(leadx, 1, color);
        } if (cr) {
            setCoord(leadx+1, 1, color);
        } if (bl) {
            setCoord(leadx, 0, color);
        } if (br) {
            setCoord(leadx+1, 0, color);
        }
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
