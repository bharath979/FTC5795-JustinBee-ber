package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private final Servo widthServo;
    private final Servo angleServo;

    // Hood width for red and blue balls (also the starting width).
    // Servo positions must be between 0 and 1 (negatives are treated as 0). On this
    // mechanism 0 is pushed all the way out, and larger numbers pull it back in.
    public static final double WIDTH = 0.56;
    // Hood width for yellow balls.
    public static final double WIDTH_YELLOW = 0.22;
    // Furthest the width servo may turn: past this the ball is over-compressed.
    public static final double WIDTH_MAX = 0.61;

    private Hood.States state = Hood.States.NECTAR;

    public enum States {
        POLLEN,
        NECTAR,

    }

    public void setState(Hood.States newState) {state = newState;}

    public void update(){
        switch (state){
            case NECTAR:
                widthServo.setPosition(WIDTH);
                angleServo.setPosition(0.15);
                break;
            case POLLEN:
                widthServo.setPosition(WIDTH_YELLOW);
                angleServo.setPosition(0.22);
                break;
        }
    }

    // Angle the hood goes to when the opmode starts, until the Limelight sees a tag and
    // takes over. 1.0 is all the way up, 0.0 is all the way down.
    public static final double ANGLE_START = 0.22;

    // How far the angle servo moves per loop while a dpad button is held.
    private static final double ANGLE_MANUAL_STEP = 0.05;
    private static final double WIDTH_MANUAL_STEP = 0.01;

    // Distance-to-angle table for Limelight auto aim. Distance is the floor distance from the
    // Limelight to the AprilTag in inches; angle is the angle servo position to use there.
    // Farther away = hood higher, closer = hood lower. Between points it blends smoothly;
    // past either end it holds the end value. Placeholders: tune by driving to each distance,
    // dpad up/down until shots land, and writing the angle down here.
    private static final double[] AUTO_DISTANCES = {24, 48, 72, 96, 120};
    private static final double[] AUTO_ANGLES    = {0.40, 0.47, 0.53, 0.59, 0.65};

    // Extra amount added on top of the distance table; dpad up/down changes it during a match.
    private double angleTrim = 0.0;

    // Manual width nudge value; starts at WIDTH and is only changed by adjustYellowWidth().
    private double yellowWidth = WIDTH;

    private double currentWidthPosition = WIDTH;
    private double currentAnglePosition = ANGLE_START;

    /**
     * @param hardwareMap      the OpMode's hardwareMap
     * @param widthServoName   name of the servo that changes hood width/circumference
     * @param angleServoName   name of the servo that changes hood angle
     */
    public Hood(HardwareMap hardwareMap, String widthServoName, String angleServoName) {
        widthServo = hardwareMap.get(Servo.class, widthServoName);
        angleServo = hardwareMap.get(Servo.class, angleServoName);
    }

    /** Moves only the angle servo to the given position, clamped to the servo range. */
    public void setAngle(double anglePosition) {
        currentAnglePosition = Math.max(0.0, Math.min(1.0, anglePosition));
        angleServo.setPosition(currentAnglePosition);
    }

    /** Moves only the width servo to the given position, clamped between 0 and WIDTH_MAX. */
    public void setWidth(double widthPosition) {
        currentWidthPosition = clampWidth(widthPosition);
        widthServo.setPosition(currentWidthPosition);
    }

    private static double clampWidth(double widthPosition) {
        return Math.max(0.0, Math.min(WIDTH_MAX, widthPosition));
    }

    /** Moves the width servo to WIDTH (no movement if it is already there). */
    public void goToWidth() {
        setWidth(WIDTH);
    }

    /** Moves both servos to the starting position (fixed width, start angle) right away. */
    public void goToStart() {
        goToWidth();
        setAngle(ANGLE_START);
    }

    /**
     * Sets the hood width for the detected color: yellow uses WIDTH_YELLOW, red and blue use
     * WIDTH. The angle is left alone; only the Limelight distance (plus dpad trim) sets it.
     */
    public void setForColor(ColorSensor.DetectedColor color) {
        switch (color) {
            case YELLOW:
                setWidth(WIDTH_YELLOW);
                setAngle(0.22);
                break;
            case RED:
            case BLUE:
                setWidth(WIDTH);
                setAngle(0.15);
                break;
            case NONE:
            default:
                break;
        }
    }

    /** Directly set raw servo positions, bypassing the color lookup table. */
    public void setPositions(double widthPosition, double anglePosition) {
        currentWidthPosition = clampWidth(widthPosition);
        currentAnglePosition = anglePosition;
        widthServo.setPosition(currentWidthPosition);
        angleServo.setPosition(anglePosition);
    }

    /**
     * Sets the hood angle from the Limelight distance to the tag (inches): raises the hood as
     * the robot backs up and lowers it as the robot drives closer. Adds the dpad trim on top.
     */
    public void setAngleForDistance(double distanceInches) {
        setAngle(angleForDistance(distanceInches) + angleTrim);
    }

    /** Looks up the hood angle for a distance in AUTO_DISTANCES / AUTO_ANGLES, blending between points. */
    public static double angleForDistance(double distanceInches) {
        int last = AUTO_DISTANCES.length - 1;
        if (distanceInches <= AUTO_DISTANCES[0]) {
            return AUTO_ANGLES[0];
        }
        if (distanceInches >= AUTO_DISTANCES[last]) {
            return AUTO_ANGLES[last];
        }
        for (int i = 0; i < last; i++) {
            if (distanceInches <= AUTO_DISTANCES[i + 1]) {
                double t = (distanceInches - AUTO_DISTANCES[i]) / (AUTO_DISTANCES[i + 1] - AUTO_DISTANCES[i]);
                return AUTO_ANGLES[i] + t * (AUTO_ANGLES[i + 1] - AUTO_ANGLES[i]);
            }
        }
        return AUTO_ANGLES[last];
    }

    public double getAngleTrim() {
        return angleTrim;
    }

    /**
     * Nudges the hood angle up (+1) or down (-1) by one manual step, clamped to the servo range.
     * Also shifts the auto-aim trim so the nudge sticks while the Limelight is controlling the angle.
     */
    public void adjustAngle(int direction) {
        angleTrim += direction * ANGLE_MANUAL_STEP;
        currentAnglePosition = Math.max(0.0, Math.min(1.0, currentAnglePosition + direction * ANGLE_MANUAL_STEP));
        angleServo.setPosition(currentAnglePosition);
    }

    /**
     * Manually nudges the width servo by one step (+1 or -1) for tuning. The next detected
     * color puts it back at WIDTH.
     */
    public void adjustYellowWidth(int direction) {
        yellowWidth = clampWidth(yellowWidth + direction * WIDTH_MANUAL_STEP);
        currentWidthPosition = yellowWidth;
        widthServo.setPosition(yellowWidth);
    }

    public double getYellowWidth() {
        return yellowWidth;
    }

    public double getWidthPosition() {
        return currentWidthPosition;
    }

    public double getAnglePosition() {
        return currentAnglePosition;
    }
}
