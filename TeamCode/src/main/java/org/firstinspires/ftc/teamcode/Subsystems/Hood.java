package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private final Servo widthServo;
    private final Servo angleServo;

    // The hood width always sits at this position, for every ball color.
    // Servo positions must be between 0 and 1 (negatives are treated as 0). On this
    // mechanism 0 is pushed all the way out, and larger numbers pull it back in.
    public static final double WIDTH = 0.56;

    // Angle positions. Red and blue share one angle; yellow gets its own.
    private static final double ANGLE_YELLOW = 0.51;
    // Angle the hood goes to when the opmode starts. 1.0 is all the way up, 0.0 is all the way down.
    public static final double ANGLE_START = 0.51;
    // The angle used for both red and blue balls.
    public static final double ANGLE_RED_BLUE = 0.51;

    // How far the angle servo moves per loop while a dpad button is held.
    private static final double ANGLE_MANUAL_STEP = 0.01;
    private static final double WIDTH_MANUAL_STEP = 0.01;

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

    /** Moves only the width servo to the given position, clamped to the servo range. */
    public void setWidth(double widthPosition) {
        currentWidthPosition = Math.max(0.0, Math.min(1.0, widthPosition));
        widthServo.setPosition(currentWidthPosition);
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
     * Sets the hood angle for the detected color. The width is the same for every color, so it
     * is just held at WIDTH.
     */
    public void setForColor(ColorSensor.DetectedColor color) {
        double angle;

        switch (color) {
            case YELLOW:
                angle = ANGLE_YELLOW;
                break;
            case RED:
            case BLUE:
                angle = ANGLE_RED_BLUE;
                break;
            case NONE:
            default:
                return;
        }

        goToWidth();
        setAngle(angle);
    }

    /** Directly set raw servo positions, bypassing the color lookup table. */
    public void setPositions(double widthPosition, double anglePosition) {
        currentWidthPosition = widthPosition;
        currentAnglePosition = anglePosition;
        widthServo.setPosition(widthPosition);
        angleServo.setPosition(anglePosition);
    }

    /** Nudges the hood angle up (+1) or down (-1) by one manual step, clamped to the servo range. */
    public void adjustAngle(int direction) {
        currentAnglePosition = Math.max(0.0, Math.min(1.0, currentAnglePosition + direction * ANGLE_MANUAL_STEP));
        angleServo.setPosition(currentAnglePosition);
    }

    /**
     * Manually nudges the width servo by one step (+1 or -1) for tuning. The next detected
     * color puts it back at WIDTH.
     */
    public void adjustYellowWidth(int direction) {
        yellowWidth = Math.max(0.0, Math.min(1.0, yellowWidth + direction * WIDTH_MANUAL_STEP));
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
