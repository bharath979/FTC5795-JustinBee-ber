package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private final Servo widthServo;
    private final Servo angleServo;

    // Placeholder width positions. Red and blue share one value since the
    // balls are the same size; yellow (smaller ball) gets its own.
    private static final double WIDTH_RED_BLUE = 0.50;
    private static final double WIDTH_YELLOW = 0.30;
    private static final double WIDTH_DEFAULT = 0.50; // used when no ball is detected

    // Placeholder angle positions -- one per color, plus a default.
    private static final double ANGLE_RED = 0.40;
    private static final double ANGLE_BLUE = 0.55;
    private static final double ANGLE_YELLOW = 0.65;
    private static final double ANGLE_DEFAULT = 0.50;

    private double currentWidthPosition;
    private double currentAnglePosition;

    /**
     * @param hardwareMap      the OpMode's hardwareMap
     * @param widthServoName   name of the servo that changes hood width/circumference
     * @param angleServoName   name of the servo that changes hood angle
     */
    public Hood(HardwareMap hardwareMap, String widthServoName, String angleServoName) {
        widthServo = hardwareMap.get(Servo.class, widthServoName);
        angleServo = hardwareMap.get(Servo.class, angleServoName);
    }

    /** Moves the hood to the width/angle configured for the given ball color. */
    public void setForColor(ColorSensor.DetectedColor color) {
        double width;
        double angle;

        switch (color) {
            case RED:
                width = WIDTH_RED_BLUE;
                angle = ANGLE_RED;
                break;
            case BLUE:
                width = WIDTH_RED_BLUE;
                angle = ANGLE_BLUE;
                break;
            case YELLOW:
                width = WIDTH_YELLOW;
                angle = ANGLE_YELLOW;
                break;
            case NONE:
            default:
                width = WIDTH_DEFAULT;
                angle = ANGLE_DEFAULT;
                break;
        }

        setPositions(width, angle);
    }

    /** Directly set raw servo positions, bypassing the color lookup table. */
    public void setPositions(double widthPosition, double anglePosition) {
        currentWidthPosition = widthPosition;
        currentAnglePosition = anglePosition;
        widthServo.setPosition(widthPosition);
        angleServo.setPosition(anglePosition);
    }

    public double getWidthPosition() {
        return currentWidthPosition;
    }

    public double getAnglePosition() {
        return currentAnglePosition;
    }
}
