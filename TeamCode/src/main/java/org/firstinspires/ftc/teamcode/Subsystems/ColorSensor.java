package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensor {

    public enum DetectedColor {
        NONE,
        RED,
        BLUE,
        YELLOW
    }

    // Bitmask value reported for each detected color -- each is its own bit.
    public static final int VALUE_NONE = 0;   // 000
    public static final int VALUE_RED = 1;    // 001
    public static final int VALUE_BLUE = 2;   // 010
    public static final int VALUE_YELLOW = 4; // 100

    // Hue ranges -- tune to your field lighting / game pieces.
    private static final float RED_HUE_MIN_LOW = 0f;
    private static final float RED_HUE_MAX_LOW = 15f;
    private static final float RED_HUE_MIN_HIGH = 345f;
    private static final float RED_HUE_MAX_HIGH = 360f;

    private static final float BLUE_HUE_MIN = 190f;
    private static final float BLUE_HUE_MAX = 250f;

    private static final float YELLOW_HUE_MIN = 40f;
    private static final float YELLOW_HUE_MAX = 65f;

    private static final float MIN_SATURATION = 0.35f;
    private static final float MIN_VALUE = 0.05f;

    private final NormalizedColorSensor colorSensor;
    private final float[] hsvValues = new float[3];

    private DetectedColor currentColor = DetectedColor.NONE;
    private DetectedColor lastColor = DetectedColor.NONE;
    private int value = VALUE_NONE;

    // True only on the loop iteration where a ball first appears (transition
    // from NONE -> a color). Lets you trigger the hood/flywheel exactly once
    // per ball instead of every loop while the ball sits in view.
    private boolean newDetection = false;

    /**
     * @param hardwareMap the OpMode's hardwareMap
     * @param sensorName  name configured in the Robot Controller config file
     */
    public ColorSensor(HardwareMap hardwareMap, String sensorName) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, sensorName);
        colorSensor.setGain(2.0f);
    }

    /** Call this once per loop iteration to refresh the reading and the value. */
    public void update() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float brightness = hsvValues[2];

        DetectedColor detected = DetectedColor.NONE;

        if (saturation >= MIN_SATURATION && brightness >= MIN_VALUE) {
            boolean isRed = (hue >= RED_HUE_MIN_LOW && hue <= RED_HUE_MAX_LOW)
                    || (hue >= RED_HUE_MIN_HIGH && hue <= RED_HUE_MAX_HIGH);

            if (isRed) {
                detected = DetectedColor.RED;
            } else if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) {
                detected = DetectedColor.BLUE;
            } else if (hue >= YELLOW_HUE_MIN && hue <= YELLOW_HUE_MAX) {
                detected = DetectedColor.YELLOW;
            }
        }

        // A "new" ball is one where we just transitioned from seeing nothing
        // to seeing a color. Once the ball clears (back to NONE), the sensor
        // is ready to flag the next one.
        newDetection = (detected != DetectedColor.NONE && currentColor == DetectedColor.NONE);

        lastColor = currentColor;
        currentColor = detected;
        value = colorToValue(detected);
    }

    private int colorToValue(DetectedColor color) {
        switch (color) {
            case RED:
                return VALUE_RED;
            case BLUE:
                return VALUE_BLUE;
            case YELLOW:
                return VALUE_YELLOW;
            case NONE:
            default:
                return VALUE_NONE;
        }
    }

    public DetectedColor getDetectedColor() {
        return currentColor;
    }

    /** Raw hue (0-360) from the last update() -- useful for tuning the hue ranges. */
    public float getHue() {
        return hsvValues[0];
    }

    /** Raw saturation (0-1) from the last update(). */
    public float getSaturation() {
        return hsvValues[1];
    }

    /** Raw brightness/value (0-1) from the last update(). */
    public float getBrightness() {
        return hsvValues[2];
    }

    /** The current numeric value for whatever color this sensor sees right now. */
    public int getValue() {
        return value;
    }

    /**
     * True only on the single loop iteration where a new ball is first seen
     * (the sensor was seeing NONE, now sees a color). Use this to trigger a
     * hood/flywheel update exactly once per ball:
     *
     *   sensor.update();
     *   if (sensor.isNewDetection()) {
     *       hood.setForColor(sensor.getDetectedColor());
     *   }
     */
    public boolean isNewDetection() {
        return newDetection;
    }
}
