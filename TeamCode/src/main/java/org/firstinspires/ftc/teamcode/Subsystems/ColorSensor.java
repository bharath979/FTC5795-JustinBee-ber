package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Detects a single RED, BLUE, or YELLOW ball in front of a REV Color Sensor V3.
 *
 * How it works:
 *   1. Ball present?  Uses the V3's built-in distance sensor (ball closer than
 *      BALL_DISTANCE_CM). If the sensor has no distance reading, falls back to
 *      total brightness (r + g + b > MIN_TOTAL_BRIGHTNESS).
 *   2. Which color?   Compares the red / green / blue channels to EACH OTHER,
 *      so it works no matter how bright or dim the reading is:
 *        BLUE   -> blue is the strongest channel
 *        RED    -> red is strongest and green is much weaker than red
 *        YELLOW -> red and green are both strong, blue is weak
 *   3. Debounce.     A color must be seen for STABLE_LOOPS loops in a row
 *      before it's reported, so one noisy reading can't flip the result.
 *
 * Tuning: call addTelemetry(telemetry) every loop, hold each ball in front of
 * the sensor, and adjust the constants below based on the R/G/B numbers shown.
 */
public class ColorSensor {

    public enum DetectedColor {
        NONE,
        RED,
        BLUE,
        YELLOW
    }

    // ---------------- Tunable constants ----------------

    // Sensor gain. Higher = bigger readings. REV V3 works well around 10-20.
    private static final float GAIN = 15f;

    // A ball counts as "present" when it's closer than this (cm).
    private static final double BALL_DISTANCE_CM = 4.0;

    // Fallback presence check if no distance reading: r + g + b must exceed this.
    private static final float MIN_TOTAL_BRIGHTNESS = 0.15f;

    // RED: green must be less than this fraction of red.
    // YELLOW: green must be at least this fraction of red.
    private static final float GREEN_TO_RED_YELLOW_RATIO = 0.7f;

    // YELLOW: blue must be less than this fraction of the larger of red/green.
    private static final float YELLOW_MAX_BLUE_RATIO = 0.6f;

    // Same color must be seen this many loops in a row before it's reported.
    private static final int STABLE_LOOPS = 3;

    // ---------------------------------------------------

    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor; // null if not supported

    private float red, green, blue;
    private double distanceCm = Double.NaN;

    private DetectedColor rawColor = DetectedColor.NONE;     // this loop's reading
    private DetectedColor currentColor = DetectedColor.NONE; // debounced result
    private DetectedColor candidateColor = DetectedColor.NONE;
    private int candidateCount = 0;
    private boolean newDetection = false;

    /**
     * @param hardwareMap the OpMode's hardwareMap
     * @param sensorName  name configured in the Robot Controller config file
     *                    (configure it as "REV Color Sensor V3")
     */
    public ColorSensor(HardwareMap hardwareMap, String sensorName) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, sensorName);
        colorSensor.setGain(GAIN);

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        distanceSensor = (colorSensor instanceof DistanceSensor)
                ? (DistanceSensor) colorSensor
                : null;
    }

    /** Call once per loop to refresh the reading. */
    public void update() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        red = colors.red;
        green = colors.green;
        blue = colors.blue;

        if (distanceSensor != null) {
            distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
        }

        rawColor = classify();

        // Debounce: only accept a new color after STABLE_LOOPS matching reads.
        if (rawColor == candidateColor) {
            candidateCount++;
        } else {
            candidateColor = rawColor;
            candidateCount = 1;
        }

        DetectedColor previous = currentColor;
        if (candidateCount >= STABLE_LOOPS) {
            currentColor = candidateColor;
        }

        newDetection = (previous == DetectedColor.NONE && currentColor != DetectedColor.NONE);
    }

    private DetectedColor classify() {
        if (!isBallPresent()) {
            return DetectedColor.NONE;
        }

        float max = Math.max(red, Math.max(green, blue));
        if (max <= 0f) {
            return DetectedColor.NONE;
        }

        if (blue == max) {
            return DetectedColor.BLUE;
        }

        float greenToRed = (red > 0f) ? green / red : Float.MAX_VALUE;
        float blueToRedGreen = blue / Math.max(red, green);

        if (red == max && greenToRed < GREEN_TO_RED_YELLOW_RATIO) {
            return DetectedColor.RED;
        }

        if (greenToRed >= GREEN_TO_RED_YELLOW_RATIO && blueToRedGreen < YELLOW_MAX_BLUE_RATIO) {
            return DetectedColor.YELLOW;
        }

        return DetectedColor.NONE;
    }

    private boolean isBallPresent() {
        if (distanceSensor != null && !Double.isNaN(distanceCm)) {
            return distanceCm < BALL_DISTANCE_CM;
        }
        return (red + green + blue) > MIN_TOTAL_BRIGHTNESS;
    }

    /** The debounced color the sensor currently sees. */
    public DetectedColor getDetectedColor() {
        return currentColor;
    }

    /**
     * True only on the single loop where a ball first shows up (NONE -> a color).
     * Use it to trigger the hood/flywheel exactly once per ball.
     */
    public boolean isNewDetection() {
        return newDetection;
    }

    public float getRed() {
        return red;
    }

    public float getGreen() {
        return green;
    }

    public float getBlue() {
        return blue;
    }

    /** Distance to the object in cm, or NaN if the sensor has no distance reading. */
    public double getDistanceCm() {
        return distanceCm;
    }

    /** Adds color + raw readings to telemetry for tuning. Caller still calls telemetry.update(). */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Ball Color", currentColor);
        telemetry.addData("Raw Color (this loop)", rawColor);
        telemetry.addData("R / G / B", "%.3f / %.3f / %.3f", red, green, blue);
        telemetry.addData("Distance (cm)", "%.2f", distanceCm);
    }
}
