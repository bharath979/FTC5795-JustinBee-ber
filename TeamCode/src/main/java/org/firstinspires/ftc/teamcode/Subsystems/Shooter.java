package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
public class Shooter {

    // PIDF coefficients -- placeholders, not tuned yet.
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;

    // Placeholder target velocities in ticks/second, per ball color.
    private static final double VELOCITY_RED_BLUE = 1800.0; // slower
    private static final double VELOCITY_YELLOW = 2400.0;   // faster

    private final DcMotorEx motor;

    // The velocity to spin up to next time spin() is called, based on the
    // most recently detected ball color.
    private double queuedVelocity = VELOCITY_RED_BLUE;

    // The velocity currently commanded to the motor (0 while stopped).
    private double targetVelocity = 0.0;

    /**
     * @param hardwareMap the OpMode's hardwareMap
     * @param motorName   name configured in the Robot Controller config file
     */
    public Shooter(HardwareMap hardwareMap, String motorName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPIDF();
    }

    /** Pushes the current P, I, D, F values to the motor controller. Call again after changing them. */
    public void applyPIDF() {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F));
    }

    /**
     * Queues the correct target speed for the given ball color -- faster
     * for yellow, slower for red/blue. Does not spin the motor by itself;
     * call spin() (e.g. while a trigger is held) to actually engage it.
     */
    public void setVelocityForColor(ColorSensor.DetectedColor color) {
        if (color == ColorSensor.DetectedColor.YELLOW) {
            queuedVelocity = VELOCITY_YELLOW;
        } else {
            // RED or BLUE -- same slower speed for both. NONE keeps
            // whatever speed was queued for the last real ball detected.
            if (color == ColorSensor.DetectedColor.RED || color == ColorSensor.DetectedColor.BLUE) {
                queuedVelocity = VELOCITY_RED_BLUE;
            }
        }
    }

    /** Spins the shooter up to the currently queued target velocity. */
    public void spin() {
        setTargetVelocity(queuedVelocity);
    }

    /** Sets the target spin speed directly, in encoder ticks per second. */
    public void setTargetVelocity(double ticksPerSecond) {
        targetVelocity = ticksPerSecond;
        motor.setVelocity(targetVelocity);
    }

    /** Stops the shooter (target velocity 0) without forgetting the queued color speed. */
    public void stop() {
        setTargetVelocity(0.0);
    }

    public double getQueuedVelocity() {
        return queuedVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** Current measured velocity in encoder ticks per second. */
    public double getCurrentVelocity() {
        return motor.getVelocity();
    }

    /** How far off the current speed is from the target (ticks per second). */
    public double getVelocityError() {
        return targetVelocity - motor.getVelocity();
    }

    /** True once the shooter is within the given tolerance of its target speed. */
    public boolean isAtTargetVelocity(double toleranceTicksPerSecond) {
        return Math.abs(getVelocityError()) <= toleranceTicksPerSecond;
    }
}
