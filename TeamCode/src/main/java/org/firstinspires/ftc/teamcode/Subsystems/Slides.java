package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private final DcMotorEx upMotor;
    private final DcMotorEx downMotor;

    // Soft limits in encoder ticks -- tune these to your slide's real
    // travel once you've measured it (see class comment above).
    public static int MIN_POSITION = 0;
    public static int MAX_POSITION = 3000;

    private boolean softLimitsEnabled = true;

    /**
     * @param hardwareMap    the OpMode's hardwareMap
     * @param upMotorName    name of the motor that reels in the up-string (extends the slide)
     * @param downMotorName  name of the motor that reels in the down-string (retracts the slide)
     */
    public Slides(HardwareMap hardwareMap, String upMotorName, String downMotorName) {
        upMotor = hardwareMap.get(DcMotorEx.class, upMotorName);
        downMotor = hardwareMap.get(DcMotorEx.class, downMotorName);

        // The two motors are usually mounted facing opposite ways relative
        // to their strings, so one is reversed here so that positive power
        // on both drives the slide the same way instead of fighting itself.
        // If your slide moves backwards, or the motors strain against each
        // other, flip which one is REVERSE.
        upMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        downMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // BRAKE so the slide holds its height under gravity when power is 0.
        upMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        downMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    /**
     * Drives both motors together at the given power (-1.0 to 1.0).
     * Positive extends the slide, negative retracts it. If soft limits are
     * enabled, power toward a limit is clipped to 0 once the slide reaches
     * MIN_POSITION or MAX_POSITION so it can't drive past its travel.
     */
    public void setPower(double power) {
        if (softLimitsEnabled) {
            int position = getPosition();
            if (power > 0 && position >= MAX_POSITION) {
                power = 0;
            } else if (power < 0 && position <= MIN_POSITION) {
                power = 0;
            }
        }

        upMotor.setPower(power);
        downMotor.setPower(power);
    }

    /** Stops the slide (holds height, since ZeroPowerBehavior is BRAKE). */
    public void stop() {
        setPower(0);
    }

    /** Average of the two motor encoder positions, in ticks. */
    public int getPosition() {
        return (upMotor.getCurrentPosition() + downMotor.getCurrentPosition()) / 2;
    }

    public int getUpMotorPosition() {
        return upMotor.getCurrentPosition();
    }

    public int getDownMotorPosition() {
        return downMotor.getCurrentPosition();
    }

    /** Zeroes both encoders. Call this while the slide is fully retracted. */
    public void resetEncoders() {
        upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        downMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        downMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSoftLimitsEnabled(boolean enabled) {
        softLimitsEnabled = enabled;
    }

    public boolean isAtMax() {
        return getPosition() >= MAX_POSITION;
    }

    public boolean isAtMin() {
        return getPosition() <= MIN_POSITION;
    }
}
