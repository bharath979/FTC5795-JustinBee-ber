package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Servo-driven turret that keeps the shooter (and the Limelight mounted on it)
 * pointed at the goal AprilTags. Because the camera rides on the turret, the
 * Limelight's tx is the turret's aiming error directly: drive tx to zero and
 * the shooter is on target.
 */
public class Turret {
    private final Servo leftServo;
    private final Servo rightServo;

    // How many turret degrees one full servo range (0 -> 1) covers.
    // Axon servos travel 350deg; this assumes a 1:1 ratio from the left servo
    // to the turret. Multiply by the gear ratio if there is one.
    private static final double TURRET_DEGREES_PER_SERVO_UNIT = 350.0;

    // Fraction of the aiming error corrected on each new camera frame. Raise
    // for faster tracking, lower if the turret swings back and forth on the tag.
    private static final double KP = 0.25;

    // Ignore errors smaller than this so the turret doesn't jitter on target.
    private static final double DEADBAND_DEGREES = 1.0;

    // Most the turret may turn on one camera frame, so a bad reading can't
    // throw it far off the tag.
    private static final double MAX_STEP_DEGREES = 6.0;

    // Flip the sign (-1.0 <-> 1.0) if the turret turns away from the tag instead of toward it.
    private static final double DIRECTION = 1.0;

    // With no tag in view for this long, the turret goes back to center. Until
    // then it holds still, so a blurry or dropped frame doesn't swing it away.
    private static final double LOST_TAG_RETURN_SECONDS = 1.5;

    // Servo position where the turret (and Limelight) points straight forward.
    private static final double CENTER_POSITION = 0.50;

    // How far the turret may turn each way from center, seen from above. It
    // hits a hard stop about 30deg clockwise, so this stays a little short of
    // that to keep the servos from straining against it. Counterclockwise has
    // 350deg+ of room, but the servo only reaches 175deg past center
    // (0.5 x 350deg), so that side is limited by the servo, not the turret.
    private static final double CLOCKWISE_LIMIT_DEGREES = 25.0;
    private static final double COUNTERCLOCKWISE_LIMIT_DEGREES = 170.0;

    // Servo positions for those limits. Clockwise is the same way as positive
    // tx (tag to the right), so DIRECTION also says which way the servo turns
    // clockwise. Flipping DIRECTION keeps the short limit on the clockwise side.
    private static final double CLOCKWISE_POSITION =
            CENTER_POSITION + DIRECTION * CLOCKWISE_LIMIT_DEGREES / TURRET_DEGREES_PER_SERVO_UNIT;
    private static final double COUNTERCLOCKWISE_POSITION =
            CENTER_POSITION - DIRECTION * COUNTERCLOCKWISE_LIMIT_DEGREES / TURRET_DEGREES_PER_SERVO_UNIT;
    private static final double MIN_POSITION =
            Math.max(0.0, Math.min(CLOCKWISE_POSITION, COUNTERCLOCKWISE_POSITION));
    private static final double MAX_POSITION =
            Math.min(1.0, Math.max(CLOCKWISE_POSITION, COUNTERCLOCKWISE_POSITION));

    // Whether the right servo runs mirrored to the left one. If the servos
    // fight each other (right one strains or jitters off center), flip this.
    private static final boolean RIGHT_REVERSED = true;

    // How far the right servo moves for each unit the left servo moves. Both
    // Axons travel 350deg out of the box, so this is 1.0 unless the two use
    // different size gears onto the turret or one was reprogrammed to a
    // different range. Lower it if the right servo turns too far.
    private static final double RIGHT_TRAVEL_SCALE = 1.0;

    // Right servo position that lines up with the left servo at CENTER_POSITION.
    private static final double RIGHT_CENTER_POSITION = 0.50;

    private double currentPosition;
    private boolean tracking;
    private double lastFrameTimestamp = -1;
    private double lastSeenTime;

    /**
     * @param hardwareMap      the OpMode's hardwareMap
     * @param leftServoName    name of the first turret servo
     * @param rightServoName   name of the second turret servo (mirrored)
     */
    public Turret(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
        leftServo = hardwareMap.get(Servo.class, leftServoName);
        rightServo = hardwareMap.get(Servo.class, rightServoName);
        setPosition(CENTER_POSITION);
    }

    /**
     * Call every loop. Pass the Limelight tx (degrees, positive = tag is right
     * of center) of the tag cluster to aim at, or null if none is visible.
     * With no tag the turret holds still, then returns to center.
     *
     * @param frameTimestamp the camera's timestamp for the frame tx came from.
     *                       The loop runs faster than the camera, so the turret
     *                       only corrects once per new frame; reusing an old tx
     *                       would stack corrections and overshoot the tag.
     */
    public void track(Double txDegrees, double frameTimestamp) {
        tracking = txDegrees != null;
        double now = System.nanoTime() / 1e9;
        if (!tracking) {
            if (now - lastSeenTime > LOST_TAG_RETURN_SECONDS) {
                center();
            }
            return;
        }
        lastSeenTime = now;
        if (frameTimestamp == lastFrameTimestamp) {
            return;
        }
        lastFrameTimestamp = frameTimestamp;

        if (Math.abs(txDegrees) < DEADBAND_DEGREES) {
            return;
        }
        // Turn part of the way toward the tag each frame; the next frame shows
        // how much error is left, so it settles on the tag instead of overshooting.
        double correctionDegrees = KP * txDegrees;
        correctionDegrees = Math.max(-MAX_STEP_DEGREES, Math.min(MAX_STEP_DEGREES, correctionDegrees));
        setPosition(currentPosition + DIRECTION * correctionDegrees / TURRET_DEGREES_PER_SERVO_UNIT);
    }

    public void center() {
        setPosition(CENTER_POSITION);
    }

    public void setPosition(double position) {
        currentPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
        leftServo.setPosition(currentPosition);

        // Same turret angle on the right servo, in its own direction and range
        double rightOffset = (currentPosition - CENTER_POSITION) * RIGHT_TRAVEL_SCALE;
        if (RIGHT_REVERSED) {
            rightOffset = -rightOffset;
        }
        rightServo.setPosition(Math.max(0.0, Math.min(1.0, RIGHT_CENTER_POSITION + rightOffset)));
    }

    public double getPosition() {
        return currentPosition;
    }

    /** Turret angle relative to the robot's front, in degrees (positive = right, same as tx). */
    public double getAngleDegrees() {
        return DIRECTION * (currentPosition - CENTER_POSITION) * TURRET_DEGREES_PER_SERVO_UNIT;
    }

    /** True if a tag was visible on the last call to {@link #track}. */
    public boolean isTracking() {
        return tracking;
    }

    /** True if the turret has hit a servo limit and can't turn further toward the tag. */
    public boolean isAtLimit() {
        return currentPosition <= MIN_POSITION || currentPosition >= MAX_POSITION;
    }
}
