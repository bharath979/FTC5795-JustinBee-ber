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

    // Fraction of the aiming error corrected each loop. Raise for faster
    // tracking, lower if the turret oscillates around the tag.
    private static final double KP = 0.20;

    // Ignore errors smaller than this so the turret doesn't jitter on target.
    private static final double DEADBAND_DEGREES = 1.0;

    // Top speed the turret is allowed to turn while tracking. Lower this
    // first if the turret moves too fast or whips past the tag.
    private static final double MAX_SPEED_DEGREES_PER_SECOND = 360.0;

    // How much of the tag's measured speed the turret matches (0 = none,
    // 1 = all of it). Lower if the turret overshoots while the robot moves.
    private static final double FEEDFORWARD_GAIN = 0.8;

    // Frame time assumed for the speed limit before two frames have been timed.
    private static final double DEFAULT_FRAME_SECONDS = 0.03;

    // Camera + servo delay to aim ahead by while the robot is moving. Raise if
    // the turret trails behind the tag at speed, lower if it leads past it.
    private static final double LATENCY_SECONDS = 0.05;

    // How quickly the tag-speed estimate follows new readings (0-1). Lower is
    // smoother but slower to react to the robot changing speed.
    private static final double RATE_SMOOTHING = 0.4;

    // Tag speeds below this are treated as standing still, so camera noise
    // doesn't make the turret twitch when the robot is parked.
    private static final double MIN_RATE_DEGREES_PER_SECOND = 5.0;

    // A gap between frames longer than this means the speed estimate is stale.
    private static final double MAX_FRAME_GAP_SECONDS = 0.25;

    // Flip to -1 if the turret turns away from the tag instead of toward it.
    private static final double DIRECTION = -1.0;

    // With no tag in view for this long, the turret goes back to center. The
    // wait stops a single dropped frame from yanking it off the tag.
    private static final double LOST_TAG_RETURN_SECONDS = 0.5;

    // Servo limits so the turret never twists into wires or the frame.
    private static final double MIN_POSITION = 0.10;
    private static final double MAX_POSITION = 0.90;
    private static final double CENTER_POSITION = 0.50;

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
    private double lastFrameTime;
    private double lastSeenTime;
    private double lastBearing;
    private double bearingRate;
    private boolean hasLastBearing;

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
     * of center) of the tag to aim at, or null if no tag is visible. With no
     * tag the turret holds briefly, then returns to center.
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
            // Lost the tag: forget its speed so a stale value isn't used when it comes back
            hasLastBearing = false;
            bearingRate = 0;
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

        // Where the tag is relative to the robot's front, and how fast that is
        // changing as the robot drives and spins
        double bearing = getAngleDegrees() + txDegrees;
        double dt = now - lastFrameTime;
        if (hasLastBearing && dt > 0 && dt < MAX_FRAME_GAP_SECONDS) {
            double rate = (bearing - lastBearing) / dt;
            bearingRate += RATE_SMOOTHING * (rate - bearingRate);
        } else {
            bearingRate = 0;
            dt = 0;
        }
        lastBearing = bearing;
        lastFrameTime = now;
        hasLastBearing = true;

        double rate = Math.abs(bearingRate) < MIN_RATE_DEGREES_PER_SECOND ? 0 : bearingRate;

        // Aim where the tag will be once the camera delay has passed
        double error = txDegrees + rate * LATENCY_SECONDS;
        if (Math.abs(error) < DEADBAND_DEGREES) {
            error = 0;
        }
        // Chase the error, and also keep turning at the tag's speed so the
        // turret doesn't fall behind while the robot is moving
        double correctionDegrees = KP * error + FEEDFORWARD_GAIN * rate * dt;

        // Never turn faster than the speed limit, however big the error is
        double maxStep = MAX_SPEED_DEGREES_PER_SECOND * (dt > 0 ? dt : DEFAULT_FRAME_SECONDS);
        correctionDegrees = Math.max(-maxStep, Math.min(maxStep, correctionDegrees));
        setPosition(currentPosition + DIRECTION * correctionDegrees / TURRET_DEGREES_PER_SERVO_UNIT);
    }

    /** How fast the tag is sweeping past the robot, in degrees per second (positive = to the right). */
    public double getBearingRate() {
        return bearingRate;
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
