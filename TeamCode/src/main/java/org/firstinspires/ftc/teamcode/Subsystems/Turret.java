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

    // How much of the aiming error each camera frame aims to remove. 1.0 aims
    // straight at the tag; a bit less stops it overshooting while the camera
    // catches up. Raise if it stops short of the tag, lower if it swings past.
    private static final double KP = 0.9;

    // How old a camera frame is by the time the code reads it, plus how far the
    // servos lag behind their commands (seconds). Each correction is added to
    // where the turret was pointing when the frame was taken, not where it is
    // now, so the same error isn't corrected twice (that's what made it
    // overshoot when it first locked on). If it still swings past the tag when
    // it locks on, try 0.02 higher; if it starts wobbling back and forth, it's
    // too high, so go 0.02 lower. 0 turns this off.
    private static final double CAMERA_DELAY_SECONDS = 0.05;

    // Ignore errors smaller than this so the turret doesn't jitter on target.
    private static final double DEADBAND_DEGREES = 1.0;

    // Top turning speed. The turret glides toward its target at up to this
    // speed every loop instead of jumping once per camera frame, which is
    // what made it tick. Raise for snappier tracking, lower if it's jerky.
    private static final double MAX_SPEED_DEGREES_PER_SECOND = 1000.0;

    // Loop time assumed on the first call, before a real one has been measured.
    private static final double DEFAULT_LOOP_SECONDS = 0.02;

    // Flip the sign (-1.0 <-> 1.0) if the turret turns away from the tag instead of toward it.
    private static final double DIRECTION = 1.0;

    // With no tag in view for this long, the turret goes back to center. Until
    // then it holds still, so a blurry or dropped frame doesn't swing it away.
    private static final double LOST_TAG_RETURN_SECONDS = 1.5;

    // Servo position where the turret (and Limelight) points straight forward.
    private static final double CENTER_POSITION = 0.50;

    // The two ends of the turret's travel, as LEFT servo positions (0 to 1).
    // The turret (camera and odometry aiming) never goes outside these. Measure
    // them with the "Turret Limit Finder" OpMode: drive the turret to each end
    // of the range you want, press A at one end and B at the other, and copy
    // the numbers here. Order doesn't matter.
    // Defaults (not yet measured): 170deg one way and 25deg the other from center.
    private static final double LIMIT_POSITION_A = 0.014;
    private static final double LIMIT_POSITION_B = 0.571;

    private static final double MIN_POSITION =
            Math.max(0.0, Math.min(LIMIT_POSITION_A, LIMIT_POSITION_B));
    private static final double MAX_POSITION =
            Math.min(1.0, Math.max(LIMIT_POSITION_A, LIMIT_POSITION_B));

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
    private double targetPosition = CENTER_POSITION;
    private boolean tracking;
    private double lastFrameTimestamp = -1;
    private double lastSeenTime;
    private double lastLoopTime = -1;

    // Recent servo commands (time in seconds, position), newest at historyHead,
    // so track() can look up where the turret pointed when a frame was taken.
    private static final int HISTORY_SIZE = 64;
    private final double[] historyTime = new double[HISTORY_SIZE];
    private final double[] historyPosition = new double[HISTORY_SIZE];
    private int historyHead = -1;
    private int historyCount = 0;

    // Robot heading from odometry (degrees, counterclockwise = +), if the OpMode
    // gives it. Lets the turret turn against the robot's own turning right away
    // instead of waiting for the next camera frame to show the tag moved.
    private boolean hasHeading;
    private double robotHeadingDeg;
    private double lastLoopHeadingDeg;
    private boolean hasLastLoopHeading;
    private final double[] headingTime = new double[HISTORY_SIZE];
    private final double[] headingValue = new double[HISTORY_SIZE];
    private int headingHead = -1;
    private int headingCount = 0;

    /**
     * @param hardwareMap      the OpMode's hardwareMap
     * @param leftServoName    name of the first turret servo
     * @param rightServoName   name of the second turret servo (mirrored)
     */
    public Turret(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
        leftServo = hardwareMap.get(Servo.class, leftServoName);
        rightServo = hardwareMap.get(Servo.class, rightServoName);
        // No servo command here, so nothing moves during INIT. Call center()
        // after waitForStart() to power the servos and point forward.
        currentPosition = CENTER_POSITION;
    }

    /**
     * Call every loop. Pass the Limelight tx (degrees, positive = tag is right
     * of center) of the tag cluster to aim at, or null if none is visible.
     * With no tag the turret holds still, then glides back to center.
     *
     * @param frameTimestamp the camera's timestamp for the frame tx came from.
     *                       The loop runs faster than the camera, so the turret
     *                       only corrects once per new frame; reusing an old tx
     *                       would stack corrections and overshoot the tag.
     */
    public void track(Double txDegrees, double frameTimestamp) {
        tracking = txDegrees != null;
        double now = System.nanoTime() / 1e9;

        // Robot turned since last loop: turn the turret the other way by the same
        // amount so it keeps pointing at the same spot on the field.
        if (hasHeading) {
            recordHeading(now);
            if (hasLastLoopHeading) {
                targetPosition = clampPosition(targetPosition
                        + DIRECTION * wrapDegrees(robotHeadingDeg - lastLoopHeadingDeg) / TURRET_DEGREES_PER_SERVO_UNIT);
            }
            lastLoopHeadingDeg = robotHeadingDeg;
            hasLastLoopHeading = true;
        }

        if (!tracking) {
            if (now - lastSeenTime > LOST_TAG_RETURN_SECONDS) {
                targetPosition = clampPosition(CENTER_POSITION);
            }
        } else {
            lastSeenTime = now;
            // Only a new frame moves the target; reusing an old tx would stack
            // corrections and overshoot the tag.
            if (frameTimestamp != lastFrameTimestamp) {
                lastFrameTimestamp = frameTimestamp;
                if (Math.abs(txDegrees) >= DEADBAND_DEGREES) {
                    // tx is the error as of when the frame was taken, so correct
                    // from where the turret pointed then.
                    double takenAt = now - CAMERA_DELAY_SECONDS;
                    double positionWhenTaken = positionAt(takenAt);
                    // Plus however far the robot has turned since then
                    double turnedSince = hasHeading ? wrapDegrees(robotHeadingDeg - headingAt(takenAt)) : 0;
                    targetPosition = clampPosition(positionWhenTaken
                            + DIRECTION * (KP * txDegrees + turnedSince) / TURRET_DEGREES_PER_SERVO_UNIT);
                }
            }
        }

        glideToTarget(now);
    }

    /**
     * Call every loop instead of {@link #track} when the camera can't see the
     * tags but odometry knows where the goal is. Glides to the given turret
     * angle (degrees from the robot's front, positive = right, same as
     * {@link #getAngleDegrees()}) and doesn't return to center.
     */
    public void aimAtAngle(double turretDegrees) {
        tracking = false;
        hasLastLoopHeading = false;   // aimed from odometry directly; no turn carry-over
        double now = System.nanoTime() / 1e9;
        lastSeenTime = now;
        targetPosition = clampPosition(
                CENTER_POSITION + DIRECTION * turretDegrees / TURRET_DEGREES_PER_SERVO_UNIT);
        glideToTarget(now);
    }

    /** Moves toward targetPosition, no faster than the speed limit. */
    private void glideToTarget(double now) {
        double dt = lastLoopTime < 0 ? DEFAULT_LOOP_SECONDS : now - lastLoopTime;
        lastLoopTime = now;
        double maxStep = MAX_SPEED_DEGREES_PER_SECOND * dt / TURRET_DEGREES_PER_SERVO_UNIT;
        double step = Math.max(-maxStep, Math.min(maxStep, targetPosition - currentPosition));
        if (step != 0) {
            setPosition(currentPosition + step);
        }
    }

    /** Snaps straight to center (used at INIT and Play). */
    public void center() {
        targetPosition = clampPosition(CENTER_POSITION);
        setPosition(CENTER_POSITION);
    }

    private static double clampPosition(double position) {
        return Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
    }

    public void setPosition(double position) {
        applyPosition(clampPosition(position));
    }

    /**
     * Moves the turret with NO travel limits. Only for the Turret Limit Finder,
     * to measure where the limits should be; go slowly near the hard stops.
     */
    public void setPositionUnlimited(double position) {
        targetPosition = Math.max(0.0, Math.min(1.0, position));
        applyPosition(targetPosition);
    }

    /**
     * Optional: give the robot's heading from odometry (degrees, counterclockwise
     * = +) every loop, before {@link #track}. While tracking, the turret then
     * turns against the robot's own turning right away. OpModes without
     * odometry just don't call this.
     */
    public void setRobotHeading(double headingDegrees) {
        robotHeadingDeg = headingDegrees;
        hasHeading = true;
    }

    private void recordHeading(double now) {
        headingHead = (headingHead + 1) % HISTORY_SIZE;
        headingTime[headingHead] = now;
        headingValue[headingHead] = robotHeadingDeg;
        headingCount = Math.min(headingCount + 1, HISTORY_SIZE);
    }

    /** The robot heading recorded at the given time (same clock as positionAt). */
    private double headingAt(double time) {
        for (int i = 0; i < headingCount; i++) {
            int index = (headingHead - i + HISTORY_SIZE) % HISTORY_SIZE;
            if (headingTime[index] <= time) {
                return headingValue[index];
            }
        }
        return headingCount == 0 ? robotHeadingDeg
                : headingValue[(headingHead - headingCount + 1 + HISTORY_SIZE) % HISTORY_SIZE];
    }

    private static double wrapDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    /** The position commanded at the given time (seconds, System.nanoTime() clock). */
    private double positionAt(double time) {
        if (historyCount == 0) {
            return currentPosition;
        }
        // Newest to oldest: the last command sent at or before that time
        for (int i = 0; i < historyCount; i++) {
            int index = (historyHead - i + HISTORY_SIZE) % HISTORY_SIZE;
            if (historyTime[index] <= time) {
                return historyPosition[index];
            }
        }
        // Older than the history goes back: use the oldest one kept
        return historyPosition[(historyHead - historyCount + 1 + HISTORY_SIZE) % HISTORY_SIZE];
    }

    private void applyPosition(double position) {
        currentPosition = position;
        historyHead = (historyHead + 1) % HISTORY_SIZE;
        historyTime[historyHead] = System.nanoTime() / 1e9;
        historyPosition[historyHead] = position;
        historyCount = Math.min(historyCount + 1, HISTORY_SIZE);
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

    /** Position last sent to the right (mirrored) servo. */
    public double getRightServoPosition() {
        return rightServo.getPosition();
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
