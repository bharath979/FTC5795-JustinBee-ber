package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Combines odometry with the Limelight to keep track of where the goal is.
 *
 * Every time the camera sees the goal tags, the tag position (relative to the
 * camera) is turned into a point on the field using the odometry pose and the
 * turret angle, and that point is smoothed with a Kalman filter. When the tags
 * drop out of view (turned away, blocked, blurry), odometry alone says where
 * the goal is: which way to point the turret and how far away it is.
 *
 * No field map is needed. The goal is stored in the odometry's own frame, so
 * every new camera sighting also soaks up any odometry drift.
 */
public class GoalTracker {

    // ====================== MEASURE AND EDIT THESE ======================
    // Where the turret's rotation axis sits relative to the odometry origin
    // (the point Odometry calls (0,0)), in inches. Forward = +, left = +.
    private static final double TURRET_FORWARD_OFFSET_IN = 0.0;
    private static final double TURRET_LEFT_OFFSET_IN = 0.0;

    // How far in front of the turret axis the Limelight lens is, in inches,
    // measured along the way the turret points.
    private static final double CAMERA_FORWARD_OFFSET_IN = 0.0;
    // ====================================================================

    // The camera frame is a few tens of ms old by the time we read it, so a
    // sighting taken while the robot or turret is spinning lands in the wrong
    // place. Skip sightings while turning faster than these.
    private static final double MAX_ROBOT_TURN_DEG_PER_SEC = 120.0;
    private static final double MAX_TURRET_TURN_DEG_PER_SEC = 120.0;

    // The goal doesn't move, so the filter mostly averages sightings. Process
    // noise lets it slowly follow odometry drift; raise it if the estimate
    // lags behind after driving a long way, lower it if it jumps around.
    private static final double GOAL_PROCESS_NOISE = 0.05;
    private static final double GOAL_MEASUREMENT_NOISE = 4.0;

    // Odometry drifts, so a goal not seen for this long is forgotten and the
    // turret goes back to its normal camera-only behavior.
    private static final double GOAL_MEMORY_SECONDS = 20.0;

    private final KalmanFilter goalXFilter = new KalmanFilter(GOAL_PROCESS_NOISE, GOAL_MEASUREMENT_NOISE);
    private final KalmanFilter goalYFilter = new KalmanFilter(GOAL_PROCESS_NOISE, GOAL_MEASUREMENT_NOISE);

    private boolean hasGoal;
    private double goalX, goalY;
    private double lastFixTime = -1;
    private double lastFrameTimestamp = -1;
    private double lastTurretAngle = Double.NaN;
    private double lastLoopTime = -1;
    private boolean usedFixThisLoop;

    // Latest robot pose, cached each update so the getters don't re-read I2C.
    private double robotX, robotY, robotHeadingRad;

    public void update(Odometry odometry, double turretAngleDeg,
                       Double tagRightIn, Double tagForwardIn, double frameTimestamp) {
        robotX = odometry.getX();
        robotY = odometry.getY();
        robotHeadingRad = odometry.getHeadingRad();
        usedFixThisLoop = false;

        double now = System.nanoTime() / 1e9;
        double dt = lastLoopTime < 0 ? 0 : now - lastLoopTime;
        lastLoopTime = now;
        double turretRate = (Double.isNaN(lastTurretAngle) || dt <= 0)
                ? 0 : Math.abs(turretAngleDeg - lastTurretAngle) / dt;
        lastTurretAngle = turretAngleDeg;

        if (hasGoal && now - lastFixTime > GOAL_MEMORY_SECONDS) {
            reset();
        }

        boolean newFrame = frameTimestamp != lastFrameTimestamp;
        lastFrameTimestamp = frameTimestamp;
        if (tagRightIn == null || tagForwardIn == null || !newFrame) {
            return;
        }
        if (Math.abs(odometry.getVelHeading()) > MAX_ROBOT_TURN_DEG_PER_SEC
                || turretRate > MAX_TURRET_TURN_DEG_PER_SEC) {
            return;
        }

        // Which way the camera faces on the field (turret positive = clockwise)
        double cameraHeading = robotHeadingRad - Math.toRadians(turretAngleDeg);
        double cos = Math.cos(cameraHeading);
        double sin = Math.sin(cameraHeading);

        double[] turretAxis = getTurretAxis();
        double cameraX = turretAxis[0] + CAMERA_FORWARD_OFFSET_IN * cos;
        double cameraY = turretAxis[1] + CAMERA_FORWARD_OFFSET_IN * sin;

        // Camera forward = (cos, sin), camera right = (sin, -cos)
        double measuredX = cameraX + tagForwardIn * cos + tagRightIn * sin;
        double measuredY = cameraY + tagForwardIn * sin - tagRightIn * cos;

        goalX = goalXFilter.update(measuredX);
        goalY = goalYFilter.update(measuredY);
        hasGoal = true;
        lastFixTime = now;
        usedFixThisLoop = true;
    }

    /** Forget the goal (e.g. when switching alliance). */
    public void reset() {
        hasGoal = false;
        goalXFilter.reset();
        goalYFilter.reset();
    }

    /** True once the goal has been seen recently enough to trust odometry for it. */
    public boolean hasGoal() {
        return hasGoal;
    }

    /** Turret angle (degrees from robot front, positive = right) that points at the goal. */
    public double getTurretAngleToGoal() {
        double[] axis = getTurretAxis();
        double fieldAngle = Math.atan2(goalY - axis[1], goalX - axis[0]);
        double relative = normalizeRadians(fieldAngle - robotHeadingRad);
        return -Math.toDegrees(relative);
    }

    /** Floor distance (inches) from the camera to the goal, as odometry sees it. */
    public double getDistanceToGoal() {
        double[] axis = getTurretAxis();
        return Math.hypot(goalX - axis[0], goalY - axis[1]) - CAMERA_FORWARD_OFFSET_IN;
    }

    /** Seconds since the camera last updated the goal position. */
    public double getSecondsSinceFix() {
        return lastFixTime < 0 ? Double.POSITIVE_INFINITY : System.nanoTime() / 1e9 - lastFixTime;
    }

    public double getGoalX() { return goalX; }
    public double getGoalY() { return goalY; }

    @SuppressLint("DefaultLocale")
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Robot Pose (in, deg)", String.format("%.1f, %.1f, %.1f",
                robotX, robotY, Math.toDegrees(robotHeadingRad)));
        if (!hasGoal) {
            telemetry.addData("Goal (odometry)", "not seen yet");
            return;
        }
        telemetry.addData("Goal (odometry)", String.format("%.1f, %.1f  %s",
                goalX, goalY, usedFixThisLoop ? "camera fix" : String.format("%.1fs since fix", getSecondsSinceFix())));
        telemetry.addData("Goal Dist / Turret Angle", String.format("%.1fin / %.1fdeg",
                getDistanceToGoal(), getTurretAngleToGoal()));
    }

    /** Field position of the turret's rotation axis. */
    private double[] getTurretAxis() {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);
        return new double[]{
                robotX + TURRET_FORWARD_OFFSET_IN * cos - TURRET_LEFT_OFFSET_IN * sin,
                robotY + TURRET_FORWARD_OFFSET_IN * sin + TURRET_LEFT_OFFSET_IN * cos
        };
    }

    private static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
