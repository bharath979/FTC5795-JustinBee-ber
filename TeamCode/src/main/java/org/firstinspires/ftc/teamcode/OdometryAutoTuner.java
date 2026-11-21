package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.PinPointDriver;

import java.util.ArrayList;
import java.util.List;

/**
 * Drives the robot by itself to tune the Pinpoint odometry.
 *
 *  1. Direction check: drives forward a little, then left a little, and checks
 *     the X and Y pods count the right way.
 *  2. Pod offsets: sets the offsets to 0 and spins in place half a turn, both
 *     ways, twice each. With 0 offsets the reported position swings out in an
 *     arc whose size and direction give the real offsets.
 *  3. Check: spins a full turn with the old offsets and with the new ones and
 *     shows how far each drifted (smaller is better; under ~0.5 in is good).
 *
 * It can't change Odometry.java itself: copy the numbers it shows into
 * X_POD_OFFSET_MM / Y_POD_OFFSET_MM (and flip X_/Y_POD_REVERSED if it says so).
 *
 * Needs about 4 ft of clear space all around. Press B at any time to stop.
 */
@TeleOp(name = "Odometry Auto Tuner", group = "Test")
public class OdometryAutoTuner extends LinearOpMode {

    // Gentle powers so the wheels don't slip; raise TURN_POWER if the robot
    // doesn't turn at all, lower it if it skids or overshoots a lot.
    private static final double DRIVE_POWER = 0.3;
    private static final double TURN_POWER = 0.3;
    private static final double DRIVE_SECONDS = 0.6;
    private static final int SPINS_EACH_WAY = 2;
    private static final double SPIN_TIMEOUT_SECONDS = 8.0;
    private static final double MIN_MOVE_INCHES = 1.0;

    private Drivetrain drivetrain;
    private Odometry odometry;
    private boolean aborted;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);
        odometry = new Odometry(hardwareMap, "odo");

        telemetry.addLine("ODOMETRY AUTO TUNER");
        telemetry.addLine("The robot will drive and spin by itself.");
        telemetry.addLine("Clear about 4 ft all around it. Press B any time to stop.");
        telemetry.update();
        waitForStart();

        // ---------- 1. Direction check ----------
        show("Step 1/3: direction check", "Robot should drive FORWARD a bit, then LEFT a bit.");
        resetAndWait();
        drive(0, DRIVE_POWER, 0, DRIVE_SECONDS);
        settle(0.5);
        double forward = robotForward();
        resetAndWait();
        drive(-DRIVE_POWER, 0, 0, DRIVE_SECONDS);   // Drivetrain: x positive = strafe right
        settle(0.5);
        double left = robotLeft();
        if (aborted) { finish("Stopped."); return; }

        List<String> problems = new ArrayList<>();
        if (forward < -MIN_MOVE_INCHES) problems.add(String.format("X went %.1f in driving forward: flip X_POD_REVERSED", forward));
        else if (forward < MIN_MOVE_INCHES) problems.add(String.format("X only moved %.1f in driving forward: check the X pod is plugged in and touching the floor", forward));
        if (left < -MIN_MOVE_INCHES) problems.add(String.format("Y went %.1f in strafing left: flip Y_POD_REVERSED", left));
        else if (left < MIN_MOVE_INCHES) problems.add(String.format("Y only moved %.1f in strafing left: check the Y pod is plugged in and touching the floor", left));
        if (!problems.isEmpty()) {
            telemetry.addLine("DIRECTION CHECK FAILED. Fix these in Odometry.java, redeploy, run again:");
            for (String p : problems) telemetry.addLine("- " + p);
            telemetry.addLine("If the robot itself didn't drive forward then left, the drivetrain");
            telemetry.addLine("motor directions are wrong, not the pods.");
            holdTelemetry();
            return;
        }

        // ---------- 2. Pod offsets ----------
        odometry.setPodOffsetsMm(0, 0);
        List<double[]> results = new ArrayList<>();
        for (int i = 0; i < SPINS_EACH_WAY * 2 && !aborted; i++) {
            double direction = (i % 2 == 0) ? 1 : -1;   // +1 = counterclockwise
            show(String.format("Step 2/3: half turn %d of %d", i + 1, SPINS_EACH_WAY * 2),
                    direction > 0 ? "Spinning left" : "Spinning right");
            resetAndWait();
            double turned = spin(direction, Math.PI);
            settle(0.5);
            if (aborted) break;
            double xMm = odometry.getX() * 25.4;
            double yMm = odometry.getY() * 25.4;
            results.add(offsetsFromSpin(xMm, yMm, turned));
        }
        if (aborted || results.isEmpty()) { finish("Stopped."); return; }

        double sumX = 0, sumY = 0, minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
        for (double[] r : results) {
            sumX += r[0]; sumY += r[1];
            minX = Math.min(minX, r[0]); maxX = Math.max(maxX, r[0]);
            minY = Math.min(minY, r[1]); maxY = Math.max(maxY, r[1]);
        }
        double newX = sumX / results.size();
        double newY = sumY / results.size();

        // ---------- 3. Check: old offsets vs new ----------
        show("Step 3/3: checking old offsets", "Full turn");
        odometry.setPodOffsetsMm(Odometry.X_POD_OFFSET_MM, Odometry.Y_POD_OFFSET_MM);
        double oldDrift = fullTurnDrift();
        show("Step 3/3: checking new offsets", "Full turn");
        odometry.setPodOffsetsMm(newX, newY);
        double newDrift = fullTurnDrift();
        drivetrain.run(0, 0, 0);
        if (aborted) { finish("Stopped."); return; }

        telemetry.addLine("DONE. Put these in Subsystems/Odometry.java:");
        telemetry.addData("X_POD_OFFSET_MM", "%.1f   (was %.1f)", newX, Odometry.X_POD_OFFSET_MM);
        telemetry.addData("Y_POD_OFFSET_MM", "%.1f   (was %.1f)", newY, Odometry.Y_POD_OFFSET_MM);
        telemetry.addData("Spread between runs (mm)", "X %.1f, Y %.1f", maxX - minX, maxY - minY);
        telemetry.addData("Drift after a full spin (in)", "old %.2f -> new %.2f", oldDrift, newDrift);
        telemetry.addLine("Spread over ~10 mm means the wheels slipped: lower TURN_POWER and rerun.");
        telemetry.addLine("Then push the robot 48 in along a tape measure in the");
        telemetry.addLine("Pinpoint Odometry Test; it should read about 48.");
        holdTelemetry();
    }

    /**
     * Pod offsets from a spin in place done with offsets set to 0.
     * With 0 offsets, a pod L mm to the left of center reads -L per radian of
     * counterclockwise turn and a pod F mm forward reads +F, so the reported
     * position after turning theta is M(theta) * (-L, F) with
     * M = [[sin, cos-1], [1-cos, sin]]. Solving that gives L and F.
     *
     * @return {X pod offset (left +), Y pod offset (forward +)} in mm
     */
    static double[] offsetsFromSpin(double xMm, double yMm, double theta) {
        double s = Math.sin(theta), c = Math.cos(theta);
        double det = s * s + (1 - c) * (1 - c);
        double vx = (s * xMm + (1 - c) * yMm) / det;
        double vy = ((c - 1) * xMm + s * yMm) / det;
        return new double[]{-vx, vy};
    }

    /** Spins a full turn counterclockwise and returns how far (in) the position drifted. */
    private double fullTurnDrift() {
        resetAndWait();
        spin(1, 2 * Math.PI);
        settle(0.5);
        return Math.hypot(odometry.getX(), odometry.getY());
    }

    /** Turns in place until the heading has changed by targetRad; returns the exact amount turned. */
    private double spin(double direction, double targetRad) {
        double turned = 0;
        double last = odometry.getHeadingRad();
        ElapsedTime timer = new ElapsedTime();
        // Drivetrain: rx positive turns clockwise, so counterclockwise is negative rx
        drivetrain.run(0, 0, -direction * TURN_POWER);
        while (opModeIsActive() && !checkAbort() && Math.abs(turned) < targetRad
                && timer.seconds() < SPIN_TIMEOUT_SECONDS) {
            odometry.update();
            double now = odometry.getHeadingRad();
            turned += wrap(now - last);
            last = now;
        }
        drivetrain.run(0, 0, 0);
        // Count any coasting after the motors stop
        ElapsedTime coast = new ElapsedTime();
        while (opModeIsActive() && coast.seconds() < 0.4) {
            odometry.update();
            double now = odometry.getHeadingRad();
            turned += wrap(now - last);
            last = now;
        }
        if (timer.seconds() >= SPIN_TIMEOUT_SECONDS) {
            telemetry.addLine("Spin timed out: raise TURN_POWER");
            aborted = true;
        }
        return turned;
    }

    private void drive(double x, double y, double rx, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        drivetrain.run(x, y, rx);
        while (opModeIsActive() && !checkAbort() && timer.seconds() < seconds) {
            odometry.update();
        }
        drivetrain.run(0, 0, 0);
    }

    /** Zeros the Pinpoint (position + IMU) and waits for it to be ready. Robot must be still. */
    private void resetAndWait() {
        drivetrain.run(0, 0, 0);
        settle(0.3);
        odometry.reset();
        ElapsedTime timer = new ElapsedTime();
        do {
            odometry.update();
        } while (opModeIsActive() && timer.seconds() < 2.0
                && (timer.seconds() < 0.3 || odometry.getStatus() != PinPointDriver.DeviceStatus.READY));
        settle(0.2);
    }

    private void settle(double seconds) {
        drivetrain.run(0, 0, 0);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            odometry.update();
            checkAbort();
        }
    }

    private double robotForward() {
        double h = odometry.getHeadingRad();
        return odometry.getX() * Math.cos(h) + odometry.getY() * Math.sin(h);
    }

    private double robotLeft() {
        double h = odometry.getHeadingRad();
        return -odometry.getX() * Math.sin(h) + odometry.getY() * Math.cos(h);
    }

    private boolean checkAbort() {
        if (gamepad1.b) {
            aborted = true;
            drivetrain.run(0, 0, 0);
        }
        return aborted;
    }

    private void show(String step, String detail) {
        telemetry.addLine(step);
        telemetry.addLine(detail);
        telemetry.addLine("Press B to stop");
        telemetry.update();
    }

    private void finish(String message) {
        drivetrain.run(0, 0, 0);
        telemetry.addLine(message);
        holdTelemetry();
    }

    /** Keeps the last screen up until the OpMode is stopped. */
    private void holdTelemetry() {
        drivetrain.run(0, 0, 0);
        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
