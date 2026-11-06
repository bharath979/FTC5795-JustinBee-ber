package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.PinPointDriver;

/**
 * goBILDA Pinpoint odometry (2 dead wheels: 1 forward "X" pod + 1 strafe "Y" pod).
 *
 * REQUIREMENTS
 *  1. Add goBILDA's PinPointDriver.java to your TeamCode folder
 *     (github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint).
 *  2. In the Robot Config, add an I2C device on the Control/Expansion Hub:
 *       Type: goBILDA Pinpoint Odometry Computer      Name: "odo"
 *  3. Pinpoint port 'X' = forward-facing pod, port 'Y' = strafe-facing pod.
 *
 * The Pinpoint has its own IMU and fuses everything on-board, so you do NOT
 * need a separate REV IMU or any math for the wheel/heading fusion.
 */
@TeleOp(name = "Pinpoint Odometry Test", group = "Test")
public class OdometryTest extends LinearOpMode {

    // ====================== MEASURE AND EDIT THESE ======================
    // Origin = the point on the robot you want (0,0) to be (usually center of rotation).
    // X pod offset: how far LEFT of origin the forward pod sits (left = +, right = -)
    // Y pod offset: how far FORWARD of origin the strafe pod sits (forward = +, back = -)
    static final double X_POD_OFFSET_MM = -84.0;
    static final double Y_POD_OFFSET_MM = -168.0;

    // Flip these if a value goes negative when it should be positive
    static final boolean X_POD_REVERSED = false;
    static final boolean Y_POD_REVERSED = false;

    // Drive motor names / directions (edit to match your robot)
    static final String FL = "frontLeft", FR = "frontRight", BL = "backLeft", BR = "backRight";
    // ====================================================================

    /** Small wrapper so you can reuse this in Auto as well. */
    public static class Odometry {
        private final PinPointDriver pinpoint;

        public Odometry(com.qualcomm.robotcore.hardware.HardwareMap hw, String name) {
            pinpoint = hw.get(PinPointDriver.class, name);

            pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
            pinpoint.setEncoderResolution(PinPointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    X_POD_REVERSED ? PinPointDriver.EncoderDirection.REVERSED
                            : PinPointDriver.EncoderDirection.FORWARD,
                    Y_POD_REVERSED ? PinPointDriver.EncoderDirection.REVERSED
                            : PinPointDriver.EncoderDirection.FORWARD);

            // Zeros position and calibrates the IMU -- robot must be STILL (~0.25 s)
            pinpoint.resetPosAndIMU();
        }

        /** Call once every loop. */
        public void update() { pinpoint.update(); }

        /** Set the starting pose (e.g. your auto start position). inches / degrees. */
        public void setPose(double xIn, double yIn, double headingDeg) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg));
        }

        public void reset() { pinpoint.resetPosAndIMU(); }

        public double getX()          { return pinpoint.getPosition().getX(DistanceUnit.INCH); }
        public double getY()          { return pinpoint.getPosition().getY(DistanceUnit.INCH); }
        public double getHeadingRad() { return pinpoint.getPosition().getHeading(AngleUnit.RADIANS); }
        public double getHeadingDeg() { return pinpoint.getPosition().getHeading(AngleUnit.DEGREES); }

        public double getVelX()       { return pinpoint.getVelX(DistanceUnit.INCH); }      // in/s
        public double getVelY()       { return pinpoint.getVelY(DistanceUnit.INCH); }      // in/s
        public double getVelHeading() { return pinpoint.getHeadingVelocity(AngleUnit.DEGREES.getUnnormalized()); }

        public double getFrequency()  { return pinpoint.getFrequency(); }
        public PinPointDriver.DeviceStatus getStatus() { return pinpoint.getDeviceStatus(); }
    }

    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.get(DcMotor.class, FL);
        DcMotor fr = hardwareMap.get(DcMotor.class, FR);
        DcMotor bl = hardwareMap.get(DcMotor.class, BL);
        DcMotor br = hardwareMap.get(DcMotor.class, BR);

        // Typical mecanum: reverse the left side. Change if your robot drives wrong.
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        Odometry odo = new Odometry(hardwareMap, "odo");

        telemetry.addLine("Keep the robot still, then press START");
        telemetry.addData("Status", odo.getStatus());
        telemetry.update();

        waitForStart();
        odo.setPose(0, 0, 0);

        while (opModeIsActive()) {
            odo.update();

            // Reset pose with A (robot must be still)
            if (gamepad1.a) odo.reset();

            // Robot-centric mecanum drive for testing
            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double d  = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            fl.setPower((y + x + rx) / d);
            bl.setPower((y - x + rx) / d);
            fr.setPower((y - x - rx) / d);
            br.setPower((y + x - rx) / d);

            telemetry.addData("X (in)", "%.2f", odo.getX());
            telemetry.addData("Y (in)", "%.2f", odo.getY());
            telemetry.addData("Heading (deg)", "%.2f", odo.getHeadingDeg());
            telemetry.addData("Vel X / Y (in/s)", "%.1f / %.1f", odo.getVelX(), odo.getVelY());
            telemetry.addData("Pinpoint Hz", "%.0f", odo.getFrequency());
            telemetry.addData("Status", odo.getStatus());
            telemetry.update();
        }
    }
}