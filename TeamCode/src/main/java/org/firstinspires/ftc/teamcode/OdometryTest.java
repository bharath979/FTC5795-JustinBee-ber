package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Odometry;

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

    // Pod offsets and directions live in Subsystems/Odometry.java.

    // Drive motor names (edit to match your robot)
    static final String FL = "frontLeftMotor", FR = "frontRightMotor", BL = "backLeftMotor", BR = "backRightMotor";

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