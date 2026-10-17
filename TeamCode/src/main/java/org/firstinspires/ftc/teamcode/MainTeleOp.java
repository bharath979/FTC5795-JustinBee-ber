package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Subsystems.BlueCamera;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.RedCamera;
import org.firstinspires.ftc.teamcode.Subsystems.Ity;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    // Stick readings smaller than this count as 0, so a stick that doesn't
    // center perfectly can't creep the robot. Raise it if the robot still drifts.
    private static final double STICK_DEADZONE = 0.08;

    @Override
    public void runOpMode() {
        BlueCamera blueCamera = new BlueCamera();
        blueCamera.initiate(hardwareMap);
        RedCamera redCamera = new RedCamera();
        redCamera.initiate(hardwareMap);

        // Starts on blue; the Back button flips between blue and red goal tags.
        boolean redAlliance = false;

        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);

        ColorSensor colorSensor = new ColorSensor(hardwareMap, "ColorSensor");

        // The config names are crossed on the robot: "HoodAngle" is plugged into the width servo
        // and "HoodWidth" into the angle servo, so they are passed in swapped here.
        Hood hood = new Hood(hardwareMap, "HoodAngle", "HoodWidth");

        // Constructor centers the turret so both servos are powered and holding from INIT.
        Turret turret = new Turret(hardwareMap, "TurretLeft", "TurretRight");

        Ity ity = new Ity();
        ity.initiate(hardwareMap);
        //Slides slides = new Slides(hardwareMap, "frontRightMotor", "frontRightMotor");

        Intake intake = new Intake();
        intake.initiate(hardwareMap);


        // Pick the alliance during INIT so it's set before the match starts.
        while (opModeInInit()) {
            if (gamepad1.backWasPressed()) {
                redAlliance = !redAlliance;
            }
            telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
            telemetry.addLine("Press BACK to switch");
            telemetry.update();
        }

        waitForStart();

        // Command the hood servos right away so they hold position instead of staying limp.
        hood.goToStart();
        turret.center();

        while (opModeIsActive()) {

            double y = -deadzone(gamepad1.left_stick_y);
            double x = deadzone(gamepad1.left_stick_x) * 1.1;
            double rx = deadzone(gamepad1.right_stick_x);
            drivetrain.run(x, y, rx);

            // * double slidespower = 0;
           // if(gamepad1.right_bumper) {
             //   slides.setPower(slidespower);
            //}else if(gamepad1.left_bumper){
              //  slides.setPower(slidespower);
            //}else{
             //   slides.stop();
            // } */

            //blueCamera.update(telemetry);
            colorSensor.update();

            if (colorSensor.isNewDetection()) {
                ColorSensor.DetectedColor color = colorSensor.getDetectedColor();
                hood.setForColor(color);
                ity.setVerityPowerForColor(color);

            //Below is for if Color Sensor does not work
            /* if (gamepad1.yWasPressed()) {
                hood.setState(Hood.States.POLLEN);
            } else if (gamepad1.bWasPressed()) {
                hood.setState(Hood.States.NECTAR);
            }
            hood.update(); */

            } 
            if (gamepad1.dpadUpWasPressed()) {
                hood.adjustAngle(1);
            } else if (gamepad1.dpadDownWasPressed()) {
                hood.adjustAngle(-1);
            }
            // --- Alliance: Back button switches which goal tags the camera aims at.
            if (gamepad1.backWasPressed()) {
                redAlliance = !redAlliance;
            }

            // --- Hood auto angle: Limelight distance to the tag raises the hood when farther
            // away and lowers it when closer. Holds the last angle if no tag is visible.
            Double tagDistance = redAlliance ? getTagDistance(redCamera) : getTagDistance(blueCamera);
            if (tagDistance != null) {
                hood.setAngleForDistance(tagDistance);
            }
            // --- Turret: keeps the Limelight centered on the hive's tag cluster while it's
            // visible, holds still if it's briefly lost, and returns to center after that.
            Double aimTx = redAlliance ? redCamera.getAimTx() : blueCamera.getAimTx();
            double frameTimestamp = redAlliance ? redCamera.getFrameTimestamp() : blueCamera.getFrameTimestamp();
            turret.track(aimTx, frameTimestamp);

            if (gamepad1.dpadRightWasPressed()) {
                hood.adjustYellowWidth(1);
            } else if (gamepad1.dpadLeftWasPressed()) {
                hood.adjustYellowWidth(-1);
            }

            // --- Shooter: spins at the queued speed while the trigger i
            if (gamepad1.x) {
                ity.setState(Ity.States.VERITY);
            } else if (gamepad1.a) {
                ity.setState(Ity.States.STOP);
            }
            ity.update();

            if (gamepad1.right_trigger > 0.1) {
                intake.setState(Intake.States.IN);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setState(Intake.States.OUT);
            } else {
                intake.setState(Intake.States.STOP);
            }
            intake.update();


            telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");            colorSensor.addTelemetry(telemetry);
            telemetry.addData("Hood Width", hood.getWidthPosition());
            telemetry.addData("Hood Angle", hood.getAnglePosition());
            telemetry.addData("Hood Angle Trim", hood.getAngleTrim());
            telemetry.addData("Turret Position", turret.getPosition());
            telemetry.addData("Turret Tracking", turret.isTracking());
            // Should shrink toward 0 as the turret turns; if it grows, flip DIRECTION in Turret.
            telemetry.addData("Aim Error tx (deg)", aimTx == null ? "no tag" : String.format("%.1f", aimTx));
            telemetry.addData("Tag Distance (in)", tagDistance == null ? "no tag" : String.format("%.1f", tagDistance));
            /*telemetry.addData("Slide Position", slides.getPosition());
            telemetry.addData("At Max", slides.isAtMax());
            telemetry.addData("At Min", slides.isAtMin()); */

            telemetry.update();
        }
        // Both cameras share the same Limelight, so closing one stops it.
        blueCamera.close();
    }

    /** Returns 0 for small stick readings, so stick drift doesn't move the robot. */
    private static double deadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0 : value;
    }

    /** Filtered floor distance (inches) to the blue tag being aimed at, or null if none is visible. */
    private static Double getTagDistance(BlueCamera camera) {
        LLResultTypes.FiducialResult aimTag = camera.getAimDetection();
        if (aimTag == null) {
            return null;
        }
        BlueCamera.TagPose pose = camera.getFilteredPose(aimTag.getFiducialId());
        return pose == null ? null : pose.range;
    }

    /** Filtered floor distance (inches) to the red tag being aimed at, or null if none is visible. */
    private static Double getTagDistance(RedCamera camera) {
        LLResultTypes.FiducialResult aimTag = camera.getAimDetection();
        if (aimTag == null) {
            return null;
        }
        RedCamera.TagPose pose = camera.getFilteredPose(aimTag.getFiducialId());
        return pose == null ? null : pose.range;
    }
}
