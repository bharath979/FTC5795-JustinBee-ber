package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Subsystems.BlueCamera;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoalTracker;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.RedCamera;
import org.firstinspires.ftc.teamcode.Subsystems.Ity;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Odometry + Camera TeleOp")
public class OdoCameraTeleOp extends LinearOpMode {

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

        // Servos stay still during INIT; the turret centers when Play is pressed.
        Turret turret = new Turret(hardwareMap, "TurretLeft", "TurretRight");

        Ity ity = new Ity();
        ity.initiate(hardwareMap);
        //Slides slides = new Slides(hardwareMap, "frontRightMotor", "frontRightMotor");

        Intake intake = new Intake();
        intake.initiate(hardwareMap);

        // Pinpoint odometry + camera together keep the turret and hood on the goal
        // even when the tags drop out of view. Robot must be still during INIT
        // while the Pinpoint calibrates. If there's no "odo" in the config, the
        // turret and hood fall back to camera-only.
        Odometry odometry = null;
        try {
            odometry = new Odometry(hardwareMap, "odo");
        } catch (Exception e) {
            telemetry.addLine("No Pinpoint \"odo\" found - camera-only aiming");
        }
        // One goal estimate per tag group (audience side and far side are different
        // spots on the field), so sightings of one never get averaged into the other.
        GoalTracker audienceGoal = new GoalTracker();
        GoalTracker farGoal = new GoalTracker();

        // The tag group the camera saw last; odometry aims at that group's goal.
        boolean lastAimFarSide = false;


        // Pick the alliance during INIT so it's set before the match starts.
        while (opModeInInit()) {
            if (gamepad1.backWasPressed()) {
                redAlliance = !redAlliance;
            }
            telemetry.addData("Odometry", odometry == null ? "not found (camera only)" : odometry.getStatus());
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
            // --- Alliance: Back button switches which goal tags the camera aims at.
            if (gamepad1.backWasPressed()) {
                redAlliance = !redAlliance;
                audienceGoal.reset();
                farGoal.reset();
                lastAimFarSide = false;
            }

            // --- Odometry + camera: each new camera sighting pins the goal to a spot on the
            // odometry map; while the tags are out of view, odometry says where it is.
            Double aimTx = redAlliance ? redCamera.getAimTx() : blueCamera.getAimTx();
            double frameTimestamp = redAlliance ? redCamera.getFrameTimestamp() : blueCamera.getFrameTimestamp();
            if (odometry != null) {
                odometry.update();
                // Lets the turret turn against the robot's turning before the camera sees it
                turret.setRobotHeading(odometry.getHeadingDeg());
                // Sightings go to the goal of the tag group the camera is locked onto.
                if (aimTx != null) {
                    lastAimFarSide = redAlliance ? redCamera.isAimingFarSide() : blueCamera.isAimingFarSide();
                }
                Double tagRight = null;
                Double tagForward = null;
                if (redAlliance) {
                    RedCamera.TagPose center = redCamera.getAimCenterPose();
                    if (center != null) { tagRight = center.x; tagForward = center.y; }
                } else {
                    BlueCamera.TagPose center = blueCamera.getAimCenterPose();
                    if (center != null) { tagRight = center.x; tagForward = center.y; }
                }
                GoalTracker seenGoal = lastAimFarSide ? farGoal : audienceGoal;
                GoalTracker otherGoal = lastAimFarSide ? audienceGoal : farGoal;
                seenGoal.update(odometry, turret.getAngleDegrees(), tagRight, tagForward, frameTimestamp);
                otherGoal.update(odometry, turret.getAngleDegrees(), null, null, frameTimestamp);
            }
            // Aim at the group the camera saw last; if that one has no goal yet (e.g. it
            // was only glimpsed while the turret swung past), use the other group's.
            GoalTracker goalTracker = lastAimFarSide ? farGoal : audienceGoal;
            if (!goalTracker.hasGoal()) {
                GoalTracker otherGroup = lastAimFarSide ? audienceGoal : farGoal;
                if (otherGroup.hasGoal()) {
                    goalTracker = otherGroup;
                }
            }
            boolean useOdometryAim = aimTx == null && odometry != null && goalTracker.hasGoal();

            // --- Hood auto angle: Limelight distance to the tag raises the hood when farther
            // away and lowers it when closer. With no tag in view, odometry's distance to the
            // remembered goal takes over; with neither, it holds the last angle.
            Double tagDistance = redAlliance ? getTagDistance(redCamera) : getTagDistance(blueCamera);
            if (tagDistance == null && useOdometryAim) {
                tagDistance = goalTracker.getDistanceToGoal();
            }
            if (tagDistance != null) {
                hood.setAngleForDistance(tagDistance);
            }
            // --- Turret: keeps the Limelight centered on the hive's tag cluster while it's
            // visible. When it's lost, odometry points it at where the goal should be; without
            // odometry it holds still briefly and then returns to center.
            if (useOdometryAim) {
                turret.aimAtAngle(goalTracker.getTurretAngleToGoal());
            } else {
                turret.track(aimTx, frameTimestamp);
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
            telemetry.addData("Hood Angle", hood.getAnglePosition());
            telemetry.addData("Turret Angle (deg)", "%.1f", turret.getAngleDegrees());
            telemetry.addData("Turret Left / Right Servo", "%.3f / %.3f", turret.getPosition(), turret.getRightServoPosition());
            telemetry.addData("Turret Tracking", turret.isTracking());
            // Should shrink toward 0 as the turret turns; if it grows, flip DIRECTION in Turret.
            telemetry.addData("Aim Error tx (deg)", aimTx == null ? "no tag" : String.format("%.1f", aimTx));
            telemetry.addData("Tag Distance (in)", tagDistance == null ? "no tag"
                    : String.format("%.1f%s", tagDistance, useOdometryAim ? " (odometry)" : ""));
            if (odometry != null) {
                goalTracker.addTelemetry(telemetry);
            }
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
