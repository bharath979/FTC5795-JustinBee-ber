package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Subsystems.BlueCamera;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Ity;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        BlueCamera blueCamera = new BlueCamera();
        blueCamera.initiate(hardwareMap);

        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);

        ColorSensor colorSensor = new ColorSensor(hardwareMap, "ColorSensor");

        Hood hood = new Hood(hardwareMap, "HoodWidth", "HoodAngle");

        Ity ity = new Ity();
        ity.initiate(hardwareMap);
        //Slides slides = new Slides(hardwareMap, "frontRightMotor", "frontRightMotor");

        Intake intake = new Intake();
        intake.initiate(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
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
            } 
            if (gamepad1.dpadUpWasPressed()) {
                hood.adjustAngle(1);
            } else if (gamepad1.dpadDownWasPressed()) {
                hood.adjustAngle(-1);
            }
            // --- Hood auto angle: Limelight distance to the tag raises the hood when farther
            // away and lowers it when closer. Holds the last angle if no tag is visible.
            Double tagDistance = null;
            LLResultTypes.FiducialResult aimTag = blueCamera.getAimDetection();
            if (aimTag != null) {
                BlueCamera.TagPose pose = blueCamera.getFilteredPose(aimTag.getFiducialId());
                if (pose != null) {
                    tagDistance = pose.range;
                    hood.setAngleForDistance(tagDistance);
                }
            }
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


            colorSensor.addTelemetry(telemetry);
            telemetry.addData("Hood Width", hood.getWidthPosition());
            telemetry.addData("Hood Angle", hood.getAnglePosition());
            telemetry.addData("Hood Angle Trim", hood.getAngleTrim());
            telemetry.addData("Tag Distance (in)", tagDistance == null ? "no tag" : String.format("%.1f", tagDistance));
            /*telemetry.addData("Slide Position", slides.getPosition());
            telemetry.addData("At Max", slides.isAtMax());
            telemetry.addData("At Min", slides.isAtMin()); */

            telemetry.update();
        }
        blueCamera.close();
    }
}
