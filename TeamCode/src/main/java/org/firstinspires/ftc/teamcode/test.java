package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.BlueCamera;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Ity;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
@TeleOp
public class test extends LinearOpMode {
    @Override
    public void runOpMode() {


        Ity ity = new Ity();
        ity.initiate(hardwareMap);

        ity.setVerityPowerForColor(ColorSensor.DetectedColor.BLUE);

        ColorSensor colorSensor = new ColorSensor(hardwareMap, "ColorSensor");
        // The config names are crossed on the robot: "HoodAngle" is plugged into the width servo
        // and "HoodWidth" into the angle servo, so they are passed in swapped here.
        Hood hood = new Hood(hardwareMap, "HoodAngle", "HoodWidth");

        Turret turret = new Turret(hardwareMap, "TurretLeft", "TurretRight");



        Intake intake = new Intake();
        intake.initiate(hardwareMap);
        // On INIT, put the hood width at its fixed position (no movement if it is already there).
        hood.goToWidth();

        waitForStart();

        // Put the hood at its starting position (fixed width, start angle).
        hood.goToStart();
        ;

        while (opModeIsActive()) {


            if (gamepad1.right_trigger > 0.1) {
                intake.setState(Intake.States.IN);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setState(Intake.States.OUT);
            } else {
                intake.setState(Intake.States.STOP);
            }
            intake.update();

            if (gamepad1.x) {
                ity.setState(Ity.States.VERITY);
            } else if (gamepad1.a) {
                ity.setState(Ity.States.STOP);
            }
            ity.update();


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
            if (gamepad1.dpadRightWasPressed()) {
                hood.adjustYellowWidth(1);
            } else if (gamepad1.dpadLeftWasPressed()) {
                hood.adjustYellowWidth(-1);
            }


            telemetry.addData("Ity State", ity.getState());
            telemetry.addData("Verity Power", Ity.verityPower);

            colorSensor.addTelemetry(telemetry);
            telemetry.addData("Hood Width", hood.getWidthPosition());
            telemetry.addData("Hood Angle", hood.getAnglePosition());
            telemetry.addData("Yellow Width", hood.getYellowWidth());
            telemetry.addData("Turret Position", turret.getPosition());
            telemetry.addData("Turret Tracking", turret.isTracking());
            telemetry.addData("Turret At Limit", turret.isAtLimit());
            telemetry.update();
        }

    }
}
