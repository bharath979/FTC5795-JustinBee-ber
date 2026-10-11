package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Subsystems.BlueCamera;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Ity;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

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
        Ity ity = new Ity();
        Hood hood = new Hood(hardwareMap, "HoodWidth", "HoodAngle");

        Shooter shooter = new Shooter(hardwareMap, "Shooter");
        Slides slides = new Slides(hardwareMap, "UpMotor", "DownMotor");

        Turret turret = new Turret(hardwareMap, "TurretLeft", "TurretRight");


        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            drivetrain.run(x, y, rx);

            double slidespower = 0;
            if(gamepad1.right_bumper) {
                slides.setPower(slidespower);
            }else if(gamepad1.left_bumper){
                slides.setPower(slidespower);
            }else{
                slides.stop();
            }

            blueCamera.update(telemetry);
            colorSensor.update();

            if (colorSensor.isNewDetection()) {
                ColorSensor.DetectedColor color = colorSensor.getDetectedColor();
                hood.setForColor(color);
                shooter.setVelocityForColor(color);
            }


            // --- Shooter: spins at the queued speed while the trigger is held ---

            if (gamepad1.right_trigger > 0.1) {
                shooter.spin();
            } else {
                shooter.stop();
            }

            Double aimTx = blueCamera.getAimTx();
            turret.track(aimTx, blueCamera.getFrameTimestamp());


            colorSensor.addTelemetry(telemetry);
            telemetry.addData("Hood Width", hood.getWidthPosition());
            telemetry.addData("Hood Angle", hood.getAnglePosition());
            telemetry.addData("Queued Velocity", shooter.getQueuedVelocity());
            telemetry.addData("Slide Position", slides.getPosition());
            telemetry.addData("At Max", slides.isAtMax());
            telemetry.addData("At Min", slides.isAtMin());
            telemetry.addData("Aim tx (deg)", aimTx == null ? "no tag" : aimTx);
            telemetry.addData("Tag Speed (deg/s)", turret.getBearingRate());
            telemetry.addData("Turret Position", turret.getPosition());
            telemetry.addData("Turret Tracking", turret.isTracking());
            telemetry.addData("Turret At Limit", turret.isAtLimit());
            telemetry.update();
        }
        blueCamera.close();
    }
}
