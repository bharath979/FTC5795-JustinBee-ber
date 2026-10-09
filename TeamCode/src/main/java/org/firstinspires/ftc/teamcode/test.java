package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.BlueCamera;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Ity;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
@TeleOp
public class test extends LinearOpMode {
    @Override
    public void runOpMode() {
        BlueCamera blueCamera = new BlueCamera();
        blueCamera.initiate(hardwareMap);

        Ity ity = new Ity();
        ity.initiate(hardwareMap);

        ity.setVerityPowerForColor(ColorSensor.DetectedColor.BLUE);

        ColorSensor colorSensor = new ColorSensor(hardwareMap, "ColorSensor");
        Hood hood = new Hood(hardwareMap, "HoodWidth", "HoodAngle");

        Turret turret = new Turret(hardwareMap, "TurretLeft", "TurretRight");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_trigger > 0.1) {
                ity.setState(Ity.States.VERITY);
            } else if (gamepad1.a) {
                ity.setState(Ity.States.STOP);
            }
            ity.update();

            blueCamera.update(telemetry);
            colorSensor.update();

            if (colorSensor.isNewDetection()) {
                ColorSensor.DetectedColor color = colorSensor.getDetectedColor();
                hood.setForColor(color);
                ity.setVerityPowerForColor(color);
            }

            Double aimTx = blueCamera.getAimTx();
            turret.track(aimTx, blueCamera.getFrameTimestamp());

            telemetry.addData("Ity State", ity.getState());
            telemetry.addData("Verity Power", Ity.verityPower);
            colorSensor.addTelemetry(telemetry);
            telemetry.addData("Hood Width", hood.getWidthPosition());
            telemetry.addData("Hood Angle", hood.getAnglePosition());
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
