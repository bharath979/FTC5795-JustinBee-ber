package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RedCamera;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Autonomous
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        RedCamera camera = new RedCamera();
        camera.initiate(hardwareMap);

        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            camera.update(telemetry);
            telemetry.update();
        }

        camera.close();
    }
}