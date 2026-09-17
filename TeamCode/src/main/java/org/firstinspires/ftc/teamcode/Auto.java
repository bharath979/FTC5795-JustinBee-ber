package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Autonomous
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Camera camera = new Camera();
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
