package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.BlueCamera;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Autonomous
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() {
        BlueCamera camera = new BlueCamera();
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
