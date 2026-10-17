package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Autonomous
public class SimpleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        timer.reset();

        // Every loop checks opModeIsActive() so pressing Stop halts the robot right away.
        // Forward for 1 second
        while (opModeIsActive() && timer.seconds() < 1) {
            drivetrain.SimpleAutoRun();
        }
        drivetrain.stop();

        // Wait until 5 seconds
        while (opModeIsActive() && timer.seconds() < 5) {
            idle();
        }

        // Backward from 5 to 7 seconds
        while (opModeIsActive() && timer.seconds() < 7) {
            drivetrain.SimpleAutoReverseRun();
        }
        drivetrain.stop();
    }
}
