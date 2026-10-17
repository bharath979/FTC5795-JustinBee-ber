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

        while (opModeIsActive() && timer.seconds() < 3) {
            drivetrain.SimpleAutoRun();

        }
        drivetrain.stop();

    }
}
