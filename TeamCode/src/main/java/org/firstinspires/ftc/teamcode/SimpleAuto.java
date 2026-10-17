package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Ity;

@Autonomous
public class SimpleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);

        Ity ity = new Ity();
        ity.initiate(hardwareMap);

        Intake intake = new Intake();
        intake.initiate(hardwareMap);

        Gate gate = new Gate();
        gate.initiate(hardwareMap);

        Hood hood = new Hood(hardwareMap, "HoodWidth", "HoodAngle");

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        timer.reset();

        // Every loop checks opModeIsActive() so pressing Stop halts the robot right away.
        // Forward for 1 second
        while (opModeIsActive()) {

            while (timer.seconds() < 5) {
                gate.setState(Gate.States.OPEN);
                ity.setState(Ity.States.VERITY);
                hood.setState(Hood.States.POLLEN);
                ity.update();
                hood.update();
                while (timer.seconds() > 1 && timer.seconds() < 5) {
                    intake.setState(Intake.States.IN);
                    intake.update();
                }
            }
            ity.setState(Ity.States.STOP);
            intake.setState(Intake.States.STOP);
            ity.update();
            intake.update();


            while (timer.seconds() >5 && timer.seconds() < 6) {
                    drivetrain.SimpleAutoRun();
            }
                // Wait until 5 seconds
            while (timer.seconds() > 6 && timer.seconds() < 7.5) {
                    drivetrain.run(-0.5, 0, 0);
            }

            while (timer.seconds() > 7.5 && timer.seconds() < 10) {
                    drivetrain.SimpleAutoRun();
            }


            while (timer.seconds() > 7 && timer.seconds() < 8) {
                    drivetrain.run(-0.5, 0, 0);
            }
            drivetrain.stop();
            }

        }
}

