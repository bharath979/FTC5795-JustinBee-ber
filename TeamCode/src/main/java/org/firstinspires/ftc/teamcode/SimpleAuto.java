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

        // The config names are crossed on the robot: "HoodAngle" is plugged into the width servo
        // and "HoodWidth" into the angle servo, so they are passed in swapped here.
        Hood hood = new Hood(hardwareMap, "HoodAngle", "HoodWidth");

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        timer.reset();

        // Every loop checks opModeIsActive() so pressing Stop halts the robot right away.

        // 0-5 s: open the gate, spin up the shooter, set the hood. Intake feeds from 2 s.
        gate.setState(Gate.States.OPEN);
        ity.setState(Ity.States.VERITY);
        hood.setState(Hood.States.POLLEN);
        hood.update();
        while (opModeIsActive() && timer.seconds() < 5) {
            if (timer.seconds() > 2) {
                intake.setState(Intake.States.IN);
            }
            ity.update();
            intake.update();
        }

        // Stop shooting. update() is what actually sends the new power to the motors.
        ity.setState(Ity.States.STOP);
        intake.setState(Intake.States.STOP);
        hood.setState(Hood.States.NECTAR);
        ity.update();
        intake.update();
        hood.update();

        // 5-6 s: forward
        while (opModeIsActive() && timer.seconds() < 6) {
            drivetrain.SimpleAutoRun();
        }

        // 6-7.5 s: strafe left
        while (opModeIsActive() && timer.seconds() < 7.5) {
            drivetrain.run(-0.5, 0, 0);
        }

        // 7.5-9 s: forward
        while (opModeIsActive() && timer.seconds() < 9) {
            drivetrain.SimpleAutoRun();
        }

        // 9-10 s: strafe left
        while (opModeIsActive() && timer.seconds() < 10) {
            drivetrain.run(-0.5, 0, 0);
        }

        drivetrain.stop();
    }
}
