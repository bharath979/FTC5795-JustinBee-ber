package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.BlueCamera;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Autonomous
public class Auto extends LinearOpMode {

    // Floor distance from the camera to the blue hive tag to stop at.
    private static final double TARGET_RANGE_INCHES = 24.0;
    private static final double RANGE_TOLERANCE_INCHES = 1.5;
    private static final double HEADING_TOLERANCE_DEGREES = 2.0;

    // Power per inch / per degree of error. Lower if the robot overshoots.
    private static final double DRIVE_KP = 0.03;
    private static final double TURN_KP = 0.015;

    private static final double MAX_DRIVE_POWER = 0.4;
    private static final double MAX_TURN_POWER = 0.3;

    @Override
    public void runOpMode() {
        BlueCamera camera = new BlueCamera();
        camera.initiate(hardwareMap);

        Drivetrain drivetrain = new Drivetrain();
        drivetrain.initiate(hardwareMap);

        Turret turret = new Turret(hardwareMap, "TurretLeft", "TurretRight");

        waitForStart();

        // Starts centered, so the Limelight faces the same way as the robot: at the blue hive
        turret.center();

        while (opModeIsActive()) {
            camera.update(telemetry);

            LLResultTypes.FiducialResult detection = camera.getAimDetection();
            BlueCamera.TagPose pose = detection == null
                    ? null : camera.getFilteredPose(detection.getFiducialId());
            Double aimTx = camera.getAimTx();

            if (pose == null || aimTx == null) {
                // No tag: let the turret return to center and don't drive blind
                turret.track(null, camera.getFrameTimestamp());
                drivetrain.run(0, 0, 0);
            } else {
                double tx = aimTx;
                turret.track(tx, camera.getFrameTimestamp());

                // Tag direction relative to the robot's front (positive = right)
                double headingError = turret.getAngleDegrees() + tx;
                double rangeError = pose.range - TARGET_RANGE_INCHES;

                double drive = Math.abs(rangeError) < RANGE_TOLERANCE_INCHES
                        ? 0 : clip(DRIVE_KP * rangeError, MAX_DRIVE_POWER);
                double turn = Math.abs(headingError) < HEADING_TOLERANCE_DEGREES
                        ? 0 : clip(TURN_KP * headingError, MAX_TURN_POWER);

                // Drive along the line to the tag while turning to face it
                double heading = Math.toRadians(headingError);
                drivetrain.run(drive * Math.sin(heading), drive * Math.cos(heading), turn);

                telemetry.addData("Range Error (in)", rangeError);
                telemetry.addData("Heading Error (deg)", headingError);
            }

            telemetry.addData("Turret Position", turret.getPosition());
            telemetry.addData("Turret Tracking", turret.isTracking());
            telemetry.update();
        }

        drivetrain.run(0, 0, 0);
        camera.close();
    }

    private static double clip(double value, double max) {
        return Math.max(-max, Math.min(max, value));
    }
}
