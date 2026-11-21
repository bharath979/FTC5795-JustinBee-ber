package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * goBILDA Pinpoint odometry (2 dead wheels: 1 forward "X" pod + 1 strafe "Y" pod).
 *
 * Robot Config: I2C device "goBILDA Pinpoint Odometry Computer" named "odo".
 * Pinpoint port 'X' = forward-facing pod, port 'Y' = strafe-facing pod.
 *
 * Field frame: +x is the way the robot faced at INIT, +y is to its left,
 * heading is counterclockwise-positive. Units are inches and degrees.
 */
public class Odometry {

    // Origin = the point on the robot you want (0,0) to be (usually center of rotation).
    // X pod offset: how far LEFT of origin the forward pod sits (left = +, right = -)
    // Y pod offset: how far FORWARD of origin the strafe pod sits (forward = +, back = -)
    // Not measured yet: run the "Odometry Auto Tuner" on the robot and put its numbers here.
    public static final double X_POD_OFFSET_MM = -84.0;
    public static final double Y_POD_OFFSET_MM = -168.0;

    // Flip these if a value goes negative when it should be positive
    public static final boolean X_POD_REVERSED = false;
    public static final boolean Y_POD_REVERSED = false;

    private final PinPointDriver pinpoint;

    public Odometry(HardwareMap hardwareMap) {
        this(hardwareMap, "odo");
    }

    public Odometry(HardwareMap hardwareMap, String name) {
        pinpoint = hardwareMap.get(PinPointDriver.class, name);

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(PinPointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                X_POD_REVERSED ? PinPointDriver.EncoderDirection.REVERSED
                        : PinPointDriver.EncoderDirection.FORWARD,
                Y_POD_REVERSED ? PinPointDriver.EncoderDirection.REVERSED
                        : PinPointDriver.EncoderDirection.FORWARD);

        // Zeros position and calibrates the IMU -- robot must be STILL (~0.25 s)
        pinpoint.resetPosAndIMU();
    }

    /** Call once every loop, before reading the pose. */
    public void update() { pinpoint.update(); }

    /** Set the current pose (e.g. your auto start position). inches / degrees. */
    public void setPose(double xIn, double yIn, double headingDeg) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg));
    }

    /**
     * Changes the pod offsets on the Pinpoint right now (millimeters, same
     * meaning as X_POD_OFFSET_MM / Y_POD_OFFSET_MM). Used by the Odometry Auto
     * Tuner; normal OpModes use the constants above.
     */
    public void setPodOffsetsMm(double xPodOffsetMm, double yPodOffsetMm) {
        pinpoint.setOffsets(xPodOffsetMm, yPodOffsetMm, DistanceUnit.MM);
    }

    /** Zeros the pose and recalibrates the IMU. Robot must be still. */
    public void reset() { pinpoint.resetPosAndIMU(); }

    public double getX()          { return pinpoint.getPosition().getX(DistanceUnit.INCH); }
    public double getY()          { return pinpoint.getPosition().getY(DistanceUnit.INCH); }
    public double getHeadingRad() { return pinpoint.getPosition().getHeading(AngleUnit.RADIANS); }
    public double getHeadingDeg() { return pinpoint.getPosition().getHeading(AngleUnit.DEGREES); }

    public double getVelX()       { return pinpoint.getVelX(DistanceUnit.INCH); }      // in/s
    public double getVelY()       { return pinpoint.getVelY(DistanceUnit.INCH); }      // in/s
    /** Turning speed, degrees per second (counterclockwise = +). */
    public double getVelHeading() { return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES); }

    public double getFrequency()  { return pinpoint.getFrequency(); }
    public PinPointDriver.DeviceStatus getStatus() { return pinpoint.getDeviceStatus(); }
}
