package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Ity {
    public enum States{
    VERITY,
    STOP
    }

    private States state = States.STOP;

    DcMotor shooterMotor;

    public static double verityBluePower = 0.8;
    public static double verityYellowPower = 0.8;

    public static double verityPower = verityBluePower;

    public void initiate(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.dcMotor.get("shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void setState(States newState) {
        state = newState;
    }

    /** Sets the VERITY power for the given ball color: red and blue share one power, yellow has its own. */
    public void setVerityPowerForColor(ColorSensor.DetectedColor color) {
        if (color == ColorSensor.DetectedColor.BLUE || color == ColorSensor.DetectedColor.RED) {
            verityPower = verityBluePower;
        } else if (color == ColorSensor.DetectedColor.YELLOW) {
            verityPower = verityYellowPower;
        }
    }

    public void update(){
        switch (state) {
            case VERITY:
                shooterMotor.setPower(verityPower);
                break;
            case STOP:
                shooterMotor.setPower(0);
        }
    }
    public States getState() {
        return state;
    }
}
