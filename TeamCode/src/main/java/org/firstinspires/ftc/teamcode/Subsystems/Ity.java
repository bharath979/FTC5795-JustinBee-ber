package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Ity {
    public enum States{
    VERITY,
    STOP
    }

    private States state = States.STOP;

    DcMotor intakeMotor;

    public static double verityBluePower = 1.0;
    public static double verityYellowPower = 0.9;

    public static double verityPower = verityBluePower;

    public void initiate(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("backLeftMotor");
    }
    public void setState(States newState) {
        state = newState;
    }

    /** Sets the VERITY power for the given ball color. Other colors keep the last power. */
    public void setVerityPowerForColor(ColorSensor.DetectedColor color) {
        if (color == ColorSensor.DetectedColor.BLUE) {
            verityPower = verityBluePower;
        } else if (color == ColorSensor.DetectedColor.YELLOW) {
            verityPower = verityYellowPower;
        }
    }

    public void update(){
        switch (state) {
            case VERITY:
                intakeMotor.setPower(verityPower);
                break;
            case STOP:
                intakeMotor.setPower(0);
        }
    }
    public States getState() {
        return state;
    }
}
