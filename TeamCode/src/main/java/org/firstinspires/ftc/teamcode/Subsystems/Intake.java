package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake{
    public enum States {
        IN,
        OUT,
        STOP
    }


    private States state = States.STOP;

    DcMotor intakeMotor;

    public static double inPower = 0.85;
    public static double outPower = - 0.85;

    public void initiate(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intake");
    }

    public void setState(States newState) {
        state = newState;
    }

    public void update(){
        switch (state) {
            case IN:
                intakeMotor.setPower(inPower);
                break;
            case OUT:
                intakeMotor.setPower(outPower);
                break;
            case STOP:
                intakeMotor.setPower(0);
        }
    }
    public States getState() {
        return state;
    }
}