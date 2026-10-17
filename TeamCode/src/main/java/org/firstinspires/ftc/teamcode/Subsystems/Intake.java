package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Intake.States.IN;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.States.OUT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor intakeMotor;
    DcMotor transferMotor;

    public static double intakeMotorPower = -0.8;
    public static double transferMotorPower = -0.8;
    public static double outIntakeMotorPower = 0.5;
    public static double outTransferMotorPower = 0.5;

    public void initiate(HardwareMap hardwaremap) {

        intakeMotor = hardwaremap.dcMotor.get("intakeMotor");
        transferMotor = hardwaremap.dcMotor.get("transferMotor");
    }

    private States state = States.STOP;

    public enum States {
        IN,
        OUT,
        STOP

    }

    public void setState(Intake.States newState) {state = newState;}

    public void update(){
        switch (state){
            case IN:
                intakeMotor.setPower(intakeMotorPower);
                transferMotor.setPower(transferMotorPower);
                break;
            case OUT:
                intakeMotor.setPower(outIntakeMotorPower);
                transferMotor.setPower(outTransferMotorPower);
                break;
            case STOP:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
        }
    }


}
