package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor intakeMotor;
    DcMotor transferMotor;


    public void initiate(HardwareMap hardwaremap) {
        intakeMotor = hardwaremap.dcMotor.get("backLeftMotor");
        transferMotor = hardwaremap.dcMotor.get("frontLeftMotor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run(){
        double intakeMotorPower = 0.5;
        double transferMotorPower = 0.5;

        intakeMotor.setPower(intakeMotorPower);
        transferMotor.setPower(transferMotorPower);
    }

    public void stop(){
        intakeMotor.setPower(0.0);
        transferMotor.setPower(0.0);
    }


}
