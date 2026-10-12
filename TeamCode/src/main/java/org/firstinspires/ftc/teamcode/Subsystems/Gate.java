package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    public enum States{
        OPEN,
        CLOSED
    }
    private States state = States.CLOSED;
    Servo gateServo;

    public static double openPosition = 0.5;
    public static double closedPosition = 0.0;

    public void initiate(HardwareMap hardwareMap){
        gateServo = hardwareMap.servo.get("gateServo");
    }

    /** Moves the gate servo to the position for the given state. */
    public void setState(States newState){
        state = newState;
        if (state == States.OPEN) {
            gateServo.setPosition(openPosition);
        } else {
            gateServo.setPosition(closedPosition);
        }
    }

    public void open(){
        setState(States.OPEN);
    }

    public void close(){
        setState(States.CLOSED);
    }

    /** Flips the gate: open if it was closed, closed if it was open. */
    public void toggle(){
        if (state == States.OPEN) {
            close();
        } else {
            open();
        }
    }

    public States getState(){
        return state;
    }

    public boolean isOpen(){
        return state == States.OPEN;
    }

    public double getPosition(){
        return gateServo.getPosition();
    }
}
