package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LowerSubsystem {
    private DcMotor lower;
    private Servo grabber;
    private Servo wrist;
    private boolean out;

    public LowerSubsystem(HardwareMap hardwareMap) {
        lower = hardwareMap.get(DcMotor.class, "lower");
        //lower.setDirection(DcMotor.Direction.REVERSE);
        lower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabber = hardwareMap.get(Servo.class, "grabber");
        wrist = hardwareMap.get(Servo.class, "wrist");

        out = false;
    }

    public boolean out(){
        return out;
    }
    public void bottomoff(){
        lower.setPower(0);
    }
    public void bottomon(){
        lower.setPower(0.01);
    }

    public double getPosition(){
        return lower.getCurrentPosition();
    }

    public void extend(){
        lower.setPower(-1);
        out = true;
    }

    public void retract(){
        lower.setPower(1);
        out = false;
    }

    public void slowExtend() {
        lower.setPower(-.4);
        out = true;
    }

    public void slowRetract() {
        lower.setPower(.4);
        out = false;
    }

    //Wrist
    public void lower(){
        wrist.setPosition(1);
    }
    public void raise(){
        wrist.setPosition(0.3);
    }

    public void grab() {
        grabber.setPosition(1);
    }

    public void release() {
        grabber.setPosition(0);
    }

    public boolean closed() {
        return grabber.getPosition() == 1;
    }
}
