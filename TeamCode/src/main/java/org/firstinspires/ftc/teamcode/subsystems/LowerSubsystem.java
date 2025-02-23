package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LowerSubsystem {
    private DcMotor lower;
    private Servo grabber;
    private Servo wrist;
    private Servo pivot;
    private boolean out;
    private double wristPos;

    public LowerSubsystem(HardwareMap hardwareMap) {
        lower = hardwareMap.get(DcMotor.class, "lower");
        //lower.setDirection(DcMotor.Direction.REVERSE);
        lower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabber = hardwareMap.get(Servo.class, "grabber");
        wrist = hardwareMap.get(Servo.class, "wrist");
        pivot = hardwareMap.get(Servo.class, "pivot");

        out = false;
        wristPos = 1;
    }

    public void setPivot(double pos){
        pivot.setPosition(pos);
    }
    public boolean out(){
        return out;
    }
    public void bottomoff(){
        lower.setPower(0);
    }
    public void bottomon(){
        lower.setPower(0.02);
    }

    public double getPosition(){
        return lower.getCurrentPosition();
    }

    public void extend(double power){
        lower.setPower(-power*0.7);
        out = true;
    }

    public void retract(double power){
        lower.setPower(power*0.7);
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
    public void wristUp(){
        wrist.setPosition(1);
    }
    public void lower(){
        wrist.setPosition(0);
    }
    public void raise(){
        wrist.setPosition(0.5);
    }

    public void wristPos(double pos){
        wrist.setPosition(pos);
    }

    public void grab() {
        grabber.setPosition(0.62);
    }

    public void release() {
        grabber.setPosition(0.25);
    }

    public boolean closed() {
        return grabber.getPosition() == 0.62;
    }

    public void runTo1850() {

        if (getPosition() < -1860) {
            retract(0.5);
        } else if (getPosition() > -1840) {
            extend(0.5);
        } else {
            bottomoff();
        }

    }

}
