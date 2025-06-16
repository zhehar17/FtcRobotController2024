package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.lang.Math;

public class UpperSubsystem {
    public Servo lower1 = null;
    public Servo lower2 = null;
    public Servo upperpivot = null;
    public Servo claw = null;

    double clawpos = 0.5;

    double pos1 = 0.26;
    double pos2 = 1-pos1;
    double pivot = 0.48;
    public UpperSubsystem(HardwareMap hardwareMap) {
        lower1 = hardwareMap.get(Servo.class, "upper1"); //.35  to .65 in
        lower2 = hardwareMap.get(Servo.class, "upper2"); //.35 in to .65
        upperpivot = hardwareMap.get(Servo.class, "upperpivot");
        claw = hardwareMap.get(Servo.class, "claw");
        lower1.setPosition(pos1);
        lower2.setPosition(pos2);
    }

    public void closeClaw(){
        clawpos = 0.3;
        claw.setPosition(clawpos);
    }
    public void openClaw(){
        clawpos = 0.56;
        claw.setPosition(clawpos);
    }
    public void pickup(){
        pos1 = 0.26;
        pos2 = 1-pos1;
        pivot = 0.44;
        clawpos = 0.56;
        lower1.setPosition(pos1);
        lower2.setPosition(pos2);
        upperpivot.setPosition(pivot);
        claw.setPosition(clawpos);
    }
    public void up(){
        pos1 = 0.95;
        pos2 = 1-pos1;
        pivot = 0.25;
        lower1.setPosition(pos1);
        lower2.setPosition(pos2);
        upperpivot.setPosition(pivot);
        claw.setPosition(clawpos);
    }
    public void score(){
        pos1 = 0.26;
        pos2 = 1-pos1;
        pivot = 0;
        lower1.setPosition(pos1);
        lower2.setPosition(pos2);
        upperpivot.setPosition(pivot);
    }

    public void setWrist(double wristpos){
        pivot = wristpos;
        upperpivot.setPosition(pivot);
    }
    public void setArm(double armPos){

        pos1 = armPos;
        pos2 = 1-pos1;
        lower1.setPosition(pos1);
        lower2.setPosition(pos2);
    }
}

