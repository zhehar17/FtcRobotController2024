package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class UpperSubsystem {
    private DcMotor upper;
    private boolean up;

    public UpperSubsystem(HardwareMap hardwareMap) {
        upper = hardwareMap.get(DcMotor.class, "upper");
        upper.setDirection(DcMotor.Direction.REVERSE);
        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isUp(){
        return up;
    }


    public void goUp(){
        upper.setPower(RobotConstants.upperUpPower);
        up = true;
    }

    public void stayUp(){
        upper.setPower(RobotConstants.upperUpHoldPower);
        up = false;
    }

    public void scoreDown(){
        upper.setPower(RobotConstants.upperDownPower);
        up = false;
    }
    public void floorDown(){
        upper.setPower(-0.2);
    }

    public void off(){
        upper.setPower(0);
        up = false;
    }

    public double getHeight(){
        return upper.getCurrentPosition();
    }

    public void resetEncoder(){
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

