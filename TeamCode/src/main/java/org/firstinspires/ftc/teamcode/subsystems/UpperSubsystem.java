package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class UpperSubsystem {
    private DcMotor upper;

    public UpperSubsystem(HardwareMap hardwareMap) {
        upper = hardwareMap.get(DcMotor.class, "upper");
        upper.setDirection(DcMotor.Direction.REVERSE);
        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void goUp(){
        upper.setPower(RobotConstants.upperUpPower);
    }

    public void stayUp(){
        upper.setPower(RobotConstants.upperUpHoldPower);
    }

    public void scoreDown(){
        upper.setPower(RobotConstants.upperDownPower);
    }
    public void floorDown(){
        upper.setPower(-0.2);
    }

    public void off(){
        upper.setPower(0);
    }

    public double getHeight(){
        return upper.getCurrentPosition();
    }

    public void resetEncoder(){
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

