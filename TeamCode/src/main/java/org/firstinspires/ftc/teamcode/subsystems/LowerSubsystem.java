package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LowerSubsystem {
    private DcMotor lower;
    private boolean out;

    public LowerSubsystem(HardwareMap hardwareMap) {
        lower = hardwareMap.get(DcMotor.class, "lower");
        //lower.setDirection(DcMotor.Direction.REVERSE);
        lower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
}
