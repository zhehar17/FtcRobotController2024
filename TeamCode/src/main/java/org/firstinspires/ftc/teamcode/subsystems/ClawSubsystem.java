package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class ClawSubsystem {

    private Servo claw;

    public ClawSubsystem(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void closeClaw() {
        claw.setPosition(RobotConstants.closedClawPos);
    }

    public void openClaw() {
        claw.setPosition(RobotConstants.openClawPos);
    }

    public boolean isClosed(){
        return 0.01 > Math.abs(claw.getPosition() - RobotConstants.closedClawPos);
    }

    public boolean isOpen(){
        return 0.01 > Math.abs(claw.getPosition() - RobotConstants.openClawPos);
    }

}
