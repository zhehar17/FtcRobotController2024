package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class PositionSubsystem {
    private Limelight3A limelight;
    private LLResult result;

    public PositionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    public boolean validResult(){
        result = limelight.getLatestResult();
        return result != null && result.isValid() && (result.getStaleness() < 100);
    }

    public long staleResult(){
        return result.getStaleness();
    }

    public Pose3D getPose(){
        return result.getBotpose();
    }
}
