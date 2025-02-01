package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.lang.Math;

public class PositionSubsystem {
    private Limelight3A limelight;

    public LLResult result;


    public PositionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.setPollRateHz(100);
        //limelight.start();
    }

    public void start(){
        limelight.start();
    }

    public boolean validResult(){
        result = limelight.getLatestResult();
        return result != null && result.isValid() && (result.getStaleness() < 100);
    }

    public long staleResult(){
        result = limelight.getLatestResult();
        return result.getStaleness();
    }

    public Pose3D getPose(){
        return result.getBotpose();
    }

    public double getX(){
        if(-(result.getBotpose().getPosition().y * 39.3701) + 72  > 72){
            return (result.getBotpose().getPosition().y * 39.3701) + 72;
        }
        return -(result.getBotpose().getPosition().y * 39.3701) + 72;

    }

    public double getY(){
        if(result.getBotpose().getPosition().x * 39.3701 + 72 > 72){
            return -result.getBotpose().getPosition().x * 39.3701 + 72;
        }
        return result.getBotpose().getPosition().x * 39.3701 + 72;
    }

    public double getYaw(){
        if(result.getBotpose().getPosition().x * 39.3701 + 72 > 72){
            return result.getBotpose().getOrientation().getYaw() - 90;
        }
        return result.getBotpose().getOrientation().getYaw() + 90;
    }

    public double getPieceAngle(int i){
        result = limelight.getLatestResult();
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            double firstOutput = pythonOutputs[i];
            return firstOutput;
        }
        return 0;
    }

    public boolean isRunning(){
        return limelight.isRunning();
    }
}
