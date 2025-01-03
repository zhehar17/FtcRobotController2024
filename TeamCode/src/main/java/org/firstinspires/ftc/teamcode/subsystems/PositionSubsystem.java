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
    private LLResult result;
    private DistanceSensor distance;
    private DistanceSensor distance2;

    public PositionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        distance = hardwareMap.get(DistanceSensor.class, "lowerDistance");
        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");
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

    public double getX(){
        return -(result.getBotpose().getPosition().y * 39.3701) + 72;
    }

    public double getY(){
        return result.getBotpose().getPosition().x * 39.3701 + 72;
    }

    public double getYaw(){
        return result.getBotpose().getOrientation().getYaw() + 90;
    }

    public double getDistanceLeft() {
        return distance.getDistance(DistanceUnit.INCH);
    }

    public double getDistanceRight() {
        return distance2.getDistance(DistanceUnit.INCH);
    }

    public double getDistanceDifference() {
        return distance.getDistance(DistanceUnit.INCH) - distance2.getDistance(DistanceUnit.INCH);
    }

    public double getDistanceHeading() {
        return Math.atan(getDistanceDifference() / RobotConstants.distBetweenSensors);
    }

    public double getDistanceFromSubmersible() {
        double avg = (getDistanceLeft() + getDistanceRight()) /2;
        double angledDist = avg + RobotConstants.sensorToCenter;
        double distToSubmersible = angledDist * Math.cos(getDistanceHeading());
        return distToSubmersible;
    }

    public double getPoseXWall() {
        double avg = (getDistanceLeft() + getDistanceRight()) /2;
        double angledDist = avg + RobotConstants.sensorToCenter;
        double distToWall = angledDist * Math.cos(getDistanceHeading());
        return distToWall + 3;
    }
    public double getPoseXSub() {
        return RobotConstants.submersiblePosition - getDistanceFromSubmersible();
    }

}
