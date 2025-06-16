package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.zeroPowerAccelerationMultiplier;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperSubsystem;




/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/21/2024
 */
@TeleOp(name = "Pedro Pathing TeleOp Enhancements", group = "Test")
public class TeleOpEnhancements extends OpMode {
    private final Pose startPose = new Pose(10.875, 28.935, Math.toRadians(180));
    private final Pose scorePose = new Pose(10,33,Math.toRadians(0));//(37.631, 73.500, Math.toRadians(0));

    private final Pose sidePose = new Pose(68.000, 48.000, Math.toRadians(90));
    private Follower follower;
    public UpperSubsystem upper;
    public ClawSubsystem claw;
    public LowerSubsystem lower;
    double wristpos = 0.5;
    double armpos = 0.26;

    boolean extended;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private PathChain pathx, path1, path2, path3, path4, path5;
    private Timer opmodeTimer;
    public void buildPaths() {
        pathx = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(10.000, 33.000, Point.CARTESIAN),
                                new Point(30.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(30.000, 70.000, Point.CARTESIAN),
                                new Point(40.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(40.000, 70.000, Point.CARTESIAN),
                                new Point(30.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        path1 = follower.pathBuilder() //pickup to score
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(10.875, 28.935, Point.CARTESIAN),
                                new Point(37.631, 73.500, Point.CARTESIAN)//40
                                //new Point(41.495, 22.879, Point.CARTESIAN),
                                //new Point(8.299, 75.589, Point.CARTESIAN),
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        path2 = follower.pathBuilder() //pickup to score
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(37.631, 73.500, Point.CARTESIAN),//40
                                new Point(10.875, 28.935, Point.CARTESIAN)

                                //new Point(41.495, 22.879, Point.CARTESIAN),
                                //new Point(8.299, 75.589, Point.CARTESIAN),
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        path3 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(68.000, 48.000, Point.CARTESIAN),
                                new Point(58.000, 24.000, Point.CARTESIAN),
                                new Point(16.000, 22.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        path4 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(16.000, 22.000, Point.CARTESIAN),
                                new Point(58.000, 24.000, Point.CARTESIAN),
                                new Point(68.000, 45.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
        path5 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(37.631, 62.000, Point.CARTESIAN),
                                new Point(22.000, 26.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(22.000, 26.000, Point.CARTESIAN),
                                new Point(45.000, 28.000, Point.CARTESIAN),
                                new Point(11.000, 28.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))
                .build();

    }
    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    public int curAct;
    public double pivotDouble;
    public double lowerGrabPos;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(scorePose);
        buildPaths();

        claw = new ClawSubsystem(hardwareMap);
        upper = new UpperSubsystem(hardwareMap);
        lower = new LowerSubsystem(hardwareMap);


        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        curAct = 0;
        pivotDouble = 0.26;
        lowerGrabPos = 1;
        follower.startTeleopDrive();
        lower.setInches(0);
        extended = false;
        lower.wristPos(1);
        lower.release();

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    private double timer;
    private double actionTimer;


    //private boolean feedforward = false;
    private boolean autoRetract = false;

    /* Tele Enhancements
        1-2-3-4 pickup to score
        5 score
        6-7 close claw
     */
    public void telePathUpdate() {
        switch (curAct) {
            case 0:
                break;
            case 1:
                if (lower.getPivotPosition() == .6) {
                    curAct = 3;
                } else {
                    curAct = 2;
                }
                lower.wristPos(0);
                timer = opmodeTimer.getElapsedTimeSeconds();
                break;
            case 2:
                if (opmodeTimer.getElapsedTimeSeconds() - timer > .35) {
                    lower.grab();
                    curAct = 5;
                    timer = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 3:
                if (opmodeTimer.getElapsedTimeSeconds() - timer > .35) {
                    lower.setPivot(0.55);
                    timer = opmodeTimer.getElapsedTimeSeconds();
                    curAct = 4;
                }
                break;
            case 4:
                if (opmodeTimer.getElapsedTimeSeconds() - timer > .4) {
                    lower.grab();
                    timer = opmodeTimer.getElapsedTimeSeconds();
                    curAct = 5;
                }
                break;
            case 5:
                if (opmodeTimer.getElapsedTimeSeconds() - timer > .36) {
                    lower.wristPos(.5);
                    lower.setPivot(.26);
                    lower.setInches(0);
                    extended = false;
                    curAct = 0;
                }
                break;
            case 6:
                upper.closeClaw();
                timer = opmodeTimer.getElapsedTimeSeconds();
                curAct = 7;
                break;
            case 7:
                if (opmodeTimer.getElapsedTimeSeconds() - timer > .3) {
                    upper.up();
                    follower.setPose(scorePose);
                    follower.followPath(pathx);
                    curAct = 8;
                }
                break;
            case 8:
                if (follower.getPose().getX() > 39) {
                    upper.score();
                    curAct = 9;
                }
                break;
            case 9:
                if (follower.getPose().getX() < 37) {
                    upper.pickup();
                    follower.startTeleopDrive();
                    curAct = 0;
                }
                break;



                /*
            case 1:
                follower.setPose(startPose);
                //claw.closeClaw();
                timer = opmodeTimer.getElapsedTimeSeconds();
                actionTimer = opmodeTimer.getElapsedTimeSeconds();
                curAct = 2;
                break;
            case 2:
                if(claw.isClosed() && opmodeTimer.getElapsedTimeSeconds() - timer > 0.35) { //0.35
                    //upper.goUp();
                    //stepTimes[curStep++] = opmodeTimer.getElapsedTimeSeconds();
                    follower.followPath(path1);
                    curAct = 4;
                }
                break;
            case 3:
                break;
            case 4:
                if(follower.getPose().getX() > (37)) {
                    /*follower.breakFollowing();
                    follower.startTeleopDrive();
                    follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                    curAct = 0;
                    feedforward = false;*//*
                    follower.startTeleopDrive();
                    curAct = 0;
                }
                break;
            case 5:
                break;
            case 6: //Close claw
                if (claw.isClosed()) claw.openClaw();
                else claw.closeClaw();
                timer = opmodeTimer.getElapsedTimeSeconds();
                actionTimer = opmodeTimer.getElapsedTimeSeconds();
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                curAct = 7;
                break;
            case 7:
                if(opmodeTimer.getElapsedTimeSeconds() - timer > 0.35) {
                    curAct = 0;
                }
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                break;
            case 8:
                claw.openClaw();
                actionTimer = opmodeTimer.getElapsedTimeSeconds();
                curAct = 9;
                break;
            case 9:
                follower.setPose(scorePose);
                follower.followPath(path2);
                curAct = 10;
                break;
            case 10:
                if(follower.getPose().getX() < (13)) {
                    follower.startTeleopDrive();
                    curAct = 0;
                }
                break;
            case 11:
                upper.up();
                curAct = 13;
                break;
            case 13: //Close lower claw
                if (lower.closed()) lower.release();
                else {
                    lower.grab();
                }
                timer = opmodeTimer.getElapsedTimeSeconds();
                actionTimer = opmodeTimer.getElapsedTimeSeconds();
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                curAct = 0;
                break;
            case 14:
                follower.setPose(sidePose);
                follower.followPath(path3);
                curAct = 15;
                break;
            case 15:
                if(follower.getPose().getX() < 40) {
                    lower.release();
                    curAct = 16;
                }
                break;
            case 16:
                if(follower.getPose().getX() < 17) {
                    follower.followPath(path4);
                    curAct = 17;
                }
                break;
            case 17:
                if(follower.getPose().getX() > 65) {
                    follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                    curAct = 0;
                }
                break;
            case 18:
                follower.setPose(scorePose);
                follower.followPath(path5);
                curAct = 19;
                break;
            case 19:
                if(follower.getPose().getY() < (30)) {
                    lower.release();
                    lowerGrabPos = 0.5;
                    curAct = 0;
                }
                break;
            case 20:
                if(follower.getPose().getX() < (13.5)) {
                    follower.startTeleopDrive();
                    curAct = 0;
                }
                break;
            case 21:
                follower.setPose(startPose);
                claw.closeClaw();
                timer = opmodeTimer.getElapsedTimeSeconds();
                actionTimer = opmodeTimer.getElapsedTimeSeconds();
                curAct = 22;
            break;
            case 22:
                if(opmodeTimer.getElapsedTimeSeconds() - timer > 0.35) { //0.35
                    upper.up();
                    //stepTimes[curStep++] = opmodeTimer.getElapsedTimeSeconds();
                    follower.followPath(path1, true);
                    curAct = 23;
                }
                break;
            case 23:
                curAct = 24;
                break;
            case 24:
                if(follower.getPose().getX() > (37)) {
                    curAct = 8;
                }
                break;
            case 25:
                upper.score();
                timer = opmodeTimer.getElapsedTimeSeconds();
                curAct = 26;
                break;
            case 26:
                if(opmodeTimer.getElapsedTimeSeconds() - timer > 0.2) {
                    curAct = 0;

                }
                break;*/

                /*

            case 14:
                if(opmodeTimer.getElapsedTimeSeconds() - timer > 0.2) {
                    curAct = 0;
                    if (lower.closed()) {
                        //lower.raise();
                    }
                }
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                break;*/
            /*case 11:
                lower.extend();
                if(lower.getPosition() < -1800){
                    curAct = 12;
                }
                break;
            case 12:
                lower.retract();
                curAct = 0;
                break;*/
        }
    }
    @Override
    public void loop() {

        /*
        if(gamepad1.dpad_left) { //Drive Enhancments
            if(gamepad1.x && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.2){
                pivotDouble = 0.62;
                curAct = 1;

            }
            if(gamepad1.a && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.1) curAct = 8;
        } else if (gamepad1.dpad_right){
            if (gamepad1.a) curAct = 18;
            if (gamepad1.x){
                pivotDouble = 0.62;
                curAct = 21;
            }

        }

        else {
            if(gamepad1.x && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.2) curAct = 6;
            if(gamepad1.b && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.1) curAct = 11;
            if (gamepad1.a) curAct = 5;
        }
        if(gamepad1.dpad_up && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.2) {
            if(lowerGrabPos < 1) {

                lowerGrabPos+=0.5;
                actionTimer = opmodeTimer.getElapsedTimeSeconds();
            }
        }

        if(gamepad1.dpad_down && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.2) {
            if(lowerGrabPos > 0){
                lowerGrabPos-=0.5;
                actionTimer = opmodeTimer.getElapsedTimeSeconds();
            }
        }

        lower.wristPos(lowerGrabPos);


        if(gamepad1.y){
            curAct = 14;
        }

        if (gamepad1.right_stick_button && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.35) {
            curAct = 13;
        }

        if(gamepad1.start){
            curAct = 0;
            follower.breakFollowing();
            follower.startTeleopDrive();
        }



        if (feedforward) {
            if (upper.getHeight() > 600) {
                upper.smallDown();
            } else if (upper.getHeight() > 540) {
                upper.stayUp();
            } else if (upper.getHeight() <= 540) {
                upper.goUp();
            }
        }*/
        //if(gamepad1.back) follower.setStartingPose(scorePose);

        if (gamepad1.y){
            upper.pickup();
        } else if(gamepad1.b){
            upper.closeClaw();
        } else if(gamepad1.a){
            upper.up();
        } else if(gamepad1.x){
            upper.score();
        }
        if (gamepad1.left_trigger > .1 ){
            lower.setInches(0);
            extended = false;
        }
        if (gamepad1.right_trigger > .1){
            lower.setInches(13.7);
            lower.wristPos(0.5);
            wristpos = 0.5;
            extended = true;
        }

        if (gamepad1.back) curAct = 6;

        /*
        if(gamepad1.left_bumper){
            pivotDouble += 0.02;
        } else if(gamepad1.right_bumper){
            pivotDouble -= 0.02;
        }
        if(pivotDouble < 0)pivotDouble = 0;
        if(pivotDouble > 1)pivotDouble = 1;
        lower.setPivot(pivotDouble);*/

/*
        if(gamepad1.left_bumper){
            wristpos += 0.02;
        } else if(gamepad1.right_bumper){
            wristpos -= 0.02;
        }upper.setWrist(wristpos);*/

        /*
        if(gamepad1.a){
            armpos += 0.02;
        } if(gamepad1.b){
            armpos -= 0.02;
        }upper.setArm(armpos);
         //wrist 0.24 0.44drive in
        if(gamepad1.y){
            upper.closeClaw();
        } else if(gamepad1.x){
            upper.openClaw();
        }*/

        if(gamepad1.dpad_up)lower.wristPos(1);
        else if(gamepad1.dpad_right)lower.wristPos(0.5);
        if (gamepad1.dpad_down) curAct = 1;
        if(gamepad1.dpad_left)lower.grab();
        if(gamepad1.start)lower.release();

        if (lower.seePiece() && extended) {
            curAct = 1;
        }


        if (gamepad1.left_stick_button) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
        } else {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
        }
        telePathUpdate();
        follower.update();

        telemetry.addData("curAct", curAct);
        //telemetry.addData("Feedforward", feedforward);
        telemetry.addData("Wrist", wristpos);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Pivot Pos", lower.getPivotPosition());
        telemetry.addData("Arm Pos", armpos);
        telemetry.addData("sees piece ", lower.seePiece());

    }

}
