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
    private final Pose scorePose = new Pose(42.631, 73.500, Math.toRadians(0));
    private Follower follower;
    public UpperSubsystem upper;
    public ClawSubsystem claw;
    public LowerSubsystem lower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private PathChain path1, path2;
    private Timer opmodeTimer;
    public void buildPaths() {
        path1 = follower.pathBuilder() //pickup to score
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(10.875, 28.935, Point.CARTESIAN),
                                new Point(42.631, 73.500, Point.CARTESIAN)
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
                                new Point(42.631, 73.500, Point.CARTESIAN),
                                new Point(10.875, 28.935, Point.CARTESIAN)

                                //new Point(41.495, 22.879, Point.CARTESIAN),
                                //new Point(8.299, 75.589, Point.CARTESIAN),
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
    }
    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    public int curAct;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
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
        follower.startTeleopDrive();

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    private double timer;
    private double actionTimer;

    private boolean feedforward = false;

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
                follower.setPose(startPose);
                claw.closeClaw();
                timer = opmodeTimer.getElapsedTimeSeconds();
                actionTimer = opmodeTimer.getElapsedTimeSeconds();
                curAct = 2;
                break;
            case 2:
                if(opmodeTimer.getElapsedTimeSeconds() - timer > 0.35) { //0.35
                    upper.goUp();
                    //stepTimes[curStep++] = opmodeTimer.getElapsedTimeSeconds();
                    follower.followPath(path1);
                    curAct = 3;
                }
                break;
            case 3:
                if(upper.getHeight() > 540){
                    upper.stayUp();
                    feedforward = true;
                    curAct = 4;
                }
                break;
            case 4:
                if(follower.getPose().getX() > (42)) {
                    follower.startTeleopDrive();
                    curAct = 0;
                    feedforward = false;
                }
                break;
            case 5:
                if(upper.getHeight() > RobotConstants.barHeight) {
                    upper.scoreDown();

                }
                else {
                    claw.openClaw();
                    curAct = 0;
                }
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
                if(upper.getHeight() > RobotConstants.barHeight) {
                    upper.scoreDown();
                }
                else {
                    claw.openClaw();
                    actionTimer = opmodeTimer.getElapsedTimeSeconds();
                    curAct = 9;
                }
                break;
            case 9:
                follower.followPath(path2);
                curAct = 10;
                break;
            case 10:
                if(follower.getPose().getX() < (13)) {
                    follower.startTeleopDrive();
                    curAct = 0;
                }
                break;
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

        if(upper.getHeight() < 15 && !upper.isUp()) upper.off();
        if(lower.getPosition() > -50 && !lower.out()) lower.bottomon();
        if(!gamepad1.dpad_left) { //Drive Enhancments
            if(gamepad1.x && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.1) curAct = 1;
            if(gamepad1.a && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.1) curAct = 8;
        } else {
            if(gamepad1.x && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.1) curAct = 6;
            if (gamepad1.a) curAct = 5;
        }
        /*if (gamepad1.y) {
            curAct = 11;
        }*/
        if(gamepad1.dpad_down) {
            lower.raise();
        }

        if(gamepad1.dpad_up) {
            lower.lower();
        }

        if (gamepad1.dpad_right && opmodeTimer.getElapsedTimeSeconds() - actionTimer > 0.35) {
            if (lower.closed()) lower.release();
            else lower.grab();
            actionTimer = opmodeTimer.getElapsedTimeSeconds();
        }

        if (gamepad1.left_trigger > .2 && lower.getPosition() > -1800) {
            lower.extend();
        } else if (gamepad1.right_trigger > .2 ) {
            lower.retract();
        } else if (gamepad1.left_bumper && lower.getPosition() > -1800) {
            lower.slowExtend();
        } else if (gamepad1.right_bumper) {
            lower.slowRetract();
        } else if (lower.getPosition() < -50) {
            lower.bottomoff();
        }

        if(gamepad1.back){
            curAct = 0;
            follower.breakFollowing();
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }
        /*
        if (feedforward) {
            if (upper.getHeight() > 600) {
                upper.smallDown();
            } else if (upper.getHeight() > 540) {
                upper.stayUp();
            } else if (upper.getHeight() <= 540) {
                upper.goUp();
            }
        }*/

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        telePathUpdate();
        follower.update();

        telemetry.addData("curAct", curAct);
        telemetry.addData("Height", upper.getHeight());
        telemetry.addData("Feedforward", feedforward);

    }

}
