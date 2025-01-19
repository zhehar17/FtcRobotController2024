package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PositionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperSubsystem;

import java.util.Arrays;

@Autonomous(name = "ThreeAuto", group = "Autonomous")
public class ThreeAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private boolean poseSet = false;
    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    public ClawSubsystem claw;
    public UpperSubsystem upper;
    public PositionSubsystem pos;
    public LowerSubsystem lower;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 66, 0);

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    //private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    //private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    //private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    //private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    //private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    //private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    //private Path scorePreload, park;
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;

    private boolean timerStarted = false;
    private double timer;
    private boolean scored = false;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    //Pose variables
    private double pickupX = 10; //9
    private double pickupY = 20; //20
    private double scoreX = 37;
    public void buildPaths() {
        path1 = follower.pathBuilder() //Start to score
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9, 66, Point.CARTESIAN),
                                new Point(scoreX, 66, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        path2 = follower.pathBuilder() //score to push 1 into pickup
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(scoreX, 66.000, Point.CARTESIAN),
                                new Point(17.000, 26.000, Point.CARTESIAN),
                                new Point(58.000, 37.000, Point.CARTESIAN),
                                new Point(58.000, 28.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(200))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(58.000, 28.000, Point.CARTESIAN),
                                new Point(22.000, 16.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(200))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(22.000, 16.000, Point.CARTESIAN),
                                new Point(58.000, 33.000, Point.CARTESIAN),
                                new Point(58.000, 18.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(200))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(58.000, 18.000, Point.CARTESIAN),
                                new Point(23.327, 11.664, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(200))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(23.327, 11.664, Point.CARTESIAN),
                                new Point(30.000, 22.000, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(180))
                .build();
        path3 = follower.pathBuilder() //move forward pickup
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(scoreX+1, 68.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        path4 = follower.pathBuilder() //pickup to score
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(scoreX+1, 68.000, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        path5 = follower.pathBuilder() //move forward pickup
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(scoreX+2, 70.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        path6 = follower.pathBuilder() //pickup to score
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(scoreX+2, 70.000, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        path7 = follower.pathBuilder() //move forward pickup
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(scoreX+3, 72.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        path8 = follower.pathBuilder() //pickup to score
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(scoreX+3 , 72.000, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }



    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public int[] caseOrder = {0,1,2,3,4,5,6,7,8,9,-1};
    public int curCase = 0;
    public boolean timerStep = false;
    //public boolean localized = false;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0://move forward
                follower.followPath(path1, true);
                upper.goUp();
                setPathState(caseOrder[curCase++]);
                break;
            case 1://hold up
                if(upper.getHeight() > 540){
                    upper.stayUp();
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 2://score then move
                if(follower.getPose().getX() > (36.5)) {
                    if(upper.getHeight() > RobotConstants.barHeight) {
                        upper.scoreDown();
                    }
                    else {
                        claw.openClaw();
                        follower.followPath(path2, true);
                    }
                    if(upper.getHeight() < 15) scored = true;

                }
                if (scored) {
                    upper.off();
                    scored = false;
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 3: //pickup and score
                if (follower.getPose().getX() < 10) {
                    claw.closeClaw();
                    if (!timerStarted) {
                        timerStarted = true;
                        timer = pathTimer.getElapsedTimeSeconds();
                    }
                }
                if(claw.isClosed() && (pathTimer.getElapsedTimeSeconds() - timer > 0.35)) {
                    upper.goUp();
                    //stepTimes[curStep++] = opmodeTimer.getElapsedTimeSeconds();
                    follower.followPath(path3, true);
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 4: //score to pickup
                if(follower.getPose().getX() > (37.5)) {
                    if(upper.getHeight() > RobotConstants.barHeight) {
                        upper.scoreDown();
                    }
                    else {
                        claw.openClaw();
                        if(!timerStep) {
                            //stepTimes[curStep++] = opmodeTimer.getElapsedTimeSeconds();
                            timerStep = true;
                            //localized = false;
                            follower.followPath(path4, true);
                        }
                    }
                    if(upper.getHeight() < 15) scored = true;
                } else if(upper.getHeight() > 540){
                    upper.stayUp();
                }
                if (scored) {
                    upper.off();
                    scored = false;
                    timerStep = false;
                    timerStarted = false;
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 5: //pickup to score
                if (follower.getPose().getX() < 10) {
                    claw.closeClaw();
                    if (!timerStarted) {
                        timerStarted = true;
                        timer = pathTimer.getElapsedTimeSeconds();
                    }
                }
                if(claw.isClosed() && (pathTimer.getElapsedTimeSeconds() - timer > 0.35)) {
                    upper.goUp();
                    //stepTimes[curStep++] = opmodeTimer.getElapsedTimeSeconds();
                    follower.followPath(path5, true);
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 6:
                if(follower.getPose().getX() > (38.5)) {
                    if(upper.getHeight() > RobotConstants.barHeight) {
                        upper.scoreDown();
                    }
                    else {
                        claw.openClaw();
                        if(!timerStep) {
                            //stepTimes[curStep++] = opmodeTimer.getElapsedTimeSeconds();
                            timerStep = true;
                            //localized = false;
                            follower.followPath(path6, true);
                        }
                    }
                    if(upper.getHeight() < 15) scored = true;
                } else if(upper.getHeight() > 540){
                    upper.stayUp();
                }
                if (scored) {
                    upper.off();
                    scored = false;
                    timerStep = false;
                    timerStarted = false;
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 7: //score to pickup
                if (follower.getPose().getX() < 10) {
                    claw.closeClaw();
                    if (!timerStarted) {
                        timerStarted = true;
                        timer = pathTimer.getElapsedTimeSeconds();
                    }
                }
                if(claw.isClosed() && (pathTimer.getElapsedTimeSeconds() - timer > 0.35)) {
                    upper.goUp();
                    //stepTimes[curStep++] = opmodeTimer.getElapsedTimeSeconds();
                    follower.followPath(path7, true);
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 8:
                if(follower.getPose().getX() > (39.5)) {
                    if(upper.getHeight() > RobotConstants.barHeight) {
                        upper.scoreDown();
                    }
                    else {
                        claw.openClaw();
                        if(!timerStep) {
                            //stepTimes[curStep++] = opmodeTimer.getElapsedTimeSeconds();
                            timerStep = true;
                            //localized = false;
                            follower.followPath(path8, true);
                        }
                    }
                    if(upper.getHeight() < 15) scored = true;
                } else if(upper.getHeight() > 540){
                    upper.stayUp();
                }
                if (scored) {
                    upper.off();
                    scored = false;
                    timerStep = false;
                    timerStarted = false;
                    setPathState(caseOrder[curCase++]);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        //telemetry.addData("case state", caseOrder[curCase]);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Upper Height", upper.getHeight());
        telemetry.addData("Path timing", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("upper height", upper.getHeight());
        //telemetry.addData("submersibleX", pos.getPoseXSub());
        //telemetry.addData("submersibleHeading", pos.getDistanceHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        claw = new ClawSubsystem(hardwareMap);
        upper = new UpperSubsystem(hardwareMap);
        pos = new PositionSubsystem(hardwareMap);
        lower = new LowerSubsystem(hardwareMap);

        lower.bottomon();
        claw.closeClaw();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}