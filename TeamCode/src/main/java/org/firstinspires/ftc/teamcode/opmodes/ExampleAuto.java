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
import org.firstinspires.ftc.teamcode.subsystems.PositionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperSubsystem;

import java.util.Arrays;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "ExampleAuto", group = "Autonomous")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private boolean poseSet = false;
    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    public ClawSubsystem claw;
    public UpperSubsystem upper;
    public PositionSubsystem pos;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(10, 72, 0);

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
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;
    //private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    private double[][] poses = new double[5][3];
    private int curI = 0;
    public double lastX = 0;
    public int median = -1;
    private boolean timerStarted = false;
    private double timer;
    private boolean scored = false;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    //Pose variables
    public static final double localX = 20; //localizing x //14.958
    public static final double localY = 32; //28.935
    public static final double pickup2x = 12;//12.875;//11.458; //Up against wall to pickup
    public static final double pickupy = 28.935;
    public static final double scorex = 38.131;
    public static final double startx = 10.000; //starting x
    public static final double starty = 72.000;
    public static final double increase = 1.5; //How far apart each spec is placed
    public void buildPaths() {
        path1 = follower.pathBuilder() //Start to score
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(startx, starty, Point.CARTESIAN),
                                new Point(scorex, starty, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        path2 = follower.pathBuilder() //score to push 1 into pickup
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(scorex, starty, Point.CARTESIAN),
                                new Point(21.084, 49.570, Point.CARTESIAN),
                                new Point(29.832, 34.093, Point.CARTESIAN),
                                new Point(44.411, 35.888, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(44.411, 35.888, Point.CARTESIAN),
                                new Point(56.075, 35.664, Point.CARTESIAN),
                                new Point(57.421, 24.449, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(57.421, 24.449, Point.CARTESIAN),
                                new Point(22.430, 23.551, Point.CARTESIAN),
                                new Point(21.981, 13.234, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-160))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(21.981, 13.234, Point.CARTESIAN),
                                new Point(32.299, 25.794, Point.CARTESIAN),
                                new Point(localX, localY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(-160),
                         Math.toRadians(-180)
                )
                .build();
        path3 = follower.pathBuilder() //move forward pickup
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(localX, 28.935, Point.CARTESIAN),
                                new Point(pickup2x, 28.935, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(-180),
                        Math.toRadians(-180)
                )
                .build();
        path4 = follower.pathBuilder() //pickup to score
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(pickup2x, 28.935, Point.CARTESIAN),
                                new Point(35.888, 38.579, Point.CARTESIAN),
                                new Point(14.579, 68.411, Point.CARTESIAN),
                                new Point(scorex, starty+increase, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(-180),
                        Math.toRadians(0)
                )
                .build();
        path5 = follower.pathBuilder() //score to push 2 into pickup
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(scorex, starty+increase, Point.CARTESIAN),
                                new Point(21.084, 49.570, Point.CARTESIAN),
                                new Point(29.832, 34.093, Point.CARTESIAN),
                                new Point(44.411, 35.888, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(44.411, 35.888, Point.CARTESIAN),
                                new Point(56.075, 35.664, Point.CARTESIAN),
                                new Point(58.318, 17.579, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(58.318, 17.579, Point.CARTESIAN),
                                new Point(18.234, 12.888, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-160))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(18.234, 12.888, Point.CARTESIAN),
                                new Point(32.299, 25.794, Point.CARTESIAN),
                                new Point(localX, localY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(-160),
                        Math.toRadians(-180)
                )
                .build();
        path6 = follower.pathBuilder() //pickup to score
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(pickup2x, localY, Point.CARTESIAN),
                                new Point(33.196, 29.832, Point.CARTESIAN),
                                new Point(14.131, 77.159, Point.CARTESIAN),
                                new Point(scorex, starty+(2*increase), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
                .build();
        path7 = follower.pathBuilder() //score to push 3 to pickup
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(scorex, starty+(2*increase), Point.CARTESIAN),
                                new Point(21.084, 49.570, Point.CARTESIAN),
                                new Point(29.159, 29.383, Point.CARTESIAN),
                                new Point(66.841, 31.402, Point.CARTESIAN),
                                new Point(58.318, 9.196, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-180))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(58.318, 9.196, Point.CARTESIAN),
                                new Point(23.103, 10.766, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(23.103, 10.766, Point.CARTESIAN),
                                new Point(32.299, 25.794, Point.CARTESIAN),
                                new Point(localX, localY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(-180),
                        Math.toRadians(-180)
                )
                .build();
        path8 = follower.pathBuilder() //pickup to score
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(pickup2x, localY, Point.CARTESIAN),
                                new Point(33.196, 29.832, Point.CARTESIAN),
                                new Point(14.131, 77.159, Point.CARTESIAN),
                                new Point(scorex, starty+(3*increase), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
                .build();
        path9 = follower.pathBuilder() //score to pickup
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(scorex, starty+(3*increase), Point.CARTESIAN),
                                new Point(6.953, 79.178, Point.CARTESIAN),
                                new Point(32.299, 29.832, Point.CARTESIAN),
                                new Point(pickup2x, localY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
                .build();
        path10 = follower.pathBuilder() //pickup to score
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(pickup2x, localY, Point.CARTESIAN),
                                new Point(33.196, 29.832, Point.CARTESIAN),
                                new Point(14.131, 77.159, Point.CARTESIAN),
                                new Point(scorex, starty+(4*increase), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
                .build();
        path11 = follower.pathBuilder() //park
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(scorex, starty+(4*increase), Point.CARTESIAN),
                                new Point(13.234, 10.318, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(-180),
                        Math.toRadians(-180)
                )
                .build();
    }



    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public static final int[] caseOrder = { 0,12,1,
                                            2,3,4,5,12,1,
                                            6,3,4,7,12,1, //6 b
                                            8,3,4,9,12,1,
                                            10,1};
    public int curCase = 0;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //start position to score
                follower.followPath(path1, true);
                upper.goUp();
                setPathState(caseOrder[curCase++]);
                break;
            case 1: //Score Case
                if(follower.getPose().getX() > (38.131 - 1) && !follower.isBusy()) {
                    if (upper.getHeight() > RobotConstants.barHeight) {
                        upper.scoreDown();
                    } else {
                        claw.openClaw();
                        setPathState(caseOrder[curCase++]);
                    }
                }
                break;
            case 2: //push to OZ
                follower.followPath(path2, true);
                if(upper.getHeight() < 15) {
                    upper.off();
                    setPathState(caseOrder[curCase++]);
                }
            case 3: //Relocalize
                scored = false;
                if(follower.getPose().getX() < (localX + 0.252) && !follower.isBusy() && pos.validResult() && pos.getX() != lastX && poses[4][0] == 0){
                    poses[curI] = new double[]{pos.getX(), pos.getY(), Math.toRadians(pos.getYaw())};
                    lastX = poses[curI][0];
                    curI++;
                }
                if(poses[4][0] != 0 && median == -1){
                    double[] arr = {poses[0][0], poses[1][0], poses[2][0], poses[3][0], poses[4][0]};
                    Arrays.sort(arr);
                    for(int i = 0; i < 5; i++){
                        if(poses[i][0] == arr[2]) median = i;
                    }
                }
                if(median != -1){
                    follower.setPose(new Pose(poses[median][0], poses[median][1], poses[median][2]));
                    follower.followPath(path3, true);
                    median = -1;
                    poses = new double[5][3];
                    lastX = 0;
                    curI = 0;
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 4: //Pickup off wall
                if (follower.getPose().getX() < pickup2x+0.25) {
                    claw.closeClaw();
                    if(!timerStarted){
                        timerStarted = true;
                        timer = pathTimer.getElapsedTimeSeconds();
                    }
                }
                if(claw.isClosed() && (pathTimer.getElapsedTimeSeconds() - timer > 0.35)) {
                    upper.goUp();
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 5://p2s
                if(follower.isBusy()) follower.followPath(path4, true);
                setPathState(caseOrder[curCase++]);
                break;
            case 6: //push to OZ
                follower.followPath(path5, true);
                if(upper.getHeight() < 15) {
                    upper.off();
                    setPathState(caseOrder[curCase++]);
                }
                break;

            case 7://p2s
                follower.followPath(path6, true);
                setPathState(caseOrder[curCase++]);
                break;
            case 8: //push to OZ
                follower.followPath(path7, true);
                if(upper.getHeight() < 15) {
                    upper.off();
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 9://p2s
                follower.followPath(path8, true);
                setPathState(caseOrder[curCase++]);
                break;
            case 10: //score to pickup
                follower.followPath(path9, true);
                if(upper.getHeight() < 15) {
                    upper.off();
                    setPathState(caseOrder[curCase++]);
                }
                break;
            case 11: //p2s
                follower.followPath(path10, true);
                setPathState(caseOrder[curCase++]);
                break;
            case 12: //stay up
                if(upper.getHeight() > 600){
                    upper.stayUp();
                    setPathState(caseOrder[curCase++]);
                }
                break;
                /*
            case 13:
                if(follower.getPose().getX() > (38.131 - 1)) {
                    if(upper.getHeight() > RobotConstants.barHeight) {
                        upper.scoreDown();
                    }
                    else {
                        claw.openClaw();
                        //follower.followPath(path2, true);
                    }
                    if(upper.getHeight() < 15) scored = true;

                }
                if (scored) {
                    upper.off();
                    setPathState(caseOrder[curCase++]);
                }
                break;*/
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Upper Height", upper.getHeight());
        telemetry.addData("Path timing", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("distance", pos.getDistanceLeft());
        telemetry.addData("upper height", upper.getHeight());
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