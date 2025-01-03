package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PositionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperSubsystem;

import java.util.concurrent.TimeUnit;
import java.util.Arrays;

@Autonomous(name = "TestingAuto", group = "Autonomous")
public class TestingAuto extends OpMode {

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
    private final Pose startPose = new Pose(8.875, 72, 0);

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
    private PathChain path1, path2, path3, path4;

    private double[][] poses = new double[5][3];
    private double[] curResult = new double[3];
    private int curI = 0;
    public double lastX = 0;
    public int median = -1;
    private boolean timerStarted = false;
    private double timer;
    private boolean scored = false;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    //Pose variables
    public void buildPaths() {
        path1 = follower.pathBuilder() //Start to score
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(8.875, 72.000, Point.CARTESIAN),
                                new Point(38.131, 72.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        path2 = follower.pathBuilder() //score to push 1 into pickup
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(38.131, 72.000, Point.CARTESIAN),
                                new Point(20, 32.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-180))
                .build();
        path3 = follower.pathBuilder() //move forward pickup
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(20, 28.935, Point.CARTESIAN),
                                new Point(12.875, 28.935, Point.CARTESIAN)
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
                                new Point(12.875, 28.935, Point.CARTESIAN),
                                new Point(41.495, 22.879, Point.CARTESIAN),
                                new Point(8.299, 75.589, Point.CARTESIAN),
                                new Point(38.131, 73.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
                .build();
    }



    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
                upper.goUp();
                setPathState(1);
                break;
            case 1:
                if(upper.getHeight() > 600){
                    upper.stayUp();
                    setPathState(2);
                }
                break;
            case 2:
                if(follower.getPose().getX() > (38.131 - 1)) {
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
                    setPathState(3);
                }
                break;
            case 3: // Relocalize
                scored = false;
                if(follower.getPose().getX() < (20.25) && !follower.isBusy() && pos.validResult() && pos.getX() != lastX && poses[4][0] == 0){
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
                    setPathState(4);
                }
                break;
            case 4:
                if (follower.getPose().getX() < 13) {
                    claw.closeClaw();
                    if(!timerStarted){
                        timerStarted = true;
                        timer = pathTimer.getElapsedTimeSeconds();
                    }
                }
                if(claw.isClosed() && (pathTimer.getElapsedTimeSeconds() - timer > 0.35)) {
                    upper.goUp();
                    //follower.setPose(new Pose(8.875, 28.935, -180));
                    follower.followPath(path4, true);
                    setPathState(5);
                }
                break;
            case 5:
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
                    setPathState(-1);
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Upper Height", upper.getHeight());
        telemetry.addData("Path timing", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("distance", pos.getDistance());
        telemetry.addData("upper height", upper.getHeight());
        if(pos.validResult()){
            telemetry.addData("llxyhead", "lly", pos.getX(), pos.getY(), Math.toRadians(pos.getYaw()));            telemetry.addLine("x " + pos.getX() + " y " + pos.getY() + " yaw " + Math.toRadians(pos.getYaw()));
        }
        telemetry.addData("median", median);
        telemetry.addLine("x " + poses[0][0] + " y " + poses[0][1] + " yaw " + poses[0][2]);
        telemetry.addLine("x " + poses[1][0] + " y " + poses[1][1] + " yaw " + poses[1][2]);
        telemetry.addLine("x " + poses[2][0] + " y " + poses[2][1] + " yaw " + poses[2][2]);
        telemetry.addLine("x " + poses[3][0] + " y " + poses[3][1] + " yaw " + poses[3][2]);
        telemetry.addLine("x " + poses[4][0] + " y " + poses[4][1] + " yaw " + poses[4][2]);
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