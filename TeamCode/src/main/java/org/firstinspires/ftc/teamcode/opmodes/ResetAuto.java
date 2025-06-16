//package org.firstinspires.ftc.teamcode.opmodes;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.RobotConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
//import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.PositionSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.UpperSubsystem;
//
//import java.util.Arrays;
//
//@Autonomous(name = "Reset", group = "Autonomous")
//public class ResetAuto extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//    private boolean poseSet = false;
//    /** This is the variable where we store the state of our auto.
//     * It is used by the pathUpdate method. */
//    private int pathState;
//
//    public ClawSubsystem claw;
//    public UpperSubsystem upper;
//    public PositionSubsystem pos;
//    public LowerSubsystem lower;
//
//    /** Create and Define Poses + Paths
//     * Poses are built with three constructors: x, y, and heading (in Radians).
//     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
//     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
//     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
//     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
//     * Lets assume our robot is 18 by 18 inches
//     * Lets assume the Robot is facing the human player and we want to score in the bucket */
//
//    /** Start Pose of our robot */
//    private final Pose startPose = new Pose(8.875, 72, 0);
//
//    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
//    //private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));
//
//    /** Lowest (First) Sample from the Spike Mark */
//    //private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));
//
//    /** Middle (Second) Sample from the Spike Mark */
//    //private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));
//
//    /** Highest (Third) Sample from the Spike Mark */
//    //private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));
//
//    /** Park Pose for our robot, after we do all of the scoring. */
//    //private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));
//
//    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
//     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
//    //private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));
//
//    /* These are our Paths and PathChains that we will define in buildPaths() */
//    //private Path scorePreload, park;
//    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;
//
//    private double[][] poses = new double[5][3];
//    private double[] curResult = new double[3];
//    private double[] wallPosesLeft = new double[4];
//    private double[] wallPosesRight = new double[4];
//    private int curI = 0;
//    private int curLeft = 0;
//    private int curRight = 0;
//    public double lastX = 0;
//    public int median = -1;
//    private boolean timerStarted = false;
//    private double timer;
//    private boolean scored = false;
//    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
//     * It is necessary to do this so that all the paths are built before the auto starts. **/
//    //Pose variables
//    public void buildPaths() {
//
//    }
//
//
//
//    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
//     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
//     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
//    public int[] caseOrder = {0,1,2,3,4,5,10,6,7,10,8,9};
//    public int curCase = 0;
//    public double[] stepTimes = new double[50];
//    public int curStep = 0;
//    public boolean timerStep = false;
//
//    private double startTime;
//    //public boolean localized = false;
//
//
//    /** These change the states of the paths and actions
//     * It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot
//        follower.update();
//
//        // Feedback to Driver Hub
//        telemetry.addData("path state", pathState);
//        //telemetry.addData("case state", caseOrder[curCase]);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("Upper Height", upper.getHeight());
//        telemetry.addData("Path timing", pathTimer.getElapsedTimeSeconds());
//        telemetry.addData("upper height", upper.getHeight());
//        //telemetry.addData("submersibleX", pos.getPoseXSub());
//        //telemetry.addData("submersibleHeading", pos.getDistanceHeading());
//        telemetry.addData("time 1:", stepTimes[0]);
//        telemetry.addData("time 2 (push3 & pickup):", stepTimes[1] - stepTimes[0]);
//        telemetry.addData("time 3 (p2s)", stepTimes[2]-stepTimes[1]);
//        telemetry.addData("time 4 (s2p)", stepTimes[3]-stepTimes[2]);
//        telemetry.addData("time 5 (p2s)", stepTimes[4]-stepTimes[3]);
//        telemetry.addData("time 6 (s2p)", stepTimes[5]-stepTimes[4]);
//        telemetry.addData("time 7 (p2s)", stepTimes[6]-stepTimes[5]);
//        telemetry.addData("time 8 (s2p)", stepTimes[7]-stepTimes[6]);
//        telemetry.addData("time 9 (p2s)", stepTimes[8]-stepTimes[7]);
//        if(pos.validResult()){
//            telemetry.addData("llxyhead", "lly", pos.getX(), pos.getY(), Math.toRadians(pos.getYaw()));            telemetry.addLine("x " + pos.getX() + " y " + pos.getY() + " yaw " + Math.toRadians(pos.getYaw()));
//        }
//        telemetry.addData("median", median);
//        telemetry.addLine("x " + poses[0][0] + " y " + poses[0][1] + " yaw " + poses[0][2]);
//        telemetry.addLine("x " + poses[1][0] + " y " + poses[1][1] + " yaw " + poses[1][2]);
//        telemetry.addLine("x " + poses[2][0] + " y " + poses[2][1] + " yaw " + poses[2][2]);
//        telemetry.addLine("x " + poses[3][0] + " y " + poses[3][1] + " yaw " + poses[3][2]);
//        telemetry.addLine("x " + poses[4][0] + " y " + poses[4][1] + " yaw " + poses[4][2]);
//        telemetry.update();
//    }
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//
//        opmodeTimer.resetTimer();
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        buildPaths();
//
//        claw = new ClawSubsystem(hardwareMap);
//        upper = new UpperSubsystem(hardwareMap);
//        pos = new PositionSubsystem(hardwareMap);
//        lower = new LowerSubsystem(hardwareMap);
//
//        lower.bottomon();
//        claw.closeClaw();
//
//        opmodeTimer.resetTimer();
//    }
//
//    /** This method is called continuously after Init while waiting for "play". **/
//    @Override
//    public void init_loop() {
//        if (opmodeTimer.getElapsedTimeSeconds() < .05) {
//            upper.goUp();
//        } else if (opmodeTimer.getElapsedTimeSeconds() < .1) {
//            upper.scoreDown();
//        } else {
//            upper.off();
//        }
//
//    }
//
//    /** This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {
//    }
//}