package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

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

    private Servo claw;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */
    //public ClawSubsystem claw;

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
    private PathChain path1, path2, path3;
    //private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(10.000, 72.000, Point.CARTESIAN),
                                new Point(38.131, 72.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        path2 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(38.131, 72.000, Point.CARTESIAN),
                                new Point(4.508, 15.381, Point.CARTESIAN),
                                new Point(89.635, 49.591, Point.CARTESIAN),
                                new Point(57.017, 25.193, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(57.017, 25.193, Point.CARTESIAN),
                                new Point(34.475, 27.580, Point.CARTESIAN),
                                new Point(20.420, 20.155, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(20.420, 20.155, Point.CARTESIAN),
                                new Point(32.354, 41.105, Point.CARTESIAN),
                                new Point(11.138, 28.641, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(208), Math.toRadians(180))
                .build();
        path3 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(11.138, 28.641, Point.CARTESIAN),
                                new Point(39.249, 34.210, Point.CARTESIAN),
                                new Point(9.017, 73.724, Point.CARTESIAN),
                                new Point(38.131, 73.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
    }



    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
                setPathState(1);
                break;
            case 1:
                if(follower.getPose().getX() > (38.131 - 1)) {
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(follower.getPose().getX() < (11.138 + 1) && follower.getPose().getY() < (28.641 + 1)) {
                    follower.followPath(path3, true);
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