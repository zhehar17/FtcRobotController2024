package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Timer;

@Disabled
@Config
@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {

    public class Upper {
        private DcMotor upper;

        public Upper(HardwareMap hardwareMap) {
            upper = hardwareMap.get(DcMotor.class, "upper");
            upper.setDirection(DcMotor.Direction.REVERSE);
            upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class LiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    upper.setPower(1);
                    initialized = true;
                }

                // checks lift's current position
                double pos = upper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 545) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    upper.setPower(.45);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDownToScore implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    upper.setPower(-0.55);
                    initialized = true;
                }

                // checks lift's current position
                double pos = upper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 475) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    upper.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action liftDownToScore() {
            return new LiftDownToScore();
        }

        public class LiftDownToPickup implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    upper.setPower(-0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = upper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    upper.setPower(0);
                    upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action liftDownToPickup() {
            return new LiftDownToPickup();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.6);
                sleep(500);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(.25);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() {
// instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Claw claw = new Claw(hardwareMap);
        // make a Lift instance
        Upper lift = new Upper(hardwareMap);

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(30)
                .waitSeconds(.2);
        TrajectoryActionBuilder tab2 = tab1.fresh()
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(30,-37, Math.PI),3*Math.PI/2)
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(52, -46, Math.PI),3*Math.PI/2)
                .waitSeconds(.1);
        TrajectoryActionBuilder tab3 = tab2.fresh()
                .setTangent(Math.PI)
                .lineToX(15)
                .setTangent(Math.PI/2)
                .strafeTo(new Vector2d(17,-44))
                .splineToLinearHeading(new Pose2d(.5, -45, Math.PI), Math.PI)
                .waitSeconds(.2);
        TrajectoryActionBuilder tab4 = tab3.fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(25.75, 3.5, 0), Math.PI/2)
                .splineToLinearHeading(new Pose2d(30,3.5,0),Math.PI/2)
                .waitSeconds(.2);
        TrajectoryActionBuilder tab5 = tab4.fresh()
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(10, -44.5, Math.PI), Math.PI * 3/2)
                .splineToLinearHeading(new Pose2d(.5, -44.5, Math.PI), Math.PI * 3/2)
                .waitSeconds(.2);
        TrajectoryActionBuilder tab6 = tab5.fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(25.75, 5, 0), Math.PI/2)
                .splineToLinearHeading(new Pose2d(31.2,4.5,0),Math.PI/2)
                .waitSeconds(.2);
        TrajectoryActionBuilder tab7 = tab6.fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(30.25, 2, 0), Math.PI/2)
                .waitSeconds(.2);
        TrajectoryActionBuilder tab8 = tab7.fresh()
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(30,-50, Math.PI),3*Math.PI/2)
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(50, -50, Math.PI),3*Math.PI/2)
                .waitSeconds(.2);
        TrajectoryActionBuilder tab9 = tab8.fresh()
                .setTangent(Math.PI)
                .lineToX(5)
                .setTangent(Math.PI * 3/2)
                .strafeTo(new Vector2d(2, -50))
                .waitSeconds(.2);
        TrajectoryActionBuilder tab10 = tab9.fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(30, 3, 0), Math.PI/2)
                .waitSeconds(.2);
        TrajectoryActionBuilder tab11 = tab10.fresh()
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(10,-40, Math.PI),3*Math.PI/2);


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionOne;
        Action trajectoryActionTwo;
        Action trajectoryActionThree;
        Action trajectoryActionFour;
        Action trajectoryActionFive;
        Action trajectoryActionSix;
        Action trajectoryActionSeven;
        Action trajectoryActionEight;
        Action trajectoryActionNine;
        Action trajectoryActionTen;
        Action trajectoryActionEleven;
        trajectoryActionOne = tab1.build();
        trajectoryActionTwo = tab2.build();
        trajectoryActionThree = tab3.build();
        trajectoryActionFour = tab4.build();
        trajectoryActionFive = tab5.build();
        trajectoryActionSix = tab6.build();
        trajectoryActionSeven = tab7.build();
        trajectoryActionEight = tab8.build();
        trajectoryActionNine = tab9.build();
        trajectoryActionTen = tab10.build();
        trajectoryActionEleven = tab11.build();

        Actions.runBlocking(
                new ParallelAction(
                        lift.liftUp(),
                        trajectoryActionOne
                       /*
                        trajectoryActionSeven,
                        lift.liftDownToScore(),
                        claw.openClaw(),
                        lift.liftDownToPickup(),
                        trajectoryActionEight,
                        trajectoryActionNine,
                        claw.closeClaw(),
                        lift.liftUp(),
                        trajectoryActionTen,
                        lift.liftDownToScore(),
                        claw.openClaw(),
                        lift.liftDownToPickup(),
                        trajectoryActionEleven*/
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        lift.liftDownToScore()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        claw.openClaw(),
                        lift.liftDownToPickup(),
                        trajectoryActionTwo
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionThree,
                        claw.closeClaw()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        lift.liftUp(),
                        trajectoryActionFour
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        lift.liftDownToScore()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        claw.openClaw(),
                        lift.liftDownToPickup(),
                        trajectoryActionFive
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        lift.liftUp(),
                        trajectoryActionSix
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        lift.liftDownToScore()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        claw.openClaw(),
                        lift.liftDownToPickup(),
                        trajectoryActionEleven
                )
        );
    }
}