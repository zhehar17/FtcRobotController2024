package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {

    public class Upper {
        private DcMotor upper;

        public Upper(HardwareMap hardwareMap) {
            upper = hardwareMap.get(DcMotor.class, "upper");
            upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            upper.setDirection(DcMotor.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    upper.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = upper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 750) {
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
                    upper.setPower(-0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = upper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 675) {
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
                claw.setPosition(1);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
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
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = tab1.fresh()
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(10,-50, Math.PI),3*Math.PI/2)
                .setTangent(Math.PI)
                .lineToX(2);
        TrajectoryActionBuilder tab3 = tab2.fresh()
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(30,0,0),0);
        TrajectoryActionBuilder tab4 = tab3.fresh()
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(10,-50, Math.PI),3*Math.PI/2)
                .setTangent(Math.PI)
                .lineToX(50)
                .strafeTo(new Vector2d(50, -55))
                .lineToX(10)
                .lineToX(50)
                .strafeTo(new Vector2d(50, -60))
                .lineToX(10)
                .lineToX(50)
                .strafeTo(new Vector2d(50, -65))
                .lineToX(10);

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionOne;
        Action trajectoryActionTwo;
        Action trajectoryActionThree;
        Action trajectoryActionFour;
        trajectoryActionOne = tab1.build();
        trajectoryActionTwo = tab2.build();
        trajectoryActionThree = tab3.build();
        trajectoryActionFour = tab4.build();

        Actions.runBlocking(
                new SequentialAction(
                        //lift.liftUp(),
                        trajectoryActionOne,
                        //lift.liftDownToScore(),
                        //claw.openClaw(),
                        //lift.liftDownToPickup(),
                        trajectoryActionTwo,
                        //claw.closeClaw(),
                        trajectoryActionThree,
                        //lift.liftDownToScore(),
                        //claw.openClaw(),
                        //lift.liftDownToPickup(),
                        trajectoryActionFour
                )
        );
    }
}