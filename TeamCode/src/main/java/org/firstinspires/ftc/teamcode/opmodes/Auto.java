package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
                if (pos < 100.0) {
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

        public class LiftDown implements Action {
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

        public Action liftDown() {
            return new LiftDown();
        }
    }

    public class Lower {
        private DcMotor lower;
        private DcMotor winch;

        public Lower(HardwareMap hardwareMap) {
            lower = hardwareMap.get(DcMotor.class, "lower");
            lower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lower.setDirection(DcMotor.Direction.FORWARD);
            winch = hardwareMap.get(DcMotor.class, "winch");
            winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            winch.setDirection(DcMotor.Direction.FORWARD);
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

    public class Intake {
        private Servo intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(Servo.class, "intake");
        }
    }

    @Override
    public void runOpMode() {

    }
}