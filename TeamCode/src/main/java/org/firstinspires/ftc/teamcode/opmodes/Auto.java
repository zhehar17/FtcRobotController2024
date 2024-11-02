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