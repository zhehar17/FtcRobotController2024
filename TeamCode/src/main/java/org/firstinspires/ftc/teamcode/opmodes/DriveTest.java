/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="DriveTest", group="Test")

public class DriveTest extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor  leftBack = null;
    public DcMotor rightBack = null;

    public DcMotor lower = null;
    public DcMotor upper = null;
    public DcMotor winch = null;
    public Servo claw = null;
    public CRServo intake = null;

    public DistanceSensor distance = null;

    boolean extending = false;
    boolean liftingUp = false;
    boolean goingDown = false;
    boolean scoringPiece = false;

    @Override
    public void runOpMode() {
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        // Define and Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftBack   = hardwareMap.get(DcMotor.class, "backLeft");
        rightBack  = hardwareMap.get(DcMotor.class, "backRight");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and Initialize Motors
        lower = hardwareMap.get(DcMotor.class, "lower");
        upper = hardwareMap.get(DcMotor.class, "upper");
        winch = hardwareMap.get(DcMotor.class, "winch");

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        claw.setPosition(1);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upper.setTargetPosition(0);
        upper.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
/*
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);*/

            //mechanism below
            double extend;
            double winchPower;

            if (gamepad1.right_trigger != 0) {
                lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extend = gamepad1.right_trigger*0.65;
                winchPower = -gamepad1.right_trigger;
                extending = false;
            } else if (gamepad1.left_trigger != 0) {
                lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extend = -gamepad1.left_trigger*0.25;
                winchPower = gamepad1.left_trigger;
                extending = false;
            } else {
                extend = 0;
                winchPower = 0;
            }

            if (gamepad1.a) {
                upper.setTargetPosition(650);
                upper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                upper.setPower(.6);
                goingDown = true;
                scoringPiece = true;
            }
            if (scoringPiece && Math.abs(upper.getCurrentPosition() - 650) < 25) {
                scoringPiece = false;
                claw.setPosition(0);
                upper.setTargetPosition(0);
                upper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                upper.setPower(.6);
                sleep(50);
                claw.setPosition(1);
            }

            if (gamepad1.b) {
                upper.setTargetPosition(775);
                upper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                upper.setPower(.7);
                liftingUp = true;
            }

            if (goingDown && upper.getCurrentPosition() < 0) {
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(0);
                goingDown = false;
            }

            if (liftingUp && upper.getCurrentPosition() > 750) {
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(0);
                liftingUp = false;
            }

            if (gamepad1.dpad_up) {
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(.6);
            }

            if (gamepad1.dpad_down) {
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(-.6);
            }

            if (gamepad1.start) {
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(0);
            }

            if (gamepad1.back) {
                upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (!extending) {
                winch.setPower(winchPower);
                lower.setPower(extend);
            }
            /*
            if (gamepad1.y) {
                lower.setTargetPosition(1000);
                lower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lower.setPower(1);
                extending = true;
            }
            if (gamepad1.x) {
                lower.setTargetPosition(0);
                lower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lower.setPower(1);
                extending = true;
            }
            */
            //winch for intake
            if (gamepad1.left_stick_y <= 0) {
                if (gamepad1.dpad_left) {
                    winch.setPower(0.8);
                } else if (gamepad1.dpad_right) {
                    winch.setPower(-0.8);
                } else {
                    winch.setPower(0);
                }
            }
            //claw
            if (gamepad1.left_bumper) claw.setPosition(0);
            if(gamepad1.right_bumper) claw.setPosition(1);

            if (gamepad1.right_stick_x > 0) intake.setPower(1);
            else if (gamepad1.right_stick_x < 0) intake.setPower(-1);
            else intake.setPower(0);

            if (-gamepad1.left_stick_y > 0) {
                if (distance.getDistance(DistanceUnit.INCH) > 1.66) winch.setPower(-.2);
                else if (distance.getDistance(DistanceUnit.INCH) < 1.56) winch.setPower(.2);
                else winch.setPower(0);
            }

            telemetry.addData("position", upper.getCurrentPosition());
            telemetry.addData("distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}