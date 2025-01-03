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
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.PositionSubsystem;

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
    public Servo claw = null;
    public CRServo intake = null;

    public DistanceSensor lowerDistance = null;
    public PositionSubsystem pos;

    boolean extending = false;
    boolean liftingUp = false;
    boolean goingDown = false;
    boolean scoringPiece = false;
    boolean slow = false;
    boolean clawClosed = true;
    boolean lowerCapped = false;

    @Override
    public void runOpMode() {

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
        lowerDistance = hardwareMap.get(DistanceSensor.class, "lowerDistance");

        pos = new PositionSubsystem(hardwareMap);

        upper.setDirection(DcMotor.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        claw.setPosition(0.56);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        upper.setTargetPosition(0);
        upper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            if (gamepad2.y) {
                slow = !slow;
                sleep(125);
            }
            if (slow) denominator *= 4;
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            //mechanism below
            double extend;

            //button to cap lower movement
            //if (winch.getCurrentPosition() < -17000) lowerCapped = true;
            lowerCapped = false;

            if (gamepad1.a) lowerCapped = false;

            if (gamepad2.right_trigger != 0 && !lowerCapped) {
                lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extend = gamepad2.right_trigger*0.55;
                extending = false;
            } else if (gamepad2.left_trigger != 0) {
                lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extend = -gamepad2.left_trigger*0.25;
                extending = false;
            } else {
                extend = 0;
            }

            if (!extending) {
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
            }*/


            //Upper Mech
            if (gamepad2.a) {
                upper.setTargetPosition(450);
                upper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                upper.setPower(Math.abs(RobotConstants.upperDownPower));
                goingDown = true;
                scoringPiece = true;
            }
            if (scoringPiece && Math.abs(upper.getCurrentPosition() - 450) < 40) {
                scoringPiece = false;
                claw.setPosition(RobotConstants.openClawPos);
                upper.setTargetPosition(0);
                upper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                upper.setPower(Math.abs(RobotConstants.upperDownPower));
                sleep(50);
            }

            if (gamepad2.b) {
                upper.setTargetPosition(625);
                upper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                upper.setPower(RobotConstants.upperUpPower);
                liftingUp = true;
            }

            if (goingDown && upper.getCurrentPosition() < 15) {
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(0);
                goingDown = false;
            }

            if (liftingUp && upper.getCurrentPosition() > 600) {
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(RobotConstants.upperUpHoldPower);
                liftingUp = false;
            }



            //Debug Upper, try not to use
            if (gamepad1.dpad_up) {
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(RobotConstants.upperUpPower);
            }

            if (gamepad1.dpad_down) {
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(RobotConstants.upperDownPower);
            }

            if (gamepad1.start) {
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(RobotConstants.upperUpHoldPower);
            }

            if (gamepad1.back) {
                upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                upper.setPower(0);
            }


            //Claw
            if(gamepad2.x){
                if(clawClosed) claw.setPosition(RobotConstants.openClawPos);
                else claw.setPosition(RobotConstants.closedClawPos);
                sleep(125);
                clawClosed = !clawClosed;
            }
            /* Intake
            if (gamepad2.left_bumper) intake.setPower(1);
            else if (gamepad2.right_bumper) intake.setPower(-1);
            else intake.setPower(0);
            */

            telemetry.addData("upperPosition", upper.getCurrentPosition());
            telemetry.addData("lowerPosition", lowerDistance.getDistance(DistanceUnit.INCH));
            if(pos.validResult()) {
                telemetry.addData("X: ", pos.getX());
                telemetry.addData("Y: ", pos.getY());
                telemetry.addData("Heading: ", pos.getYaw());
            }
            telemetry.update();
        }
    }
}