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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
@Disabled
@TeleOp(name="MechanismTest", group="Test")

public class MechanismTest extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor lower = null;
    public DcMotor upper = null;
    public Servo claw = null;

    boolean extending = false;

    @Override
    public void runOpMode() {
        // Define and Initialize Motors
        lower = hardwareMap.get(DcMotor.class, "lower");
        upper = hardwareMap.get(DcMotor.class, "upper");

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(1);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double extend;

            if (gamepad1.right_trigger != 0) {
                lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extend = gamepad1.right_trigger;
                extending = false;
            } else if (gamepad1.left_trigger != 0) {
                lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extend = -gamepad1.left_trigger;
                extending = false;
            } else {
                extend = 0;
            }

            if (gamepad1.a) {
                upper.setTargetPosition(0);
                upper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                upper.setPower(1);
            } else if (gamepad1.b) {
                upper.setTargetPosition(1000);
                upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                upper.setPower(1);
            }
            if (!extending) {
                lower.setPower(extend);
            }


            
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

            if (gamepad1.left_bumper) claw.setPosition(0);
            if(gamepad1.right_bumper) claw.setPosition(1);
        }
    }
}