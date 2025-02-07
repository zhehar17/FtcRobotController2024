   /* Copyright (c) 2023 FIRST. All rights reserved.
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

   import com.qualcomm.hardware.limelightvision.LLResult;
   import com.qualcomm.hardware.limelightvision.LLResultTypes;
   import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
   import com.qualcomm.robotcore.eventloop.opmode.Disabled;
   import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
   import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
   import com.qualcomm.robotcore.hardware.DcMotor;
   import com.qualcomm.robotcore.hardware.DcMotorSimple;

   import com.qualcomm.hardware.limelightvision.Limelight3A;
   import java.util.List;

   /*
    * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
    * the easy way.
    *
    * For an introduction to AprilTags, see the FTC-DOCS link below:
    * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
    *
    * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
    * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
    * the current Season's AprilTags and a small set of "test Tags" in the high number range.
    *
    * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
    * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
    * https://ftc-docs.firstinspires.org/apriltag-detection-values
    *
    * To experiment with using AprilTags to navigate, try out these two driving samples:
    * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
    *
    * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
    * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
    */
   @Disabled
   @TeleOp(name = "LimelightTest", group = "Test")

   public class LimelightTest extends LinearOpMode {

       Limelight3A limelight;

       double ayaw = 0;
       double ax;
       double ay;
       double az;
       @Override
       public void runOpMode() {

           initLimelight();

           // Declare our motors
           // Make sure your ID's match your configuration
           DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
           DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
           DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
           DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");


           frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
           backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

           // Wait for the DS start button to be touched.
           telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
           telemetry.addData(">", "Touch Play to start OpMode");
           telemetry.update();
           waitForStart();

           if (opModeIsActive()) {
               while (opModeIsActive()) {

                   telemetryLimelight();

                   // Push telemetry to the Driver Station.
                   telemetry.update();

                //double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double y;
                if (ay < 10 && ay > 9) {
                    y = 0;
                } else if (ay > 10) {
                    y = .1;
                } else {
                    y = -.1;
                }
                //double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                // double rx = -gamepad1.right_stick_x;
                double x;
                if (ax < 1 && ax > -1) {
                    x = 0;
                } else if (ax > 1) {
                    x = .35;
                } else {
                    x = -.35;
                }

                double rx = 0;
                /*double rx = 0;
                if (ayaw < 2 && ayaw > -2) {
                    rx = 0;
                } else if (ayaw < -2){
                    rx = -0.1;
                } else {
                    rx = 0.1;
                }*/

                   // Denominator is the largest motor power (absolute value) or 1
                   // This ensures all the powers maintain the same ratio,
                   // but only if at least one is out of the range [-1, 1]
                   double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                   double frontLeftPower = (y + x + rx) / denominator;
                   double backLeftPower = (y - x + rx) / denominator;
                   double frontRightPower = (y - x - rx) / denominator;
                   double backRightPower = (y + x - rx) / denominator;

                   frontLeftMotor.setPower(frontLeftPower);
                   backLeftMotor.setPower(backLeftPower);
                   frontRightMotor.setPower(frontRightPower);
                   backRightMotor.setPower(backRightPower);


                   // Share the CPU.
                   sleep(5);
               }
           }
           limelight.stop();
       }   // end method runOpMode()

       /**
        * Initialize the AprilTag processor.
        */
       private void initLimelight() {

           limelight = hardwareMap.get(Limelight3A.class, "limelight");
           limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
           limelight.start(); // This tells Limelight to start looking!
           limelight.pipelineSwitch(0); // Switch to pipeline number 0
       }   // end method initLimelight()

       /**
        * Add telemetry about AprilTag detections.
        */
       private void telemetryLimelight() {

           LLResult currentDetection = limelight.getLatestResult();

           ayaw = 0;
           ax = 0;
           ay = 9.5;
           // Step through the list of detections and display info for each one.
           if (currentDetection != null && currentDetection.isValid()) {
               telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)", currentDetection.getTx(), currentDetection.getTa(), currentDetection.getTy()));
               ax = currentDetection.getTx();
               ay = currentDetection.getTa();
               az = currentDetection.getTy();
           } else {
               telemetry.addData("Limelight", "No Targets");
           }
       }   // end method telemetryLimelight()

   }   // end class