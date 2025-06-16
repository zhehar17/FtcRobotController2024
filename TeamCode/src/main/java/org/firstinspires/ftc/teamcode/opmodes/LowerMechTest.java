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

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

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

@TeleOp(name="LowerMechanismTest", group="Test")

public class LowerMechTest extends LinearOpMode {

    public Servo lower1 = null;
    public Servo lower2 = null;
    double pos1 = .75;
    double pos2 = .36;
    double inches = 0;
    double centerx;


    @Override
    public void runOpMode() {
        // Define and Initialize Motors
        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for.  You can use a predefined color, or create you own color range
         *     .setTargetColorRange(ColorRange.BLUE)                      // use a predefined color match
         *       Available predefined colors are: RED, BLUE YELLOW GREEN
         *     .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,      // or define your own color match
         *                                           new Scalar( 32, 176,  0),
         *                                           new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *         ImageRegion.entireFrame()
         *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
         *         ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height square centered on screen
         *
         * - Define which contours are included.
         *     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)  // return all contours
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)            // exclude contours inside other contours
         *        note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
         *
         * - turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
         *     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)    Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
         *                                    The higher the number of pixels, the more blurred the image becomes.
         *                                    Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *                                    Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
         *        .setErodeSize(int pixels)   Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *                                    Erosion can grow holes inside regions, and also shrink objects.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         *        .setDilateSize(int pixels)  Dilation makes objects more visible by filling in small holes, making lines appear thicker,
         *                                    and making filled shapes appear larger. Dilation is useful for joining broken parts of an
         *                                    object, such as when removing noise from an image.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         */
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.


        lower1 = hardwareMap.get(Servo.class, "lower1"); //.35  to .75 in
        lower2 = hardwareMap.get(Servo.class, "lower2"); //.36 in to .76
        lower1.setPosition(pos1);
        lower2.setPosition(pos2);


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
             *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
             *
             * Use any of the following filters.
             *
             * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
             *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
             *   The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             */
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
             *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
             *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
             */

            telemetry.addLine(" Area Density Aspect  Center");

            // Display the size (area) and center location for each Blob.
            for(ColorBlobLocatorProcessor.Blob b : blobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                Point[] boxCorners = new Point[4];
                boxFit.points(boxCorners);
                double lengthLeft = Math.pow((boxCorners[0].y - boxCorners[1].y),2) + Math.pow((boxCorners[0].x-boxCorners[1].x),2);
                double lengthTop = Math.pow((boxCorners[0].y - boxCorners[3].y),2) + Math.pow((boxCorners[0].x-boxCorners[3].x),2);
                int i;
                int neg = 1;
                if (lengthTop > lengthLeft) {
                    i = 1; //left = short
                } else {
                    i = 3;
                    neg = -1; //top = short
                }
                centerx = (int) boxFit.center.y;
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));

                telemetry.addLine("x: "+ boxCorners[0].x + " y: "+ boxCorners[0].y); //topleft
                telemetry.addLine("x: "+ boxCorners[1].x + " y: "+ boxCorners[1].y); //botleft
                telemetry.addLine("x: "+ boxCorners[2].x + " y: "+ boxCorners[2].y); //botright
                telemetry.addLine("x: "+ boxCorners[3].x + " y: "+ boxCorners[3].y); //topright
                telemetry.addLine("" + (neg * Math.atan((boxCorners[0].y - boxCorners[i].y)/(boxCorners[0].x-boxCorners[i].x)))); //shorter side
            }


            sleep(50);

            if (gamepad1.right_bumper) {
                pos1 = .75;
                pos2 = .36;
            }
            if (gamepad1.left_bumper) {
                pos1 = .35;
                pos2 = .76;
            }
            if (gamepad1.x) {
                pos1 -= .01;
                pos2 += .01;
                inches = 12;
            }
            if (gamepad1.a) {
                pos1 += .01;
                pos2 -= .01;
                inches = 0;
            }
            if (gamepad1.y) {
                pos1 -= .1;
                pos2 += .1;
                inches += 1;
            }
            if (gamepad1.b) {
                pos1 += .1;
                pos2 -= .1;
                inches -= 1;
            }
            if (gamepad1.start) {

                inches = 0.000961912 * Math.pow(centerx, 2) -0.305174 * centerx +21.46748;
            }

            inches = Math.max(Math.min(inches, 12), 0);
            pos2 = .132965 * Math.asin(.144949 * inches - 1) + .568861;
            pos1 = 0.75 - (pos2 - 0.36);
            lower1.setPosition(Math.max(Math.min(pos1, .75), .35));
            lower2.setPosition(Math.max(Math.min(pos2, .76), .36));

            telemetry.addLine("pos1 " + pos1);
            telemetry.addLine("pos2 " + pos2);
            telemetry.addLine("inches " + inches);
            telemetry.addLine("A: "+ (0.000961912 * Math.pow(centerx, 2) -0.305174 * centerx +21.46748));
            telemetry.update();
            sleep(100);
        }
    }
}