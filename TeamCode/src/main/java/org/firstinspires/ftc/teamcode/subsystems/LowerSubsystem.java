package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

public class LowerSubsystem {
    private Servo grabber;
    private Servo wrist;
    private Servo pivot;
    private Servo lower1;
    private Servo lower2;

    private double angle;
    double wristPos;
    double centerx;

    private ColorBlobLocatorProcessor colorLocator = null;

    private VisionPortal portal = null;

    public LowerSubsystem(HardwareMap hardwareMap) {

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "cam"))
                .build();

        //telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        //telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        lower1 = hardwareMap.get(Servo.class, "lower1"); //.35  to .75 in
        lower2 = hardwareMap.get(Servo.class, "lower2"); //.36 in to .76


        // Send telemetry message to signify robot waiting;
        //telemetry.addData(">", "Robot Ready.  Press START.");    //
        //telemetry.update();


        grabber = hardwareMap.get(Servo.class, "grabber");
        wrist = hardwareMap.get(Servo.class, "wrist");
        pivot = hardwareMap.get(Servo.class, "pivot");

        wristPos = 1;
    }

    public void setPivot(double pos){
        pivot.setPosition(pos);
    }

    public void setInches(double inches){
        inches = Math.max(Math.min(inches, 13.7), 0);
        double pos2 = .132965 * Math.asin(.144949 * inches - 1) + .568861;
        double pos1 = 0.75 - (pos2 - 0.36);
        lower1.setPosition(Math.max(Math.min(pos1, .75), .35));
        lower2.setPosition(Math.max(Math.min(pos2, .76), .36));

    }
    public void retractFull(){
        setInches(0);
    }
    public void extendFull(){
        setInches(12);
    }

    //Wrist
    public void wristUp(){
        wrist.setPosition(1);
    }
    public void lower(){
        wrist.setPosition(0);
    }
    public void raise(){
        wrist.setPosition(0.5);
    }

    public void wristPos(double pos){
        wrist.setPosition(pos);
    }

    public void grab() {
        grabber.setPosition(0.25);
    }

    public void release() {
        grabber.setPosition(0.62);
    }

    public boolean closed() {
        return grabber.getPosition() == 0.62;
    }

    public double getLastAngle() {
        return angle;
    }

    public boolean seePiece() {
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
        ColorBlobLocatorProcessor.Util.filterByArea(10000, 75000, blobs);  // filter out very small blobs.

        /*
         * The list of Blobs can be sorted using the same Blob attributes as listed above.
         * No more than one sort call should be made.  Sorting can use ascending or descending order.
         *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
         *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
         *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
         */


        // Display the size (area) and center location for each Blob.
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            return true;
        }
        return false;
    }
    public double getPivotPosition() {

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
        ColorBlobLocatorProcessor.Util.filterByArea(10000, 75000, blobs);  // filter out very small blobs.

        /*
         * The list of Blobs can be sorted using the same Blob attributes as listed above.
         * No more than one sort call should be made.  Sorting can use ascending or descending order.
         *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
         *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
         *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
         */


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
            angle = Math.abs((neg * Math.atan((boxCorners[0].y - boxCorners[i].y)/(boxCorners[0].x-boxCorners[i].x)))); //shorter side
            return ((Math.abs((neg * Math.atan((boxCorners[0].y - boxCorners[i].y)/(boxCorners[0].x-boxCorners[i].x))))) > 3*Math.PI/8 ? 0.6 : 0.26);
        }
        return .26;
    }

}
