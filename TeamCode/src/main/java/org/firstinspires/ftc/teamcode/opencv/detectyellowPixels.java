package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class detectyellowPixels extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Mat yellowMask = preprocessFrame(input);

        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest yellow contour (blob)
        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            //Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);

            for (MatOfPoint contour : contours) {
                Imgproc.drawContours(input, contours, contours.indexOf(contour), new Scalar(255, 0, 0), 2);
            }
        }

        return input;
    }


    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2YCrCb);

        Scalar lowerYellow = new Scalar(0, 138, 0);
        Scalar upperYellow = new Scalar(255, 255, 104);


        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }

}
