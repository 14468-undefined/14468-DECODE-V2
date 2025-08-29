package org.firstinspires.ftc.teamcode.util;

import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.;
import org.firstinspires.ftc.teamcode.util.TagDetectionPipeline;

import java.util.ArrayList;

public class TagDetectionPipeline extends OpenCvPipeline {
    private AprilTagDetectionPipeline aprilTag;
    private AprilTagDetection latestDetection;

    public TagDetectionPipeline() {
        // camera params (fx, fy, cx, cy) + tag size in meters
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
        double tagsize = 0.166;

        aprilTag = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
    }

    @Override
    public org.opencv.core.Mat processFrame(org.opencv.core.Mat input) {
        ArrayList<AprilTagDetection> detections = aprilTag.processFrame(input);

        if (detections.size() > 0) {
            latestDetection = detections.get(0);
        } else {
            latestDetection = null;
        }

        return input;
    }

    public AprilTagDetection getLatestDetection() {
        return latestDetection;
    }

    // Error calculation for driving
    public double getErrorX() {
        if (latestDetection == null) return 0;
        return latestDetection.center.x - 400; // image center (tune for your res)
    }

    public double getErrorY() {
        if (latestDetection == null) return 0;
        return 300 - latestDetection.center.y; // higher = closer/farther
    }
}
