package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.teamcode.util.Util.avATPoseFtc;
import static org.firstinspires.ftc.teamcode.util.Util.avATPoseRaw;
import static org.firstinspires.ftc.teamcode.util.Util.avPose3D;
import static org.firstinspires.ftc.teamcode.util.Util.avd;
import static org.firstinspires.ftc.teamcode.util.Util.avPoint;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Vision {
    public static boolean SORT_BY_X = false;

    CameraName webcam;
    public VisionPortal portal;
    public AprilTagProcessor processor;
    ArrayList<AprilTagDetection> detections;
    final int width = 640, height = 480;
    private boolean gettingAvg = false;
    private int avgPasses;
    private int targetId = -1;
    private AprilTagDetection avgDetection;
    public Vision(HardwareMap hm) {
        webcam = hm.get(WebcamName.class, "Webcam 1");
        processor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(821.993f, 821.993f, 330.489f, 248.997f) // For Logitech C310 Camera, Should be Default
                //.setLensIntrinsics(907.659, 907.659, 659.985, 357.874) // For Global Shutter Camera
                .build();
        // For Intrinsics refer to TeamCode/src/main/res/xml/teamwebcamcalibrations.xml
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(width,height))
                .addProcessor(processor)
                .build();
    }
    public void Update() {
        detections = processor.getDetections();
        if (SORT_BY_X) {
            Collections.sort(detections, new Comparator<AprilTagDetection>() {
                public int compare(AprilTagDetection c1, AprilTagDetection c2) {
                    if (c1.ftcPose.x > c2.ftcPose.x) return -1;
                    if (c1.ftcPose.x < c2.ftcPose.x) return 1;
                    return 0;
                }
            });
        }
        for (AprilTagDetection detection : detections) {

            if (gettingAvg && detection.id == targetId) {
                if (avgDetection != null)
                    avgDetection = averageDetection(avgDetection, detection);
                else
                    avgDetection = detection;
            }
        }
    }
    public void StartAvg(int id) {
        targetId = id;
        gettingAvg = true;
    }
    public void StopAvg() {targetId = -1; gettingAvg = false;}
    public AprilTagDetection getAverageDetection() {
        return avgDetection;
    }
    public ArrayList<AprilTagDetection> GetDetections() { return detections; }

    private AprilTagDetection averageDetection(AprilTagDetection detection1, AprilTagDetection detection2) {
        Point[] avgCorners = new Point[detection1.corners.length];
        for (int i = 0;i<detection1.corners.length;i++) {
            avgCorners[i] = avPoint(detection1.corners[i], detection2.corners[i]);
        }

        AprilTagDetection output = new AprilTagDetection(
                detection1.id, detection1.hamming, detection1.decisionMargin,
                avPoint(detection1.center, detection2.center),
                avgCorners,
                detection1.metadata,
                avATPoseFtc(detection1.ftcPose, detection2.ftcPose),
                avATPoseRaw(detection1.rawPose,detection2.rawPose),
                avPose3D(detection1.robotPose,detection2.robotPose),
                detection1.frameAcquisitionNanoTime
        );
        return output;
    }

}
