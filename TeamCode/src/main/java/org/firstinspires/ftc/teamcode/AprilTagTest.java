package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;


@TeleOp
@Disabled
public class AprilTagTest extends LinearOpMode {



    @Override
    public void runOpMode() {
        // Innacurate Camera Position Values for testing. Y value how high up the camera is from the turret in inches
        Position cameraPose = new Position(DistanceUnit.INCH, 0,16.25,0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0);

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPose,cameraOrientation ) // For Testing Built in Camera Offset Calculations
                .build();
        /*VisionPortal visionPortal = */new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        waitForStart();
        while (opModeIsActive()) {
            // Sort Tags from Left to Right by Least X Val to Greatest :
            List<AprilTagDetection> detections = aprilTag.getDetections();
            Collections.sort(detections, new Comparator<AprilTagDetection>() {
                public int compare(AprilTagDetection c1, AprilTagDetection c2) {
                    if (c1.ftcPose.x > c2.ftcPose.x) return -1;
                    if (c1.ftcPose.x < c2.ftcPose.x) return 1;
                    return 0;
                }});

            // List Off Detected Tags In Telemetry
            telemetry.addLine("  Detected Tags:");
            for (AprilTagDetection detection : detections) {
                double dx = detection.robotPose.getPosition().x;
                double dy = detection.robotPose.getPosition().y;
                double dz = detection.robotPose.getPosition().z;

                telemetry.addData("Tag "+detection.id+" Pose", "(x:"+dx+", y:"+dy+", z:"+dz+")");
            }
            telemetry.update();
        }
    }
}
