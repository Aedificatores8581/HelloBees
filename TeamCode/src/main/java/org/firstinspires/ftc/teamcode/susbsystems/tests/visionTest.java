package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp (name = "Vision Test", group = "SubsysTest")
public class visionTest extends OpMode {
    Vision vision;
    @Override
    public void init() {
        vision = new Vision(hardwareMap);
    }
    @Override
    public void loop() {
        ArrayList<AprilTagDetection> detections = vision.GetDetections();
        if (detections != null)
        for (AprilTagDetection detection : detections) {
            telemetry.addData("Tag "+detection.id+" x", detection.ftcPose.x);
        }
        telemetry.update();
    }
}
