package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Vision725;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;

@TeleOp (name = "Vision Test", group = "SubsysTest")
public class visionTest extends OpMode {
    Vision725 vision;
    ButtonBlock toggleAvg;
    boolean averaging = false;
    @Override
    public void init() {
        vision = new Vision725(hardwareMap);
        toggleAvg = new ButtonBlock() . onTrue(() -> {toggleAvg();});
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        toggleAvg.update(gamepad1.a);
        vision.Update();
        ArrayList<AprilTagDetection> detections = vision.GetDetections();
        if (detections != null)
            for (AprilTagDetection detection : detections) {
                AprilTagPoseFtc detPose = detection.ftcPose;
                telemetry.addLine(String.format("id: %03d x: %.2f y: %.2f z: %.2f", detection.id, detPose.x, detPose.y, detPose.z) );
            }
        telemetry.addData("PerTagAvgPoseSolveTime", vision.processor.getPerTagAvgPoseSolveTime());
        if (vision.getAverageDetection() != null) {
            AprilTagPoseFtc avgPose = vision.getAverageDetection().ftcPose;
            telemetry.addData("Average Tag", String.format("x: %.2f y: %.2f z: %.2f", avgPose.x, avgPose.y, avgPose.z) );
        }
        telemetry.update();
    }
    private void toggleAvg() {
        if (averaging) {
            vision.StopAvg();
        } else {
            vision.StartAvg(583);
        }
        averaging = !averaging;
    }
}
