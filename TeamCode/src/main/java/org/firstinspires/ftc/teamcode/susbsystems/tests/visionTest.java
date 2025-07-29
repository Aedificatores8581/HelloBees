package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.susbsystems.Vision725;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;

@TeleOp (name = "Vision Test", group = "SubsysTest")
public class visionTest extends OpMode {
    Vision725 vision;
    ButtonBlock toggleAvg, toggleUFD;
    boolean averaging = false;
    @Override
    public void init() {
        vision = new Vision725(hardwareMap);
        toggleAvg = new ButtonBlock() .onTrue(() -> {toggleAvg();});
        toggleUFD = new ButtonBlock() .onTrue(() -> {toggleUFD();});
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        toggleAvg.update(gamepad1.a);
        toggleUFD.update(gamepad1.b);
        vision.Update();
        ArrayList<AprilTagDetection> detections = vision.GetDetections();
        if (detections != null)
            for (AprilTagDetection detection : detections) {
                if (detection == null || detection.robotPose == null || detection.robotPose.getPosition() == null) continue;
                Position detPose = detection.robotPose.getPosition();
                telemetry.addLine(String.format("id: %03d x: %.2f y: %.2f z: %.2f", detection.id, detPose.x, detPose.y, detPose.z) );
            }
        telemetry.addData("Target Tag Id", vision.GetTargetID());
        telemetry.addData("Averaging Data Set Size",vision.DetectionsForAvgCount());
        telemetry.addData("Averaging",averaging);
        telemetry.addData("UsingFreshDetections",vision.IsUsingFreshDetections());
        if (vision.GetAverageDetection() != null) {
            Position avgPose = vision.GetAverageDetection().robotPose.getPosition();
            telemetry.addData("Average Tag", String.format("x: %.2f y: %.2f z: %.2f", avgPose.x, avgPose.y, avgPose.z) );
        }
        telemetry.update();
    }
    private void toggleAvg() {
        if (averaging) {
            vision.StopAvg();
        } else {
            vision.StartAvg(vision.GetDetections().get(0).id);
        }
        averaging = !averaging;
    }
    private void toggleUFD() {
        if (vision.IsUsingFreshDetections())
            vision.UsingFreshDetections(false);
        else
            vision.UsingFreshDetections(true);
    }
}
