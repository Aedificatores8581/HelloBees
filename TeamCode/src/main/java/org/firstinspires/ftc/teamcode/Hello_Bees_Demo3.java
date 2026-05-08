package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.susbsystems.robot_system;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name = "!Hello Bees")
public class Hello_Bees_Demo3 extends OpMode {
    robot_system robot;
    ButtonBlock stopArm, startFogCycle, stopFogCycle, startTreatment;
    ButtonBlock stopShoulder,shoulderHome;
    ButtonBlock stopHomeShoulder, homeArm, startStopFogCycle;
    ButtonBlock dpadUp, dpadDown, dpadLeft, dpadRight;
    int yPosTarget = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new robot_system(hardwareMap);
        stopArm = new ButtonBlock()
                .onTrue(() -> {robot.stopArmToPosition();});

        startFogCycle = new ButtonBlock()
                .onTrue(() -> {robot.startFogCycle();});
        stopFogCycle = new ButtonBlock()
                .onTrue(() -> {robot.stopFogCycle();});
        stopShoulder = new ButtonBlock()
                .onTrue(() -> {robot.shoulderStop();});
        shoulderHome = new ButtonBlock()
                .onTrue(() -> {robot.shoulderHome();});
        stopHomeShoulder = new ButtonBlock()
                .onTrue(()-> {
                    if (robot.shoulderIsBusy()) {robot.shoulderStop();}
                    else {robot.shoulderHome();}});
        startStopFogCycle = new ButtonBlock()
                .onTrue(()-> {
                    if (robot.isCycling()) {robot.stopFogCycle();}
                    else {robot.startFogCycle();}});
        homeArm = new ButtonBlock()
                .onTrue(()-> {
                    robot.armHome();});
        startTreatment = new ButtonBlock()
                .onTrue(()->{robot.startTreatment();});
        robot.wristGoPos(.9);
        telemetry.addLine("Initialized");
        telemetry.update();

    }
    @Override
    public void loop() {
        buttonEvents();
        robot.robot_drive(gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (!robot.shoulderIsBusy()) robot.shoulderSetPower(-gamepad2.right_stick_y);
        else if (Math.abs(gamepad2.right_stick_y) > 0) {
            robot.shoulderStop();
            robot.shoulderSetPower(-gamepad2.right_stick_y);
        }
        if(gamepad2.dpad_up){robot.moveWristManual(.5);}
        if(gamepad2.dpad_down){robot.moveWristManual(-.5);}
        robot.moveExtensionManual(gamepad2.left_stick_y);
        robot.moveTurretManual(gamepad2.left_trigger - gamepad2.right_trigger);
        robot.update();

        telemetry();
    }
    private void buttonEvents() {
        // Simplified down all into classes that handle button blocks the same way that a normal one does

        startStopFogCycle.update(gamepad1.b);
        stopArm.update(gamepad1.a);
        shoulderHome.update(gamepad2.b);
        homeArm.update(gamepad1.x);
        startTreatment.update(gamepad1.y);
    }

    @Override
    public void stop() {
    }
    private void telemetry() {
        telemetry.addLine("  Controls Guide:");
        telemetry.addLine("(A:Stop Arm To Pos) (B:Start/Stop Fog)");
        telemetry.addLine("(X:Home Arm) (Y:Start Treatment)");
        telemetry.addData("[Ready] Treat: ",robot.isReadyToTreat()+" Arm: "+robot.isArmHomed()+" Imp: "+robot.isArmLocationLogicImprovement());
         telemetry.addData("Fog Cycle: ", robot.isCycling()+"Cycle Count: "+robot.getCyclecount());
        telemetry.addLine("Telemetry: (Arm)");
        telemetry.addData("Position:"," (X) %.1f (Y) %.1f (Z) %.1f", robot.getCurrent_Arm_Position().x,robot.getCurrent_Arm_Position().y,robot.getCurrent_Arm_Position().z);
        telemetry.addData("Target:"," (X) %.1f (Y) %.1f (Z) %.1f (d) %.1f (e) %.1f", robot.getTarget_position().x,robot.getTarget_position().y,robot.getTarget_position().z,robot.getTarget_degrees(),robot.getExtensionLocal_target());
        telemetry.addData("Auto: (Yes/No)", robot.isArm_automation()+" (State) "+robot.armAutoState()+" (Ready) "+robot.isArm_ready());
        telemetry.addData("Homed: (Yes/No)", robot.isArmHomed()+" (Busy) "+robot.isArmBusy()+" BadPos: "+robot.isArm_last_position_bad());
        telemetry.addData("Wrist: H:","%.1f L:%.1f C:%.2f",robot.getWristHeight(),robot.getWristLength(),robot.getWristCalc());
        telemetry.addData("Shoulder: H:","%.1f, L:%.1f Raw:%.2f",robot.shoulderHeight(),robot.shoulderLength(),robot.shoulderRaw());
        telemetry.addLine(String.format("[TAG] Detected %b x: %.2f y: %.2f z: %.2f", robot.isTagDetected(),robot.getTagLocation().x, robot.getTagLocation().y, robot.getTagLocation().z) );
        telemetry.addLine(String.format("[Homed] Shoulder %b Turret: %b Extension: %b", robot.shoulderIsHomed(),robot.turretIsHomed(),robot.isExtensionHome()) );
        telemetry.addLine("  Controls Guide: Gamepad2");
        telemetry.addLine("Left Stick Y: Extension  Right Stick Y: Shoulder");
        telemetry.addLine("Dpad Up/Down: Wrist  Triggers: Turret");
        telemetry.addLine("(A:Stop Arm To Pos) (B:Home Shoulder)");


        telemetry.update();
    }
}
