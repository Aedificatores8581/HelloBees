package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.susbsystems.robot_system;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name = "Robot Test", group = "SubsysTest")
public class RobotTest extends OpMode {
    robot_system robot;
    ButtonBlock stopArm,startArm, startFogCycle, stopFogCycle;
    ButtonBlock stopShoulder,shoulderHome;
    ButtonBlock stopHomeShoulder,startStopArm, homeArm, startStopFogCycle;
    ButtonBlock stopStartFullCycle;
    Position armTarget, tagLocation;
    ButtonBlock dpadUp, dpadDown, dpadLeft, dpadRight;
    int yPosTarget = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new robot_system(hardwareMap);
        armTarget = new Position(DistanceUnit.INCH,-30,-7,-6,System.nanoTime());
        tagLocation = new Position(DistanceUnit.INCH,0,0,0,0);
        stopArm = new ButtonBlock()
                .onTrue(() -> {robot.stopArmToPosition();});
        startArm = new ButtonBlock()
                .onTrue(() -> {robot.startarmToPosition(armTarget);});
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
        startStopArm = new ButtonBlock()
                .onTrue(()-> {
                    if (robot.isArm_automation()) {robot.stopArmToPosition();}
                    else {robot.startarmToPosition(armTarget);}});
        startStopFogCycle = new ButtonBlock()
                .onTrue(()-> {
                    if (robot.isCycling()) {robot.stopFogCycle();}
                    else {robot.startFogCycle();}});
        homeArm = new ButtonBlock()
                .onTrue(()-> {
                    if (robot.isArm_automation()) {robot.stopArmToPosition();}
                    else {robot.armHome();}});
        stopStartFullCycle = new ButtonBlock()
                .onTrue(()-> {
                    if (robot.isFullCycleAutomation()) {robot.stopFullCycle();}
                    else {robot.startFullCycle(armTarget);}});
        dpadUp = new ButtonBlock().onTrue(() -> {yPos(yPosTarget++);});
        dpadDown = new ButtonBlock().onTrue(() -> {yPos(yPosTarget--);});
        dpadLeft = new ButtonBlock().onTrue(() -> {armTarget.z++;});
        dpadRight = new ButtonBlock().onTrue(() -> {armTarget.z--;});
        telemetry.addLine("Initialized");
        telemetry.update();

    }
    @Override
    public void loop() {
        buttonEvents();
        robot.robot_drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        if (!robot.shoulderIsBusy()) robot.shoulderSetPower(-gamepad1.right_stick_y);
        else if (Math.abs(gamepad1.right_stick_y) > 0) {
            robot.shoulderStop();
            robot.shoulderSetPower(-gamepad1.right_stick_y);
        }
        robot.update();
        tagLocation = robot.getTagLocation();
        telemetry();
    }
    private void buttonEvents() {
        // Simplified down all into classes that handle button blocks the same way that a normal one does
        //stopArm.update(gamepad1.a);
        //startArm.update(gamepad1.b);
        //stopShoulder.update(gamepad1.a);
        //shoulderHome.update(gamepad1.b);
        //stopStartFullCycle.update(gamepad1.a);
        startStopArm.update(gamepad1.a);
        stopHomeShoulder.update(gamepad1.b);
        homeArm.update(gamepad1.x);
        startStopFogCycle.update(gamepad1.y);
        //startFogCycle.update(gamepad1.x);
        //stopFogCycle.update(gamepad1.y);
        dpadUp.update(gamepad1.dpad_up);
        dpadDown.update(gamepad1.dpad_down);
        dpadLeft.update(gamepad1.dpad_left);
        dpadRight.update(gamepad1.dpad_right);
    }

    @Override
    public void stop() {
    }
    private void telemetry() {
        telemetry.addLine("  Controls Guide:");
        //telemetry.addLine("Arm to Pos: (A:Stop) (B:Start)");
        //telemetry.addLine("Shoulder: (A:Stop) (B:Home)");
        //telemetry.addLine("(A:Stop/Start Full) (B:Home/Stop Shoulder)");
        telemetry.addLine("(A:Stop/Start Arm To Pos) (B:Home/Stop Shoulder)");
        telemetry.addLine("(X:Home Arm) (Y:Start/Stop Cycle)");
        //telemetry.addLine("Fog Cycle: (X:Start) (Y:Stop)");
        //telemetry.addLine("Toggle Direction ()");
        telemetry.addLine("+/- Y Position: (DPadUp:+) (DPadDown:-)");
        telemetry.addLine("+/- Z Position: (DPadLeft:+) (DPadRight:-)");
        /*telemetry.addLine();
        telemetry.addLine("  Telemetry Info:");
        telemetry.addData("Pump: ","(On/Off)"+pump.GetState()+" Fan: (On/Off)"+fan.GetState());
        telemetry.addData("Target Position: ", pump.GetTarget()+" Busy: "+pump.isBusy());*/
        telemetry.addData("Fog Cycle: ", robot.isCycling()+"Cycle Count: "+robot.getCyclecount());
        telemetry.addLine("Telemetry: (Arm)");
        telemetry.addData("Position:"," (X) %.2f (Y) %.2f (Z) %.2f", robot.getCurrent_Arm_Position().x,robot.getCurrent_Arm_Position().y,robot.getCurrent_Arm_Position().z);
        telemetry.addData("Target:"," (X) %.2f (Y) %.2f (Z) %.2f", armTarget.x,armTarget.y,armTarget.z);
        telemetry.addData("Auto: (Yes/No)", robot.isArm_automation()+" (State) "+robot.armAutoState()+" (Ready) "+robot.isArm_ready());
        telemetry.addData("Homed: (Yes/No)", robot.isArmHomed()+" (Busy) "+robot.isArmBusy()+" BadPos: "+robot.isArm_last_position_bad());
        //telemetry.addData("Extension:"," (PosTarget) %.2f (isBusy) %b",robot.getExtensionTarget(),robot.isBusyExtension());
        telemetry.addData("Wrist: H:","%.2f L:%.2f",robot.getWristHeight(),robot.getWristLength());
        //telemetry.addData("Turret:", "(isBusy) %b (Pos) %.3f (Target) %.3f",robot.turretIsBusy(),robot.turretPosition(),robot.turretTargetPosition());
        telemetry.addLine(String.format("[TAG] Detected %b x: %.2f y: %.2f z: %.2f", robot.isTagDetected(),tagLocation.x, tagLocation.y, tagLocation.z) );
        telemetry.update();
    }
    private void yPos(int new_y_Position){
        if(yPosTarget < 0){
            yPosTarget = 0;
            armTarget.y = 7;
        }
        else if(yPosTarget == 0){
            armTarget.y = 7;
        }
        else if(yPosTarget == 1){
            armTarget.y = 0;
        }
        else if(yPosTarget == 2){
            armTarget.y = 10;
        }
        else if(yPosTarget == 3){
            armTarget.y = -7;
        }
        else {
            yPosTarget = 3;
            armTarget.y = -7;
        }
    }
}
