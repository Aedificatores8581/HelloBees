package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.susbsystems.robot_system;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@Config
@TeleOp (name = "ArmPositionTest", group = "SubsysTest")
public class ArmPositionTest extends OpMode {
    robot_system robot;
    ButtonBlock stopArm,startArm, startFogCycle, stopFogCycle;
    ButtonBlock stopShoulder,shoulderHome;
    ButtonBlock stopHomeShoulder,startStopArm, homeArm, startStopFogCycle;
    ButtonBlock stopStartFullCycle,turret135, turret180, turret0, turret225, turret90, turret45;
    Position armTarget;
    ButtonBlock dpadUp, dpadDown;
    int yPosTarget = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new robot_system(hardwareMap);
        armTarget = new Position(DistanceUnit.INCH,-25,-7,-11.5,System.nanoTime());
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
        //dpadUp = new ButtonBlock().onTrue(() -> {yPos(yPosTarget++);});
        //dpadDown = new ButtonBlock().onTrue(() -> {yPos(yPosTarget--);});
        turret135 = new ButtonBlock()
                .onTrue(()->{robot.moveTurret(135);});
        turret0 = new ButtonBlock()
                .onTrue(()->{robot.moveTurret(0);});
        turret180 = new ButtonBlock()
                .onTrue(()->{robot.moveTurret(180);});
        turret225 = new ButtonBlock()
                .onTrue(()->{robot.moveTurret(225);});
        turret90 = new ButtonBlock()
                .onTrue(()->{robot.moveTurret(90);});
        turret45 = new ButtonBlock()
                .onTrue(()->{robot.moveTurret(45);});
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
        //stopArm.update(gamepad1.a);
        //startArm.update(gamepad1.b);
        //stopShoulder.update(gamepad1.a);
        //shoulderHome.update(gamepad1.b);
        //stopStartFullCycle.update(gamepad1.a);
        //startStopArm.update(gamepad1.a);
        stopHomeShoulder.update(gamepad1.b);
        homeArm.update(gamepad1.x);
        turret135.update(gamepad2.a);
        turret45.update(gamepad2.y);
        turret0.update(gamepad2.x);
        turret90.update(gamepad2.b);
        //startStopFogCycle.update(gamepad1.y);
        //startFogCycle.update(gamepad1.x);
        //stopFogCycle.update(gamepad1.y);
        //dpadUp.update(gamepad1.dpad_up);
        //dpadDown.update(gamepad1.dpad_down);
    }

    @Override
    public void stop() {
    }
    private void telemetry() {
        telemetry.addLine("  Controls Guide: Gamepad1");
        //telemetry.addLine("Arm to Pos: (A:Stop) (B:Start)");
        //telemetry.addLine("Shoulder: (A:Stop) (B:Home)");
        telemetry.addLine("(B:Home/Stop Shoulder)");
        telemetry.addLine("(X:Home Arm)");
        telemetry.addLine("Left Stick Y: Forward/Back  Right Stick X: Turn");
        telemetry.addLine("  Controls Guide: Gamepad2");
        telemetry.addLine("Left Stick Y: Extension  Right Stick Y: Shoulder");
        telemetry.addLine("Dpad Up/Down: Wrist  Triggers: Turret");
        telemetry.addLine("[Turret] A 135, Y 45, B 90, X 0");
        //telemetry.addLine("Fog Cycle: (X:Start) (Y:Stop)");
        //telemetry.addLine("Toggle Direction ()");
        //telemetry.addLine("+/- Y Position: (DPadUp:+) (DPadDown:-)");
        /*telemetry.addLine();
        telemetry.addLine("  Telemetry Info:");
        telemetry.addData("Pump: ","(On/Off)"+pump.GetState()+" Fan: (On/Off)"+fan.GetState());
        telemetry.addData("Target Position: ", pump.GetTarget()+" Busy: "+pump.isBusy());*/
        //telemetry.addData("Fog Cycle: ", robot.isCycling()+"Cycle Count: "+robot.getCyclecount());
        telemetry.addLine("Telemetry:");
        telemetry.addData("Arm Position:"," (X) %.2f (Y) %.2f (Z) %.2f", robot.getCurrent_Arm_Position().x,robot.getCurrent_Arm_Position().y,robot.getCurrent_Arm_Position().z);
        //telemetry.addData("Arm Target:"," (X) %.2f (Y) %.2f (Z) %.2f", armTarget.x,armTarget.y,armTarget.z);
        telemetry.addData("Arm Auto: (Yes/No)", robot.isArm_automation()+" (State) "+robot.armAutoState()+" (Ready) "+robot.isArm_ready()+" (Busy) "+robot.isArmBusy());
        //telemetry.addData("Extension:"," (PosTarget) %.2f (isBusy) %b",robot.getExtensionTarget(),robot.isBusyExtension());
        telemetry.addData("Wrist: H:","%.2f L:%.2f",robot.getWristHeight(),robot.getWristLength());
        telemetry.addData("Turret:", "(isBusy) %b (Pos) %.3f (Target) %.3f",robot.turretIsBusy(),robot.turretPosition(),robot.turretTargetPosition());
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
