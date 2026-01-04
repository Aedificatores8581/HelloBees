package org.firstinspires.ftc.teamcode.susbsystems.tests;

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
    robot_system.Subsystems subsystem;
    Position armTarget;

    @Override
    public void init() {
        robot = new robot_system(hardwareMap);
        stopArm = new ButtonBlock()
                .onTrue(() -> {robot.stopArmToPosition();});
        startArm = new ButtonBlock()
                .onTrue(() -> {robot.startarmToPosition(armTarget);});
        startFogCycle = new ButtonBlock()
                .onTrue(() -> {robot.startFogCycle();});
        stopFogCycle = new ButtonBlock()
                .onTrue(() -> {robot.stopFogCycle();});
        armTarget = new Position(DistanceUnit.INCH,-20,0,-10,System.nanoTime());
        telemetry.addLine("Initialized");
        telemetry.update();

    }
    @Override
    public void loop() {
        buttonEvents();
        robot.robot_drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        robot.update();
        telemetry();
    }
    private void buttonEvents() {
        // Simplified down all into classes that handle button blocks the same way that a normal one does
        stopArm.update(gamepad1.a);
        startArm.update(gamepad1.b);
        startFogCycle.update(gamepad1.x);
        stopFogCycle.update(gamepad1.y);
    }

    @Override
    public void stop() {
    }
    private void telemetry() {
        telemetry.addLine("  Controls Guide:");
        telemetry.addLine("Arm to Pos: (A:Stop) (B:Start)");
        telemetry.addLine("Fog Cycle: (X:Start) (Y:Stop)");
        //telemetry.addLine("Toggle Direction ()");
        //telemetry.addLine("+/- RunFor: (DPadUp:+) (DPadDown:-)");
        /*telemetry.addLine();
        telemetry.addLine("  Telemetry Info:");
        telemetry.addData("Pump: ","(On/Off)"+pump.GetState()+" Fan: (On/Off)"+fan.GetState());
        telemetry.addData("Target Position: ", pump.GetTarget()+" Busy: "+pump.isBusy());*/
        telemetry.addData("Fog Cycle: ", robot.isCycling()+"Cycle Count: "+robot.getCyclecount());
        telemetry.addData("Arm Position: (X)", robot.getCurrent_Arm_Position().x+" (Y) "+robot.getCurrent_Arm_Position().y+" (Z) "+robot.getCurrent_Arm_Position().z);
        telemetry.addData("Arm Target: (X)", armTarget.x+" (Y) "+armTarget.y+" (Z) "+armTarget.z);
        telemetry.addData("Arm Auto: (Yes/No)", robot.isArm_automation()+" (State) "+robot.armAutoState());
        telemetry.update();
    }
}
