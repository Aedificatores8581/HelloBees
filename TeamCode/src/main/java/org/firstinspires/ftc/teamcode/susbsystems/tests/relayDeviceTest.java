package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.RelayDevice;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name = "RelayDevice Test (Fan and Fogger)", group = "SubsysTest")
public class relayDeviceTest extends OpMode {
    RelayDevice fan, fogger;
    ButtonBlock fanToggle, foggerToggle, fanRunFor, foggerRunFor, dpadUp, dpadDown, devicesLock;

    double timeRunFor = 1d;

    @Override
    public void init() {
        fan = new RelayDevice(hardwareMap, "valve1");
        fogger = new RelayDevice(hardwareMap, "compressor1");

        telemetry.addLine("Initialized");
        telemetry.update();
        fan.TurnOff();
        fogger.TurnOff();

        fanToggle = new ButtonBlock()
                .onTrue(() -> {fan.ToggleState();});
        foggerToggle = new ButtonBlock()
                .onTrue(() -> {fogger.ToggleState();});
        fanRunFor = new ButtonBlock()
                .onTrue(() -> {fan.RunForSeconds(timeRunFor);});
        foggerRunFor = new ButtonBlock()
                .onTrue(() -> {fogger.RunForSeconds(timeRunFor);});
        dpadUp = new ButtonBlock()
                .onTrue(() -> {timeRunFor++;});
        dpadDown = new ButtonBlock()
                .onTrue(() -> {timeRunFor--;});
        devicesLock = new ButtonBlock()
                .onTrue(() -> {fan.ToggleInput();fogger.ToggleInput();});
    }
    @Override
    public void loop() {
        buttonEvents();
        fan.Update();
        fogger.Update();
        telemetry();
    }
    private void buttonEvents() {
        // Simplified down all into classes that handle button blocks the same way that a normal one does
        fanToggle.update(gamepad1.a);
        foggerToggle.update(gamepad1.b);
        fanRunFor.update(gamepad1.x);
        foggerRunFor.update(gamepad1.y);
        dpadUp.update(gamepad1.dpad_up);
        dpadDown.update(gamepad1.dpad_down);
        devicesLock.update(gamepad1.right_bumper);
    }
    private void telemetry() {
        telemetry.addLine("  Controls Guide:");
        telemetry.addLine("Toggles: (A:Fan) (B:Fogger)");
        telemetry.addLine("Run For Time: (X:Fan) (Y:Fogger)");
        telemetry.addLine("+/- RunForTime: (DPadUp:+) (DPadDown:-)");
        telemetry.addLine();
        telemetry.addLine("  Telemetry Info:");
        telemetry.addData("Fan State", fan.GetState());
        telemetry.addData("Fogger State", fogger.GetState());
        telemetry.addData("Time Running For", timeRunFor);
        telemetry.addData("Input State (On/Off)", "Fan: "+fan.InputState()+" Fogger:"+fogger.InputState());
        telemetry.update();
    }
    @Override
    public void stop() {
        fan.FullShutOff();
        fogger.FullShutOff();
    }
}
