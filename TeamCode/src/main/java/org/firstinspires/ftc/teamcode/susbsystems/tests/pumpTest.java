package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Pump_Subsystem;
import org.firstinspires.ftc.teamcode.susbsystems.RelayDevice;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name = "Pump Test (Fan and Fogger)", group = "SubsysTest")
public class pumpTest extends OpMode {
    Pump_Subsystem pump;
    ButtonBlock pumpToggle, pumpRunForTime, pumpRunForEncoder, dpadUp, dpadDown;

    double timeRunFor = 1d;

    @Override
    public void init() {
        pump = new Pump_Subsystem(hardwareMap, "pump");

        telemetry.addLine("Initialized");
        telemetry.update();

        pumpToggle = new ButtonBlock()
                .onTrue(() -> {pump.ToggleState();});
        pumpRunForEncoder = new ButtonBlock()
                .onTrue(() -> {pump.RunForTicks((int)timeRunFor);});
        pumpRunForTime = new ButtonBlock()
                .onTrue(() -> {pump.RunForSeconds(timeRunFor);});
        dpadUp = new ButtonBlock()
                .onTrue(() -> {timeRunFor = timeRunFor +5;});
        dpadDown = new ButtonBlock()
                .onTrue(() -> {timeRunFor = timeRunFor - 5;});
    }
    @Override
    public void loop() {
        if (!pump.isBusy()) pump.SetPower(gamepad1.right_stick_y);
        else if (Math.abs(gamepad1.right_stick_y) > 0) {
            pump.TurnOff();
            pump.SetPower(gamepad1.right_stick_y);
        }
        buttonEvents();
        pump.update();
        telemetry();
    }
    private void buttonEvents() {
        // Simplified down all into classes that handle button blocks the same way that a normal one does
        pumpToggle.update(gamepad1.a);
        pumpRunForEncoder.update(gamepad1.b);
        pumpRunForTime.update(gamepad1.x);
        dpadUp.update(gamepad1.dpad_up);
        dpadDown.update(gamepad1.dpad_down);
    }
    private void telemetry() {
        telemetry.addLine("  Controls Guide:");
        telemetry.addLine("Toggles: (A:Pump)");
        telemetry.addLine("Run For Time: (X:Pump)");
        telemetry.addLine("Run For Encoder: (B:Pump)");
        telemetry.addLine("+/- RunFor: (DPadUp:+) (DPadDown:-)");
        telemetry.addLine();
        telemetry.addLine("  Telemetry Info:");
        telemetry.addData("Running For", timeRunFor);
        telemetry.addData("Pump: ","(On/Off)"+pump.GetState()+" Power: "+pump.GetPower()+" Pos: "+pump.GetPos());
        telemetry.addData("Target Position: ", pump.GetTarget()+" Busy: "+pump.isBusy());
        telemetry.update();
    }
    @Override
    public void stop() {
    }
}
