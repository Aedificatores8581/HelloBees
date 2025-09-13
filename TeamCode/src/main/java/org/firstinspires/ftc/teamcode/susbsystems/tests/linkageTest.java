package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.LinkageExtension725;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name = "Extension Test", group = "SubsysTest")
public class linkageTest extends OpMode {
    LinkageExtension725 extension;
    ButtonBlock runToPos, stopExtension, dpadUp, dpadDown;
    double targetPos = 0;

    @Override
    public void init() {
        extension = new LinkageExtension725(hardwareMap);
        extension.StartHome();
        runToPos = new ButtonBlock().onTrue(() -> {extension.GoTo(targetPos);});
        stopExtension = new ButtonBlock().onTrue(() -> {extension.Stop();});
        dpadUp = new ButtonBlock().onTrue(() -> {targetPos += 1;});
        dpadDown = new ButtonBlock().onTrue(() -> {targetPos -= 1;});
    }

    @Override
    public void init_loop() {
        if (!extension.Homed())
            extension.Update();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (!extension.IsBusy()) extension.SetPower(gamepad1.right_stick_y);
        else if (Math.abs(gamepad1.right_stick_y) > 0) {
            extension.Stop();
            extension.SetPower(gamepad1.right_stick_y);
        }
        runToPos.update(gamepad1.a);
        stopExtension.update(gamepad1.b);
        dpadUp.update(gamepad1.dpad_up);
        dpadDown.update(gamepad1.dpad_down);
        extension.Update();
        telemetry.addLine("  Controls Guide:");
        telemetry.addLine("A: Go to Target");
        telemetry.addLine("B: Force Stop");
        telemetry.addLine("Right Stick Y: Extension");
        telemetry.addLine();
        telemetry.addLine("  Telemetry Info:");
        telemetry.addData("Motor Power",  extension.GetPower());
        telemetry.addData("Right Stick Y",  gamepad1.right_stick_y);
        telemetry.addData("Homed", extension.Homed());
        telemetry.addData("Is at Home?", extension.isAtHome());
        telemetry.addData("Is Busy", extension.IsBusy());
        telemetry.addData("Target Pos", targetPos);
        telemetry.addData("Current Pos", "Inches: "+extension.GetPos()+" Raw: "+extension.GetRawPos());
        telemetry.addData("Active Target Pos", "Inches: "+extension.GetTargetPos()+" Raw: "+extension.GetRawTargetPos());
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
