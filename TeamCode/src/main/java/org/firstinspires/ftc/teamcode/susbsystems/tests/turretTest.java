package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Turret;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name = "Turret Test", group = "SubsysTest")
public class turretTest extends LinearOpMode {
    Turret turret;
    ButtonBlock runToPos, stopTurret, dpadUp, dpadDown;

    double targetPos = 0;
    @Override
    public void runOpMode() {
        turret = new Turret(hardwareMap);

        runToPos = new ButtonBlock().onTrue(() -> {turret.GoTo(targetPos);});
        stopTurret = new ButtonBlock().onTrue(() -> {turret.Stop();});
        dpadUp = new ButtonBlock().onTrue(() -> {targetPos += 5;});
        dpadDown = new ButtonBlock().onTrue(() -> {targetPos -= 5;});

        telemetry.addLine("Initialized");
        telemetry.update();
        turret.StartHome();
        while (!turret.Homed() && !opModeIsActive() && !isStopRequested()) turret.Update();
        waitForStart();
        while (opModeIsActive()) {
            runToPos.update(gamepad1.a);
            stopTurret.update(gamepad1.b);
            dpadUp.update(gamepad1.dpad_up);
            dpadDown.update(gamepad1.dpad_down);

            if (!turret.IsBusy()) turret.SetPower(gamepad1.right_stick_x);
            else if (Math.abs(gamepad1.right_stick_x) > 0) turret.SetPower(gamepad1.right_stick_x);
            turret.Update();
            telemetry.addLine("  Controls Guide:");
            telemetry.addLine("A: Go to Target");
            telemetry.addLine("B: Force Stop");
            telemetry.addLine("Right Stick X: Rotation");
            telemetry.addLine();
            telemetry.addLine("  Telemetry Info:");
            telemetry.addData("Motor Power",  turret.GetPower());
            telemetry.addData("In Error", turret.InError());
            telemetry.addData("Is Busy", turret.IsBusy());
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos", "Deg: "+turret.GetPos()+" Raw: "+turret.GetRawPos());
            telemetry.addData("Active Target Pos", "Deg: "+turret.GetTargetPos()+" Raw: "+turret.GetRawTargetPos());
            telemetry.update();
        }
    }
}
