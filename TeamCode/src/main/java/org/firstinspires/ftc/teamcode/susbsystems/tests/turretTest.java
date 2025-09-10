package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Turret725;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name = "Turret Test", group = "SubsysTest")
public class turretTest extends LinearOpMode {
    Turret725 Turret725;
    ButtonBlock runToPos, stopTurret725, dpadUp, dpadDown;

    double targetPos = 0;
    @Override
    public void runOpMode() {
        Turret725 = new Turret725(hardwareMap);

        runToPos = new ButtonBlock().onTrue(() -> {Turret725.GoTo(targetPos);});
        stopTurret725 = new ButtonBlock().onTrue(() -> {Turret725.Stop();});
        dpadUp = new ButtonBlock().onTrue(() -> {targetPos += 5;});
        dpadDown = new ButtonBlock().onTrue(() -> {targetPos -= 5;});

        telemetry.addLine("Initialized");
        telemetry.update();
        Turret725.StartHome();
        while (!Turret725.Homed() && !opModeIsActive() && !isStopRequested()) Turret725.Update();
        waitForStart();
        while (opModeIsActive()) {
            runToPos.update(gamepad1.a);
            stopTurret725.update(gamepad1.b);
            dpadUp.update(gamepad1.dpad_up);
            dpadDown.update(gamepad1.dpad_down);

            if (!Turret725.IsBusy()) Turret725.SetPower(gamepad1.right_stick_x);
            else if (Math.abs(gamepad1.right_stick_x) > 0) {
                Turret725.Stop();
                Turret725.SetPower(gamepad1.right_stick_x);
            }
            Turret725.Update();
            telemetry.addLine("  Controls Guide:");
            telemetry.addLine("A: Go to Target");
            telemetry.addLine("B: Force Stop");
            telemetry.addLine("Right Stick X: Rotation");
            telemetry.addLine();
            telemetry.addLine("  Telemetry Info:");
            telemetry.addData("Motor Power",  Turret725.GetPower());
            telemetry.addData("In Error", Turret725.InError());
            telemetry.addData("Homed", Turret725.Homed());
            telemetry.addData("Is Busy", Turret725.IsBusy());
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos", "Deg: "+Turret725.GetPos()+" Raw: "+Turret725.GetRawPos());
            telemetry.addData("Active Target Pos", "Deg: "+Turret725.GetTargetPos()+" Raw: "+Turret725.GetRawTargetPos());
            telemetry.update();
        }
    }
}
