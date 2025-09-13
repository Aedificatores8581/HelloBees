package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Arm725;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp(name = "Shoulder Test", group = "SubsysTest")
public class shoulderTest extends LinearOpMode{
    Arm725 shoulder;
    ButtonBlock runToPos, stopArm, dpadUp, dpadDown;

    double targetPos = 0;
    @Override
    public void runOpMode() {
        shoulder = new Arm725(hardwareMap);

        runToPos = new ButtonBlock().onTrue(() -> {shoulder.GoToAngle(targetPos);});
        stopArm = new ButtonBlock().onTrue(() -> {shoulder.Stop();});
        dpadUp = new ButtonBlock().onTrue(() -> {targetPos += 2;});
        dpadDown = new ButtonBlock().onTrue(() -> {targetPos -= 2;});

        telemetry.addLine("Initialized");
        telemetry.update();
        //shoulder.StartHome();
        while (!shoulder.Homed() && !opModeIsActive() && !isStopRequested()) shoulder.Update();
        waitForStart();
        while (opModeIsActive()) {
            runToPos.update(gamepad1.a);
            stopArm.update(gamepad1.b);
            if (gamepad1.x) {shoulder.StartHome();}
            dpadUp.update(gamepad1.dpad_up);
            dpadDown.update(gamepad1.dpad_down);

            if (!shoulder.IsBusy()) shoulder.SetPower(gamepad1.right_stick_x);
            else if (Math.abs(gamepad1.right_stick_x) > 0) {
                shoulder.Stop();
                shoulder.SetPower(gamepad1.right_stick_x);
            }
            shoulder.Update();
            telemetry.addLine("  Controls Guide:");
            telemetry.addLine("A: Go to Target");
            telemetry.addLine("B: Force Stop");
            telemetry.addLine("X: Home");
            telemetry.addLine("Right Stick X: Rotation");
            telemetry.addLine();
            telemetry.addLine("  Telemetry Info:");
            telemetry.addData("Motor Power",  shoulder.GetPower());
            telemetry.addData("Homed", shoulder.Homed());
            telemetry.addData("Is Busy", shoulder.IsBusy());
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos", "Deg: "+shoulder.GetPos()+" Raw: "+shoulder.GetRawPos());
            telemetry.addData("Active Target Pos", "Deg: "+shoulder.GetTargetPos()+" Raw: "+shoulder.GetRawTargetPos());
            telemetry.update();
        }
    }
}
