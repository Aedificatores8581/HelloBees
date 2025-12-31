package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Arm725;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shoulder Test", group = "SubsysTest")
public class shoulderTest extends LinearOpMode{
    Arm725 shoulder;
    ButtonBlock runToPos, stopArm, dpadUp, dpadDown,runToHeight, runToHome;
    private Servo wrist;

    double targetPos = 0;
    @Override
    public void runOpMode() {
        shoulder = new Arm725(hardwareMap);
        wrist = hardwareMap.get(Servo.class, "wrist");

        runToPos = new ButtonBlock().onTrue(() -> {shoulder.GoToAngle(targetPos);});
        stopArm = new ButtonBlock().onTrue(() -> {shoulder.Stop();});
        dpadUp = new ButtonBlock().onTrue(() -> {targetPos += 3;});
        dpadDown = new ButtonBlock().onTrue(() -> {targetPos -= 3;});
        runToHeight = new ButtonBlock().onTrue(() -> {shoulder.GoToHeight(targetPos);});
        runToHome = new ButtonBlock().onTrue(() -> {shoulder.Home();});


        telemetry.addLine("Initialized");
        telemetry.update();

        while (!shoulder.Homed() && !opModeIsActive() && !isStopRequested()) {
            runToHome.update(gamepad1.x);
            if (!shoulder.IsBusy()) shoulder.SetPower(gamepad1.right_stick_y);
            else if (Math.abs(gamepad1.right_stick_y) > 0) {
                shoulder.Stop();
                shoulder.SetPower(gamepad1.right_stick_y);
            }
            shoulder.Update();
        }
        waitForStart();
        wrist.setPosition(1);
        while (opModeIsActive()) {
            runToPos.update(gamepad1.a);
            runToHeight.update(gamepad1.y);
            stopArm.update(gamepad1.b);
            runToHome.update(gamepad1.x);
            dpadUp.update(gamepad1.dpad_up);
            dpadDown.update(gamepad1.dpad_down);

            if (!shoulder.IsBusy()) shoulder.SetPower(gamepad1.right_stick_y);
            else if (Math.abs(gamepad1.right_stick_y) > 0) {
                shoulder.Stop();
                shoulder.SetPower(gamepad1.right_stick_y);
            }
            shoulder.Update();
            telemetry.addLine("  Controls Guide:");
            telemetry.addLine("A: Go to Target Angle");
            telemetry.addLine("B: Force Stop");
            telemetry.addLine("X: Home");
            telemetry.addLine("Y: Go to Target Height");
            telemetry.addLine("Right Stick Y: Up/Down");
            telemetry.addLine();
            telemetry.addLine("  Telemetry Info:");
            telemetry.addData("Motor Power",  shoulder.GetPower());
            telemetry.addData("Homed", shoulder.Homed());
            telemetry.addData("Is Busy", shoulder.IsBusy());
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos","Deg: "+shoulder.GetPos()+" Raw: "+shoulder.GetRawPos()+" Height: "+shoulder.GetHeight());
            telemetry.addData("Active Target Pos", "Deg: "+shoulder.GetTargetPos()+" Raw: "+shoulder.GetRawTargetPos());
            telemetry.update();
        }
    }
}
