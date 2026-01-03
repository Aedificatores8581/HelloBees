package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Wrist;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;
@Disabled
@TeleOp(name = "Wrist Test", group = "SubsysTest")
public class WristTest extends LinearOpMode{
    Wrist wrist;
    ButtonBlock runToPos, stopArm, dpadUp, dpadDown,runToHeight;
    double local_right_stick_y;

    double targetPos = 0;
    @Override
    public void runOpMode() {
        wrist = new Wrist(hardwareMap);

        runToPos = new ButtonBlock().onTrue(() -> {wrist.GoToAngle(targetPos);});
        stopArm = new ButtonBlock().onTrue(() -> {wrist.Stop();});
        dpadUp = new ButtonBlock().onTrue(() -> {targetPos += 1;});
        dpadDown = new ButtonBlock().onTrue(() -> {targetPos -= 1;});
        runToHeight = new ButtonBlock().onTrue(() -> {wrist.GoToHeight(targetPos);});

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            local_right_stick_y = gamepad1.right_stick_y;
            runToPos.update(gamepad1.a);
            runToHeight.update(gamepad1.y);
            stopArm.update(gamepad1.b);
            dpadUp.update(gamepad1.dpad_up);
            dpadDown.update(gamepad1.dpad_down);

            if (!wrist.IsBusy()) {
                if (local_right_stick_y>0)
                {
                    wrist.SetPos(wrist.GetRawPos()+.001);
                }
                if (local_right_stick_y<0)
                {
                    wrist.SetPos(wrist.GetRawPos()-.001);
                }
            }
            wrist.Update();
            telemetry.addLine("  Controls Guide:");
            telemetry.addLine("A: Go to Target Angle");
            telemetry.addLine("B: Force Stop");
            telemetry.addLine("X: Home");
            telemetry.addLine("Y: Go to Target Height");
            telemetry.addLine("Right Stick Y: Up/Down");
            telemetry.addLine();
            telemetry.addLine("  Telemetry Info:");
            telemetry.addData("Is Busy", wrist.IsBusy());
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos","Deg: "+wrist.GetPos()+" Raw: "+wrist.GetRawPos()+" Height: "+wrist.GetHeight());
            telemetry.addData("Active Target Pos", "Deg: "+wrist.GetTargetPos()+" Raw: "+wrist.GetRawTargetPos()+" T_Height: "+wrist.GetTargetHeight());
            telemetry.update();
        }
    }
}
