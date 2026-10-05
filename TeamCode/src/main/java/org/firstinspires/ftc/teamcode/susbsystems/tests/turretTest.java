package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.susbsystems.RobotGeometry;
import org.firstinspires.ftc.teamcode.susbsystems.Turret725;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name = "Turret Test", group = "SubsysTest")
public class turretTest extends LinearOpMode {
    Turret725 turret725;
    ButtonBlock runToPos, stopTurret725, dpadUp, dpadDown;

    double targetPos = .241;
    @Override
    public void runOpMode() {
        turret725 = new Turret725(hardwareMap);

        runToPos = new ButtonBlock().onTrue(() -> {
            turret725.GoTo(targetPos);});
        stopTurret725 = new ButtonBlock().onTrue(() -> {
            turret725.Stop();});
        dpadUp = new ButtonBlock().onTrue(() -> {targetPos += 5;});
        dpadDown = new ButtonBlock().onTrue(() -> {targetPos -= 5;});

        telemetry.addLine("Initialized");
        telemetry.update();
        turret725.StartHome();
        while (!turret725.Homed() && !opModeIsActive() && !isStopRequested()) turret725.Update();
        waitForStart();
        while (opModeIsActive()) {
            runToPos.update(gamepad1.a);
            stopTurret725.update(gamepad1.b);
            dpadUp.update(gamepad1.dpad_up);
            dpadDown.update(gamepad1.dpad_down);

            if (!turret725.IsBusy()) turret725.SetPower(gamepad1.right_stick_x);
            else if (Math.abs(gamepad1.right_stick_x) > 0) {
                turret725.Stop();
                turret725.SetPower(gamepad1.right_stick_x);
            }
            turret725.Update();
            telemetry.addLine("  Controls Guide:");
            telemetry.addLine("A: Go to Target");
            telemetry.addLine("B: Force Stop");
            telemetry.addLine("Right Stick X: Rotation");
            telemetry.addLine();
            telemetry.addLine("  Telemetry Info:");
            telemetry.addData("Motor Power",  turret725.GetPower());
            telemetry.addData("In Error", turret725.InError());
            telemetry.addData("Homed", turret725.Homed());
            telemetry.addData("Is Busy", turret725.IsBusy());
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Current Pos:", "(Deg) "+ turret725.GetPos()+" (Raw) "+ turret725.GetRawPos());
            telemetry.addData("Active Target Pos", "Deg: N/A Raw: "+ turret725.GetRawTargetPos());
            telemetry.addData("Pos", "X: "+ turret725.getPosition().x+" Y: "+ turret725.getPosition().y+" Z:"+ turret725.getPosition().z);
            Position roboGeomPos = RobotGeometry.turretOffset(turret725.getNewcurrentAngleDegrees());
            telemetry.addLine("Robot Geometry X: "+ roboGeomPos.x+", Y: "+roboGeomPos.y);
            telemetry.update();
        }
    }
}
