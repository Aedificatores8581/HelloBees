package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Drive;

@TeleOp (name = "Drive Test", group = "SubsysTest")
public class driveTest extends LinearOpMode {
    Drive drive;
    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            drive.Set(gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addLine("  Controls Guide:");
            telemetry.addLine("Left Stick Y: Forwards/Backwards");
            telemetry.addLine("Right Stick X: Steering (Like a Car Steering Wheel)");
            telemetry.addLine();
            telemetry.addLine("  Telemetry Info:");
            telemetry.addData("Motor Powers", "Left: "+drive.GetLeftPower()+" Right: "+drive.GetRightPower());
            telemetry.update();
        }
    }
}
