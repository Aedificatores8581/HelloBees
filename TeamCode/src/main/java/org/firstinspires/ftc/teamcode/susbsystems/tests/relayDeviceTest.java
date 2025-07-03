package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.RelayDevice;

@TeleOp (name = "RelayDevice Test (Fan and Fogger)", group = "SubsysTest")
public class relayDeviceTest extends LinearOpMode {
    RelayDevice fan, fogger;
    boolean fanButtonBlock = false, foggerButtonBlock = false;
    //TODO: Implement Running for Time using Second Button Blocks and Use DPad Up/Down to Change the Time it Runs for
    boolean fanButtonBlock2 = false, foggerButtonBlock2 = false;
    boolean dpadUpBlock = false, dpadDownBlock = false;

    @Override
    public void runOpMode() {
        fan = new RelayDevice(hardwareMap, "");
        fogger = new RelayDevice(hardwareMap, "");

        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            // Fan Toggle on A Press
            if (gamepad1.a && !fanButtonBlock) {
                fanButtonBlock = true;
                fan.ToggleState();
            } else if (!gamepad1.a) fanButtonBlock = false;

            // Fogger Toggle on B Press
            if (gamepad1.b && !foggerButtonBlock) {
                foggerButtonBlock = true;
                fogger.ToggleState();
            } else if (!gamepad1.b) foggerButtonBlock = false;

            telemetry.addLine("  Controls Guide:");
            telemetry.addLine("Toggles: (A:Fan) (B:Fogger)");
            telemetry.addLine();
            telemetry.addLine("  Telemetry Info:");
            telemetry.addData("Fan State", fan.GetState());
            telemetry.addData("Fogger State", fogger.GetState());
            telemetry.update();
        }
    }
}
