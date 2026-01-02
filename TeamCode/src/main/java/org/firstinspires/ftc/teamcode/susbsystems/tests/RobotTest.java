package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.Pump_Subsystem;
import org.firstinspires.ftc.teamcode.susbsystems.robot_system;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name = "Robot Test", group = "SubsysTest")
public class RobotTest extends OpMode {
    robot_system robot;
    ButtonBlock pumpToggle, fanToggle;
    robot_system.Subsystems subsystem;

    @Override
    public void init() {
        robot = new robot_system(hardwareMap);
        pumpToggle = new ButtonBlock()
                .onTrue(() -> {robot.doSomething(subsystem.PUMP);});
        fanToggle = new ButtonBlock()
                .onTrue(() -> {robot.doSomething(subsystem.FAN);});
        telemetry.addLine("Initialized");
        telemetry.update();

    }
    @Override
    public void loop() {
        buttonEvents();
        robot.robot_drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        robot.update();
        telemetry();
    }
    private void buttonEvents() {
        // Simplified down all into classes that handle button blocks the same way that a normal one does
        pumpToggle.update(gamepad1.a);
        fanToggle.update(gamepad1.b);
    }

    @Override
    public void stop() {
    }
    private void telemetry() {
        telemetry.addLine("  Controls Guide:");
        telemetry.addLine("Toggles: (A:Pump) (B:Fan)");
        telemetry.addLine("Run For Time: (X:Pump) Run For Encoder: (Y:Pump)");
        telemetry.addLine("Toggle Direction (Y)");
        telemetry.addLine("+/- RunFor: (DPadUp:+) (DPadDown:-)");
        /*telemetry.addLine();
        telemetry.addLine("  Telemetry Info:");
        telemetry.addData("Pump: ","(On/Off)"+pump.GetState()+" Fan: (On/Off)"+fan.GetState());
        telemetry.addData("Target Position: ", pump.GetTarget()+" Busy: "+pump.isBusy());
        telemetry.addData("Pump Direction: ", pump.Direction());*/
        telemetry.update();
    }
}
