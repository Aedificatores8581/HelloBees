package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class Turret_PID extends Hello_Bees_Teleop {
    private PIDController turret_controller;
    public static double p =0.004, i=0, d = 0.0001;
    private final double turret_ticks_in_degree = 0;

    @Override
    public void init(){
        turret_controller = new PIDController(p, i ,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop(){
        if(armAutomation) {
            turret_controller.setPID(p, i, d);
            turretMotorPower = turret_controller.calculate(armPos, turret_target);
        }
    }
}
