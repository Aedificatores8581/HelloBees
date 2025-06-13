/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Hello Bees-Java", group="Iterative OpMode")
public class Hello_Bees_Teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo rightdrive;
    private CRServo leftdrive;
    private Servo wrist;
    private DigitalChannel valve1;
    private DcMotor linkage;
    private DigitalChannel compressor1;
    private DcMotor pump;
    private DigitalChannel turret_home;
    private DcMotor turret;
    private DigitalChannel front_limit;
    private DigitalChannel rear_limit;
    private CRServo shoulder;
    private AnalogInput pot1;
    boolean AutoBlock;
    boolean PumpButtonBlock;
    boolean ButtonBlockValuePump;
    boolean CompressorButtonBlock;
    boolean ValveButtonBlock;
    boolean ButtonBblock;
    boolean ButtonAblock;
    double armPos = 0;
    double linkageMotorPower = 0;
    double wristServoPosition = 0;
    double shoulderMotorPower = 0;
    double turretMotorPower = 0;
    boolean armAutomation = false;
    double pumpMotorPower = 0;
    boolean fanRelay;
    boolean foggerRelay;
    boolean frontLinkageLimit;
    boolean rearLinkageLimit;
    boolean turretHomeSensor;
    double rightdriveMotorPower = 0;
    double leftdriveMotorPower = 0;
    double linkageMotorPosition;
    double turretMotorPosition;
    boolean turretHomed = false;
    int automationState = 0;
    boolean armStowed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        rightdrive = hardwareMap.get(CRServo.class, "rightdrive");
        leftdrive = hardwareMap.get(CRServo.class, "leftdrive");
        wrist = hardwareMap.get(Servo.class, "wrist");
        valve1 = hardwareMap.get(DigitalChannel.class, "valve1");
        linkage = hardwareMap.get(DcMotor.class, "linkage");
        compressor1 = hardwareMap.get(DigitalChannel.class, "compressor1");
        pump = hardwareMap.get(DcMotor.class, "pump");
        turret_home = hardwareMap.get(DigitalChannel.class, "turret_home");
        turret = hardwareMap.get(DcMotor.class, "turret");
        front_limit = hardwareMap.get(DigitalChannel.class, "front_limit");
        rear_limit = hardwareMap.get(DigitalChannel.class, "rear_limit");
        shoulder = hardwareMap.get(CRServo.class, "shoulder");
        pot1 = hardwareMap.get(AnalogInput.class, "pot1");

        rightdrive.setDirection(CRServo.Direction.REVERSE);
        leftdrive.setDirection(CRServo.Direction.REVERSE);
        // Put initialization blocks here.
        wrist.setPosition(0.5);
        valve1.setMode(DigitalChannel.Mode.INPUT);
        linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        compressor1.setMode(DigitalChannel.Mode.INPUT);
        ButtonBlockValuePump = false;
        CompressorButtonBlock = false;
        ValveButtonBlock = false;
        PumpButtonBlock = false;
        ButtonBblock = false;
        AutoBlock = false;
        ButtonAblock = false;
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        sensorRead();
        telemetry();
        home_turret();
    }
    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        valve1.setMode(DigitalChannel.Mode.OUTPUT);
        valve1.setState(true);
        // Put run blocks here.
        compressor1.setMode(DigitalChannel.Mode.OUTPUT);
        compressor1.setState(true);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Put loop blocks here.
        sensorRead();
        telemetry ();
        controllersGetValues ();
        if(armAutomation){
            automaticSpray();
        }
        actuatorCommands();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void telemetry (){
        telemetry.addData("Pump Power", pumpMotorPower);
        telemetry.addData("Fan Relay (False On)", fanRelay);
        telemetry.addData("Front Linkage Limit", frontLinkageLimit);
        telemetry.addData("Rear Linkage Limit", rearLinkageLimit);
        telemetry.addData("Turret Limit", turretHomeSensor);
        telemetry.addData("Fogger (False On)", foggerRelay);
        telemetry.addData("Turret Position", turretMotorPosition);
        telemetry.addData("armPos", armPos);
        telemetry.addData("servo", wristServoPosition);
        telemetry.addData("linkage", linkageMotorPosition);
        telemetry.addData("Automation State:", "State ", automationState,"Stowed ", armStowed, "Auto On ", armAutomation, "Homed " , turretHomed);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftdriveMotorPower, rightdriveMotorPower);
    }
    private void home_turret(){
        if(!turretHomed){
            turret.setPower(-.3);
            if(!turretHomeSensor){
                turret.setPower(0);
                turretHomed = true;
            }
        }
    }
    private void sensorRead (){
        //shoulder position
        armPos = pot1.getVoltage();

        foggerRelay = compressor1.getState();
        fanRelay = valve1.getState();
        frontLinkageLimit = front_limit.getState();
        rearLinkageLimit = rear_limit.getState();
        turretHomeSensor = turret_home.getState();
        turretMotorPosition = turret.getCurrentPosition();
        linkageMotorPosition = linkage.getCurrentPosition();
    }

    private void actuatorCommands (){
        //drivetrain motors
        leftdriveMotorPower = Math.min(Math.max(leftdriveMotorPower, -.3),.3); //movement safety
        rightdriveMotorPower = Math.min(Math.max(rightdriveMotorPower, -.3),.3); //movement safety
        leftdrive.setPower(leftdriveMotorPower);
        rightdrive.setPower(rightdriveMotorPower);

        //wrist servo
        wristServoPosition = Math.min(Math.max(wristServoPosition, .44),1); //movement safety
        wrist.setPosition(wristServoPosition);

        //linkage movement
        if (!frontLinkageLimit) {
            linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            linkageMotorPower = Math.min(Math.max(linkageMotorPower, 0),1); //movement safety
        }
        if (!rearLinkageLimit) {
            linkageMotorPower = Math.min(Math.max(linkageMotorPower, -1),0); //movement safety
        }
        linkage.setPower(linkageMotorPower);

        //turret movement
        if (!turret_home.getState()) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turretMotorPower = Math.min(Math.max(turretMotorPower, -.6),.6); //movement safety
        if(turretHomed && turretMotorPosition <-2000){
            turretMotorPower = Math.min(Math.max(turretMotorPower, 0),.6); //movement safety
        }
        if(turretHomed && turretMotorPosition >14000){
            turretMotorPower = Math.min(Math.max(turretMotorPower, -0.6),0); //movement safety
        }
        turret.setPower(turretMotorPower);

        //shoulder movement
        shoulderMotorPower = Math.min(Math.max(shoulderMotorPower, -.2),.2); //movement safety
        if(armPos <.35){
            shoulderMotorPower = Math.min(Math.max(shoulderMotorPower, -.2),0); //movement safety
        }
        if(armPos > 2.437){
            shoulderMotorPower = Math.min(Math.max(shoulderMotorPower, 0),.2); //movement safety
        }
        shoulder.setPower(shoulderMotorPower);

        //pump motor
        pump.setPower(pumpMotorPower);

        //fan relay
        valve1.setState(fanRelay);

        //Fogger relay
        compressor1.setState(foggerRelay);
    }

    private void controllersGetValues (){
        //drive train controls
        rightdriveMotorPower = Math.min(Math.max(-gamepad1.left_stick_y * 0.3, -0.3), 0.3);
        leftdriveMotorPower = Math.min(Math.max(gamepad1.right_stick_y * 0.3, -0.3), 0.3);

        //automate spray
        if (gamepad1.x){
            automationState = 1;
            armAutomation = true;
            automaticSpray();
        }
        if (gamepad1.y){
            armAutomation = false;
            automationState = 0;
        }
        if (gamepad1.a){
            armAutomation = true;
            automationState = 99;
        }
        //pump controls
        if (gamepad2.y && !ButtonBlockValuePump) {
            ButtonBlockValuePump = true;
            if (pumpMotorPower == 0) {
                pumpMotorPower = -1;
            } else {
                pumpMotorPower = 0;
            }
        } else if (!gamepad2.y) {
            ButtonBlockValuePump = false;
        }
        if (gamepad2.a && !ButtonAblock) {
            ButtonAblock = true;
            if (pumpMotorPower == 0) {
                pumpMotorPower = 1;
            } else {
                pumpMotorPower = 0;
            }
        } else if (!gamepad2.a) {
            ButtonAblock = false;
        }

        //fan relay controls
        if (gamepad2.x && !ValveButtonBlock) {
            ValveButtonBlock = true;
            fanRelay = !fanRelay;
        } else if (!gamepad2.x) {
            ValveButtonBlock = false;
        }

        //fogger relay controls
        if (gamepad2.b && !CompressorButtonBlock) {
            CompressorButtonBlock = true;
            foggerRelay = !foggerRelay;
        } else if (!gamepad2.b) {
            CompressorButtonBlock = false;
        }

        //manual arm controls
        if (!armAutomation) {
            //linkage manual controls
            if (gamepad2.dpad_left) {
                linkageMotorPower = .5;
            } else if (gamepad2.dpad_right) {
                linkageMotorPower = -0.5;
            } else if (!gamepad1.dpad_up && !gamepad1.dpad_up) {
                linkageMotorPower = 0;
            }

            //wrist servo manual controls
            if (gamepad2.dpad_up) {
                wristServoPosition = Math.min(Math.max(wristServoPosition + 0.01, 0), 1);
            } else if (gamepad2.dpad_down) {
                wristServoPosition = Math.min(Math.max(wristServoPosition - 0.01, 0), 1);
            }

            //turret manual controls
            turretMotorPower = gamepad2.right_trigger - gamepad2.left_trigger;
            shoulderMotorPower = -gamepad2.right_stick_y;
        }
    }

    private void automaticSpray(){
        if(automationState ==0) {
            armAutomation = false;
        }
        else if(automationState == 1) {
            if(armStowed){
                automationState = 2;
            }
            else{
                automationState = 99;
            }
        }
        else if(automationState == 2){
            automationState = 3;
            linkage.setTargetPosition(100);
            linkage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linkageMotorPower = 1;
            turret.setTargetPosition(1500);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotorPower = 1;
        }
        else if(automationState == 99){
            automationState = 98;
            linkage.setTargetPosition(0);
            linkage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linkageMotorPower = .5;
            turret.setTargetPosition(6000);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotorPower = .5;
            if(armPos>.5) {
                shoulderMotorPower = -.2;
            }
            if(armPos<.5){
                shoulderMotorPower  = .2;
            }
            if(armPos ==.5) {
                shoulderMotorPower = 0;
            }
        }
        else if(automationState == 98){
            if(!linkage.isBusy() &&!turret.isBusy() && armPos>.49 && armPos < .51){
                linkageMotorPower = 0;
                linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorPower = 0;
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shoulderMotorPower = 0;
                armStowed = true;
                automationState = 0;
                armAutomation = false;
            }
        }
        else{
            armAutomation = false;
        }
    }

}
