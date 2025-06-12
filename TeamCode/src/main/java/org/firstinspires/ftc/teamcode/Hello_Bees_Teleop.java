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
    private DigitalChannel valve0;
    private DigitalChannel valve1;
    private DigitalChannel compressor0;
    private DcMotor linkage;
    private DigitalChannel compressor1;
    private DcMotor pump;
    private DigitalChannel turret_home;
    private DcMotor turret;
    private DigitalChannel front_limit;
    private DigitalChannel rear_limit;
    private CRServo shoulder;
    private AnalogInput pot1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        boolean AutoBlock;
        boolean PumpButtonBlock;
        boolean ButtonBlockValuePump;
        boolean CompressorButtonBlock;
        boolean ValveButtonBlock;
        boolean ButtonBblock;
        boolean ButtonAblock;
        double armPos;

        rightdrive = hardwareMap.get(CRServo.class, "rightdrive");
        leftdrive = hardwareMap.get(CRServo.class, "leftdrive");
        wrist = hardwareMap.get(Servo.class, "wrist");
        valve0 = hardwareMap.get(DigitalChannel.class, "valve0");
        valve1 = hardwareMap.get(DigitalChannel.class, "valve1");
        compressor0 = hardwareMap.get(DigitalChannel.class, "compressor0");
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
        valve0.setMode(DigitalChannel.Mode.INPUT);
        valve1.setMode(DigitalChannel.Mode.INPUT);
        compressor0.setMode(DigitalChannel.Mode.INPUT);
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
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
