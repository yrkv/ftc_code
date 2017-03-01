/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.external.samples.ftc_code;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp
public class NewTroTeleOp extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor elevatorMotor;
    private DcMotor leftLauncherMotor;
    private DcMotor rightLauncherMotor;
    private DcMotor leftCapBallMotor;
    private DcMotor rightCapBallMotor;
    private DcMotor collectorMotor;
    private Servo leftServo;
    private Servo rightServo;
    private Servo capBallWinch;
    private ColorSensor beaconColorSensor;
    private ColorSensor lineColorSensor;
    private ModernRoboticsI2cGyro gyro;

    private int reverse = -1; // -1 when normal, 1 when reversed.
    private double movementMultiplier = 1;
    private boolean wasX1Pressed = false;
    private int rightServoPos = 1;
    private int leftServoPos = 0;
    private boolean wasY1Pressed = false;

    private double winchMaxPosition = 1;
    private double winchPosition = 0.58;
    private double winchMinPostion = 0;

    private double liftPower = 1;

    boolean wasRightStickPressed = false;
    boolean wasLeftBumperPressed = false;

    // private VoltageSensor voltageSensor;
    private double power = 0.4; // number of volts it tries to send.
    private double changeRate = 0.001;
    private boolean automaticBeacons = false;

    private int accelerationTime = 400;
    private int elevatorTime = 400;

    private boolean launch = false;

    private long startLaunch = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftMotor          = hardwareMap.dcMotor.get("left motor");
        rightMotor         = hardwareMap.dcMotor.get("right motor");
        elevatorMotor      = hardwareMap.dcMotor.get("elevator");
        leftLauncherMotor  = hardwareMap.dcMotor.get("left launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right launcher");
        leftCapBallMotor   = hardwareMap.dcMotor.get("left cap ball");
        rightCapBallMotor  = hardwareMap.dcMotor.get("right cap ball");
        collectorMotor     = hardwareMap.dcMotor.get("collector");
//
        beaconColorSensor = hardwareMap.colorSensor.get("beacon color sensor");
        lineColorSensor   = hardwareMap.colorSensor.get("line color sensor");
//
        lineColorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x3a));
//
//        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        capBallWinch = hardwareMap.servo.get("cap ball winch");

        leftServo = hardwareMap.servo.get("left servo");
        rightServo = hardwareMap.servo.get("right servo");
//        leftServo.scaleRange(0.86, 1);
//        rightServo.scaleRange(0.86, 1);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
    }

    private void toggleRightServo() {
        telemetry.addData("Toggled", " Servo");
        rightServoPos = rightServoPos == 1 ? 0 : 1;
        rightServo.setPosition(rightServoPos);
    }

    private void toggleLeftServo() {
        telemetry.addData("Toggled", " Servo");
        leftServoPos = leftServoPos == 1 ? 0 : 1;
        leftServo.setPosition(leftServoPos);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        lineColorSensor.enableLed(false);
        lineColorSensor.enableLed(true);

        rightServo.setPosition(rightServoPos);
        leftServo.setPosition(leftServoPos);
        capBallWinch.setPosition(winchPosition);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int t = 4;
        boolean onLine = (lineColorSensor.red() > t || lineColorSensor.green() > t || lineColorSensor.blue() > t);
        telemetry.addData("line color sensor", onLine);
        telemetry.addData("beaker color sensor", beaconColorSensor.red() + " " + beaconColorSensor.green() + " " + beaconColorSensor.blue());
        telemetry.addData("winchPos", winchPosition);

        if (gamepad2.b) collectorMotor.setPower(1);
        else collectorMotor.setPower(0);

        //Allow player 1 to drive using left and right joysticks
        if (reverse == 1) {
            leftMotor.setPower(gamepad1.right_stick_y * movementMultiplier);
            rightMotor.setPower(gamepad1.left_stick_y * movementMultiplier);
        }
        if (reverse == -1) {
            leftMotor.setPower(gamepad1.left_stick_y * movementMultiplier * reverse);
            rightMotor.setPower(gamepad1.right_stick_y * movementMultiplier * reverse);
        }

        if (gamepad1.right_bumper) {
            movementMultiplier = .2;
        }
        else if(gamepad1.left_bumper) {
            movementMultiplier = .5;
        }
        else {
            movementMultiplier = 1;
        }

        if (gamepad2.dpad_up) {
            leftCapBallMotor.setPower(liftPower);
            rightCapBallMotor.setPower(-liftPower);
        } else if (gamepad2.dpad_down) {
            leftCapBallMotor.setPower(-liftPower);
            rightCapBallMotor.setPower(liftPower);
        } else {
            leftCapBallMotor.setPower(gamepad2.dpad_left ? 0.5 : 0);
            rightCapBallMotor.setPower(gamepad2.dpad_right ? -0.5 : 0);
        }

        //The -1 gets rid of the natural inverse control of the joystick.
//        if (!launch)
        elevatorMotor.setPower(-1 * gamepad2.left_stick_y);



//        When A or Y is pressed, run the launchers at low or high power respectively
        if(gamepad2.a || gamepad2.y) {
            //Low power shot
            if (gamepad2.a) {
                leftLauncherMotor.setPower(-0.25);
                rightLauncherMotor.setPower(-0.25);
            }
            //High power shot
            if (gamepad2.y) {
                leftLauncherMotor.setPower(.35);
                rightLauncherMotor.setPower(.35);
            }
        }
        else {
            leftLauncherMotor.setPower(0);
            rightLauncherMotor.setPower(0);
        }

        //Toggle reverse when x is pressed.
        if (gamepad1.x == true && wasX1Pressed == false) {
            reverse *= -1;
        }
        wasX1Pressed = gamepad1.x;

        if (gamepad2.right_bumper && !wasRightStickPressed) {
            toggleRightServo();
        }
        wasRightStickPressed = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !wasLeftBumperPressed) {
            toggleLeftServo();
        }
        wasLeftBumperPressed = gamepad2.left_bumper;

        if(gamepad1.dpad_up && winchPosition < winchMaxPosition) {
            winchPosition += .001;
            capBallWinch.setPosition(winchPosition);
        }
        else if(gamepad1.dpad_down && winchPosition > winchMinPostion) {
            winchPosition -= .001;
            capBallWinch.setPosition(winchPosition);
        }

//        if (gamepad2.y) launch();
//        checkLaunch();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void checkLaunch() {
        if (System.currentTimeMillis() <= startLaunch + accelerationTime + elevatorTime + 100 && launch) {
            if (System.currentTimeMillis() >= startLaunch + accelerationTime) {
                elevatorMotor.setPower(1);
            }

            if (System.currentTimeMillis() >= startLaunch + accelerationTime + elevatorTime) {
                elevatorMotor.setPower(0);
                leftLauncherMotor.setPower(0);
                rightLauncherMotor.setPower(0);
                launch = false;
            }
        }
    }

    public void launch() {
//        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
//        double drop = power / 13.8;

//        if (voltage > power) {
            leftLauncherMotor.setPower(power);
            rightLauncherMotor.setPower(power);
//            leftLauncherMotor.setPower(power / (voltage - drop));
//            rightLauncherMotor.setPower(power / (voltage - drop));
            startLaunch = System.currentTimeMillis();
            launch = true;
//        }
    }
}
