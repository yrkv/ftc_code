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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private Servo leftServo;
    private Servo rightServo;
    private Servo collectorServo;
    private ColorSensor beaconColorSensor;
    private ColorSensor lineColorSensor;
    private ModernRoboticsI2cGyro gyro;

    private double collectorServoPosition = 0.5;
    private int reverse = -1; // -1 when normal, 1 when reversed.
    private double movementMultiplier = 1;
    private boolean wasX1Pressed = false;
    private int rightServoPos = 1;
    private int leftServoPos = 1;
    private boolean wasY1Pressed = false;

   // private VoltageSensor voltageSensor;
    private double power = 3.5; // number of volts it tries to send.
    private double changeRate = 0.001;
    private boolean automaticBeacons = false;

    private int accelerationTime = 3000;
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
//
//        beaconColorSensor = hardwareMap.colorSensor.get("beacon color sensor");
        lineColorSensor   = hardwareMap.colorSensor.get("line color sensor");
//
        lineColorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
//        beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x3a));
//
//        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        leftMotor = hardwareMap.servo.get("left servo");
        rightServo = hardwareMap.servo.get("right servo");
        collectorServo = hardwareMap.servo.get("collector servo");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
    }

    private void toggleRightServo() {
        if(rightServoPos == 1) {
            rightServoPos = 0;
        }
        else {
            rightServoPos = 1;
        }
        rightServo.setPosition(rightServoPos);
    }

    private void toggleLeftServo() {
        if(leftServoPos == 1) {
            leftServoPos = 0;
        }
        else {
            leftServoPos = 1;
        }
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

        rightServo.setPosition(1);
        leftServo.setPosition(1);
        collectorServo.setPosition(collectorServoPosition);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int t = 4;
        boolean onLine = (lineColorSensor.red() > t || lineColorSensor.green() > t || lineColorSensor.blue() > t);
        telemetry.addData("line color sensor", onLine);
        telemetry.addData("line Red:",lineColorSensor.red());
        telemetry.addData("line Green:",lineColorSensor.green());
        telemetry.addData("line Red:",lineColorSensor.blue());

        collectorServoPosition = gamepad2.b ? 1 : 0;
        collectorServo.setPosition(collectorServoPosition);

        //Allow player 1 to drive using left and right joysticks
//        if (gamepad1.right_stick_button) {
//        } else {
            if (reverse == 1) {
                leftMotor.setPower(gamepad1.right_stick_y*movementMultiplier);
                rightMotor.setPower(gamepad1.left_stick_y*movementMultiplier);
            }
            if (reverse == -1) {
                leftMotor.setPower(gamepad1.left_stick_y * movementMultiplier * reverse);
                rightMotor.setPower(gamepad1.right_stick_y * movementMultiplier * reverse);
            }

        if(gamepad1.right_bumper) {
            movementMultiplier = .5;
        }
        else {
            movementMultiplier = 1;
        }
//        }
        /* Umm this is fuqed
        if (gamepad2.right_stick_y != 0) {
            leftMotor.setPower(gamepad2.right_stick_y);
            rightMotor.setPower(-gamepad2.right_stick_y);
        } else {
            if (reverse == 1) {
                leftMotor.setPower(gamepad1.right_stick_y);
                rightMotor.setPower(gamepad1.left_stick_y);
            }
            if (reverse == -1) {
                leftMotor.setPower(gamepad1.left_stick_y * reverse);
                rightMotor.setPower(gamepad1.right_stick_y * reverse);
            }
        }*/

        //The -1 gets rid of the natural inverse control of the joystick.
        elevatorMotor.setPower(-1*gamepad2.right_stick_y);

        //When A or Y is pressed, run the launchers at low or high power respectively
        if(gamepad2.a || gamepad2.y) {
            //Low power shot
            if (gamepad2.a) {
                leftLauncherMotor.setPower(.45);
                rightLauncherMotor.setPower(.45);
            }
            //High power shot
            if (gamepad2.y) {
                leftLauncherMotor.setPower(.55);
                rightLauncherMotor.setPower(.55);
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
        //This variable is so that it doesnt run through the reverse loop
        //as fast as it can when the X button is pressed.
        wasX1Pressed = gamepad1.x;

        boolean wasRightBumperPressed = gamepad2.right_bumper;
        if(gamepad2.right_bumper && !wasRightBumperPressed) {
            toggleRightServo();
        }

        boolean wasLeftBumperPressed = gamepad2.left_bumper;
        if(gamepad2.left_bumper && !wasLeftBumperPressed) {
            toggleLeftServo();
        }your mom is a whore
//
//        if (gamepad2.y) launch();
//        checkLaunch();
//
//        power += changeRate * gamepad2.right_stick_y;
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
                elevatorMotor.setPower(-1);
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
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        double drop = power / 13.8;

        if (voltage > power) {
            leftLauncherMotor.setPower(power / (voltage - drop));
            rightLauncherMotor.setPower(-power / (voltage - drop));
            startLaunch = System.currentTimeMillis();
            launch = true;
        }
    }
}
