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
public class TroTeleOp extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor elevatorMotor;
   // private DcMotor leftCollectorMotor;
   // private DcMotor rightCollectorMotor
    private DcMotor leftLauncherMotor;
    private DcMotor rightLauncherMotor;
    private Servo beaconServo;
    private Servo collectorServo;
    private ColorSensor beaconColorSensor;
    private ColorSensor lineColorSensor;
    private ModernRoboticsI2cGyro gyro;
//    private DistanceSensor distanceSensor;


    private double beaconServoPosition = 0.5;
    private int reverse = 1; // 1 when normal, -1 when reversed.
    private boolean wasX1Pressed = false;
    private boolean wasX2Pressed = false;
    private boolean wasY1Pressed = false;
    private double speed = 1; // 0 to 1

   // private VoltageSensor voltageSensor;
    private double power = 3.5; // number of volts it tries to send.
    private double changeRate = 0.0005;
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
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        elevatorMotor = hardwareMap.dcMotor.get("elevator");
        
      //  leftCollectorMotor = hardwareMap.dcMotor.get("left collector");
      //  rightCollectorMotor = hardwareMap.dcMotor.get("right collector");
      
        leftLauncherMotor = hardwareMap.dcMotor.get("left launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right launcher");

        beaconColorSensor = hardwareMap.colorSensor.get("beacon color sensor");
        lineColorSensor = hardwareMap.colorSensor.get("line color sensor");

        lineColorSensor.setI2cAddress(I2cAddr.create8bit(0x3a));
        beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        beaconServo = hardwareMap.servo.get("beacon servo");
        collectorServo = hardwareMap.servo.get("collector servo");

        beaconServo.scaleRange(0.05, 0.95); // DO NOT REMOVE: Servo will break if you remove this line.

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
       // leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
       // rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");
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

        lineColorSensor.enableLed(true);

        beaconServo.setPosition(beaconServoPosition);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("power", power);
        telemetry.addData("voltage", hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage());
        telemetry.addData("rgb1", beaconColorSensor.red() + ", " + beaconColorSensor.green() + ", " + beaconColorSensor.blue());
        telemetry.addData("rgb2", lineColorSensor.red() + ", " + lineColorSensor.green() + ", " + lineColorSensor.blue());
     //   telemetry.addData("gyro", gyro.rawX() + ", " + gyro.rawY() + ", " + gyro.rawZ() + ", " + gyro.getIntegratedZValue());
        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        if (reverse == 1) {
            leftMotor.setPower(gamepad1.right_stick_y * speed);
            rightMotor.setPower(-gamepad1.left_stick_y * speed);
        }
        if (reverse == -1) {
            leftMotor.setPower(-gamepad1.left_stick_y * speed);
            rightMotor.setPower(gamepad1.right_stick_y * speed);
        }



      //  elevatorMotor.setPower(gamepad2.left_stick_y);

        collectorServo.setPosition(gamepad2.right_stick_y);

        telemetry.addData("left encoder", leftMotor.getCurrentPosition());
        telemetry.addData("right encoder", rightMotor.getCurrentPosition());

        if (gamepad1.x == true && wasX1Pressed == false) {
            reverse *= -1;
        }
        wasX1Pressed = gamepad1.x;

        if (gamepad1.y == true && wasY1Pressed == false) {
            reverse *= -1;
        }
        wasY1Pressed = gamepad1.y;

        if (gamepad2.x == true && wasX2Pressed == false) {
            beaconServoPosition = beaconServoPosition == 0 ? 1 : 0;
            beaconServo.setPosition(beaconServoPosition);
        }
        wasX2Pressed = gamepad2.x;

        if (gamepad2.y) launch();
        checkLaunch();

        if (gamepad2.a) power -= changeRate;
        if (gamepad2.b) power += changeRate;

        beaconColorSensor.enableLed(gamepad2.dpad_down);
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
