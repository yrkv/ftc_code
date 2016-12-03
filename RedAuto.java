package org.firstinspires.ftc.robotcontroller.external.samples.ftc_code;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
public class RedAuto extends LinearOpMode {

    Autonom

    @Override
    public void runOpMode() throws InterruptedException {

        Autonomous robot = new Autonomous();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        beaconServo.setPosition(0);
        
        robot.leftLauncherMotor.setPower(robot.getAdjustedPower(power));
        robot.rightLauncherMotor.setPower(-robot.getAdjustedPower(power));
        sleep(accelerationTime);
        robot.elevatorMotor.setPower(-1);
        sleep(elevatorTime);
        robot.elevatorMotor.setPower(0);robot.leftLauncherMotor.setPower(-0.75);robot.rightLauncherMotor.setPower(-0.75);
        sleep(500);
        robot.leftLauncherMotor.setPower(0);robot.rightLauncherMotor.setPower(0);
        sleep(4500);

        
        robot.leftLauncherMotor.setPower(robot.getAdjustedPower(power));
        robot.rightLauncherMotor.setPower(-robot.getAdjustedPower(power));
        sleep(accelerationTime);
        robot.elevatorMotor.setPower(-1);
        sleep(elevatorTime + 500);
        robot.elevatorMotor.setPower(0);robot.leftLauncherMotor.setPower(0);robot.rightLauncherMotor.setPower(0);
        sleep(500);


        robot.encoderDrive(0.5, 20, 20, 5);

        robot.gyroTurn(robot.getAdjustedPower(turnPower), -45);

        robot.lineColorSensor.enableLed(true);
        robot.encoderDriveToTape(5, 0.5, 57, 57, 5);
        robot.encoderDrive(1, -0.2, -0.2, 1);
        robot.encoderDriveToTape(5, 0.2, -5, -5, 3);
        robot.lineColorSensor.enableLed(false);

        robot.gyroTurn(robot.getAdjustedPower(turnPower), 90);
        
        robot.encoderDrive(0.4, -15, -15, 5);

        robot.encoderDriveToBeacon(3, 0.1, -10, -10, 3);

        robot.beaconServo.setPosition(robot.beaconColorSensor.blue() > robot.beaconColorSensor.red() ? 0 : 1); // adjust for actual servo positions

        robot.encoderDrive(0.5, -2, -2, 1);
        sleep(100);
        robot.encoderDrive(0.5, -4, -4, 1);

        telemetry.addData("Path2",  "Running at %7d :%7d",
        		robot.leftMotor.getCurrentPosition(),
        		robot.rightMotor.getCurrentPosition());
        telemetry.update();
        sleep(10000);
        telemetry.addData("Path", "Complete");
        //telemetry.addData("power", power);

        telemetry.update();
    }
}
