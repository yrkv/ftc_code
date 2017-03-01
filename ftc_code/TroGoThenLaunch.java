package org.firstinspires.ftc.robotcontroller.external.samples.ftc_code;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
public class TroGoThenLaunch extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    static final double PI = 3.14159;
    static final double COUNTS_PER_INCH = 250 / PI;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor elevatorMotor;
    private DcMotor leftLauncherMotor;
    private DcMotor rightLauncherMotor;

    private Servo leftServo;
    private Servo rightServo;
    private Servo capBallWinch;

    private double turnPower = (0.25);
    private double turnMultiplier = 1.8;

    private ColorSensor lineColorSensor;
    private ColorSensor beaconColorSensor;

    private int launcherCountsPerSecond = (int)(44.4 * 20);
    private double currentPower = 0.3;

    private int elevatorTime = 600;

    private double launchPower = (0.23);

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor          = hardwareMap.dcMotor.get("left motor");
        rightMotor         = hardwareMap.dcMotor.get("right motor");
        elevatorMotor      = hardwareMap.dcMotor.get("elevator");
        leftLauncherMotor  = hardwareMap.dcMotor.get("left launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right launcher");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        lineColorSensor = hardwareMap.colorSensor.get("line color sensor");
        beaconColorSensor = hardwareMap.colorSensor.get("beacon color sensor");

        lineColorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x3a));

        leftServo = hardwareMap.servo.get("left servo");
        rightServo = hardwareMap.servo.get("right servo");
        capBallWinch = hardwareMap.servo.get("cap ball winch");

        leftServo.setPosition(0);
        rightServo.setPosition(1);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftLauncherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lineColorSensor.enableLed(false);
        beaconColorSensor.enableLed(true);
        beaconColorSensor.enableLed(false);

        capBallWinch.setPosition(0.675);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        encoderDrive(0.4, 24, 24, 5);

//        encoderRotateWithGyro(-4);

        launchBalls();

//        encoderRotateWithGyro(4);

        sleep(15000);

        encoderDrive(0.4, 40, 40, 5);
    }

    private void launchBalls() throws InterruptedException {
//        double p = getAdjustedPower(launchPower);

        double p = launchPower;

        leftLauncherMotor.setPower(p);
        rightLauncherMotor.setPower(p);
        sleep(400);
        elevatorMotor.setPower(1);
        sleep(elevatorTime);
        elevatorMotor.setPower(0);
        leftLauncherMotor.setPower(0);
        rightLauncherMotor.setPower(0);
        sleep(200);
        leftLauncherMotor.setPower(p);
        rightLauncherMotor.setPower(p);
        sleep(400);
        elevatorMotor.setPower(1);
        sleep(elevatorTime);
        elevatorMotor.setPower(0);
        leftLauncherMotor.setPower(0);
        rightLauncherMotor.setPower(0);
    }

    private double getAdjustedPower(double p) {
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        double drop = p / 13.8;
        return launchPower / (voltage - drop);
    }

//    public void gyroTurn (double speed, double angle, double err)
//            throws InterruptedException {
//
//        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && !onHeading(speed, angle, 0.1, err)) {
//            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//            idle();
//        }
//    }

    //    void launch(double currentPower) throws InterruptedException {
    void launch() throws InterruptedException {
        int leftEncoder = leftLauncherMotor.getCurrentPosition();
        int rightEncoder = rightLauncherMotor.getCurrentPosition();

        sleep(100);
        boolean launch = false;
        while (true) {
            int currentSpeed = (leftLauncherMotor.getCurrentPosition() - leftEncoder + rightLauncherMotor.getCurrentPosition() - rightEncoder) / 2 * 10;

            leftEncoder = leftLauncherMotor.getCurrentPosition();
            rightEncoder = rightLauncherMotor.getCurrentPosition();

            if (Math.abs(currentSpeed - launcherCountsPerSecond) <= 1) {
                if (launch) break;
                launch = true;
            }
            else if (currentSpeed < launcherCountsPerSecond) {
                currentPower += 0.01;
            }
            else {
                currentPower -= 0.01;
            }
            leftLauncherMotor.setPower(currentPower);
            rightLauncherMotor.setPower(currentPower);
            //Update the displays
            telemetry.addData("power", currentPower);
            telemetry.addData("left", leftLauncherMotor.getCurrentPosition() / 44.4 / runtime.seconds());
            telemetry.addData("right", rightLauncherMotor.getCurrentPosition() / 44.4 / runtime.seconds());
            telemetry.update();
            sleep(100);
        }
    }
/*

    boolean onHeading(double speed, double angle, double PCoeff, double err) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn launchPower based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= err) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);

            double multiplier = Math.abs(error) / 30 * 0.8 + 0.2;

            rightSpeed  = speed * steer * (Math.abs(error) > 30 ? 1 : multiplier);
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
*/

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderDriveToTape(int threshold, double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy()) &&
                    (lineColorSensor.red() < threshold && lineColorSensor.green() < threshold && lineColorSensor.blue() < threshold)) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderDriveToBeacon(int threshold, double speed,
                                     double leftInches, double rightInches,
                                     double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy()) &&
                    (beaconColorSensor.red() < threshold && beaconColorSensor.blue() < threshold)) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderRotate(double speed, int degrees, double timeoutS) throws InterruptedException {
        double inches = degrees / 45.0 * Math.PI * 2 * 1.1625;
        if (degrees < 0) inches -= 0.15;
        else inches += 0.15;
        encoderDrive(speed, inches, -inches, timeoutS);
    }

    public void encoderDriveCounts(double speed,
                                   int leftCounts, int rightCounts,
                                   double timeoutS, int target) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + leftCounts;
            newRightTarget = rightMotor.getCurrentPosition() + rightCounts;
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();

                double leftSpeed = (double) (leftMotor.getCurrentPosition() - newLeftTarget) / (Math.abs(leftCounts) + 50) * speed;
                double rightSpeed = (double) (rightMotor.getCurrentPosition() - newRightTarget) / (Math.abs(rightCounts) + 50) * speed;

                leftMotor.setPower(Range.clip(Math.abs(leftSpeed) + 0.075, -1, 1));
                rightMotor.setPower(Range.clip(Math.abs(rightSpeed) + 0.075, -1, 1));

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderRotateWithGyro(int degrees) throws InterruptedException {
        sleep(100); // optional sleep for increased accuracy
        encoderDriveCounts(0.25, (int) (13.16*degrees), (int) (-13.16*degrees), Math.abs(degrees/45.0 ) + 0.5, -degrees);
    }
}