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
public class AutoRed extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
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

    private ModernRoboticsI2cGyro gyro;
    private ColorSensor lineColorSensor;
    private ColorSensor beaconColorSensor;

    private int launcherCountsPerSecond = (int)(44.4 * 20);
    private double currentPower = 0.3;

    private int elevatorTime = 400;

    private double launchPower = (0.268);

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor          = hardwareMap.dcMotor.get("left motor");
        rightMotor         = hardwareMap.dcMotor.get("right motor");
        elevatorMotor      = hardwareMap.dcMotor.get("elevator");
        leftLauncherMotor  = hardwareMap.dcMotor.get("left launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right launcher");

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        lineColorSensor = hardwareMap.colorSensor.get("line color sensor");
        beaconColorSensor = hardwareMap.colorSensor.get("beacon color sensor");

        lineColorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x3a));

        leftServo = hardwareMap.servo.get("left servo");
        rightServo = hardwareMap.servo.get("right servo");
        capBallWinch = hardwareMap.servo.get("cap ball winch");
        leftServo.scaleRange(0.86, 1);
        rightServo.scaleRange(0.86, 1);

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

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        int counter = 0;
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
            telemetry.addData(">", counter);
            telemetry.update();
            counter++;
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        leftServo.setPosition(0);
        rightServo.setPosition(0);

        encoderDrive(0.4, 16, 16, 5);

        encoderRotateWithGyro(-6);
        launchBalls();


        //Turn to -135 degrees.
        encoderRotateWithGyro(134);

        //Use the line sensor to drive to the beacon tape.
        lineColorSensor.enableLed(true);
        encoderDriveToTape(5, 0.75, -68, -68, 5);
        encoderDrive(1, 0.2, 0.2, 1);
        encoderDriveToTape(5, 0.2, 5, 5, 3);
        lineColorSensor.enableLed(false);

        //Turn to 90 degrees (facing the beacon)
        encoderRotateWithGyro(262);

        //Move closer to the beacon for the sensor.
        encoderDriveToBeacon(3, 0.2, 10, 10, 3);

        //Determine which side the red side is on and move the beacon bumper in front of it.
        if (beaconColorSensor.blue() < beaconColorSensor.red()) {
            leftServo.setPosition(1);
        }
        else {
            rightServo.setPosition(1);
        }

        //Bump the button and back up.
        encoderDrive(0.15, 9, 9, 2);
        encoderDrive(0.5, -6, -6, 1);

        //Reset servo positions.
        leftServo.setPosition(0);
        rightServo.setPosition(0);

        //Turn to the other beacon's tape.
        encoderRotateWithGyro(345);

        //Drive up to the other beacon's tape.
        encoderDrive(1, 5, 5, 2);
        lineColorSensor.enableLed(true);
        encoderDriveToTape(5, 0.65, 50, 50, 5);
        encoderDrive(1, -0.2, -0.2, 1);
        encoderDriveToTape(5, 0.2, -5, -5, 3);
        lineColorSensor.enableLed(false);

        //Turn to 90 degrees (facing the beacon).
        encoderRotateWithGyro(260);

        //Move closer to the beacon for the sensor.
        encoderDriveToBeacon(3, .2, 10, 10, 3);

        //Determine which side the red side is on and move the beacon bumper in front of it.
        if (beaconColorSensor.blue() < beaconColorSensor.red()) {
            leftServo.setPosition(1);
        }
        else {
            rightServo.setPosition(1);
        }

        //Bump the button and back up.
        encoderDrive(0.15, 9, 9, 2);
        encoderDrive(0.5, -6, -6, 1);

        leftServo.setPosition(0);
        rightServo.setPosition(0);

        //Turn toward at the cap ball.
//        gyroTurn(turnPower, 145, 1);

        encoderRotateWithGyro(135);

        //Drive to the cap ball to bump it.
        encoderDrive(0.5, 60, 60, 15);

        telemetry.addData("Path2",  "Running at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.update();
        telemetry.addData("Path", "Complete");

        telemetry.update();
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

    public void gyroTurn (double speed, double angle, double err)
            throws InterruptedException {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, 0.1, err)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            idle();
        }
    }

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
                telemetry.addData("gyro", gyro.getIntegratedZValue());
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
        int degDiff = degrees + gyro.getIntegratedZValue();
        encoderDriveCounts(0.25, (int) (13.16*degDiff), (int) (-13.16*degDiff), Math.abs(degDiff/45.0 ) + 0.5, -degrees);
    }
}