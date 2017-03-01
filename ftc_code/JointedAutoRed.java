package org.firstinspires.ftc.robotcontroller.external.samples.ftc_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.ftc_code.navx.AHRS;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
public class JointedAutoRed extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    static final double PI = 3.14159;
    static final double COUNTS_PER_INCH = 270 / PI;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor elevatorMotor;
    private DcMotor leftLauncherMotor;
    private DcMotor rightLauncherMotor;

    private Servo leftServo;
    private Servo rightServo;
    private Servo capBallWinch;

    private double turnPower = 0.25;
    private double minTurnPower = 0.025;
    private double turningThreshold = 0.15;
    private double encoderTicksPerDegree = 13.2;

    private ColorSensor lineColorSensor;
    private ColorSensor beaconColorSensor;

    private int launcherCountsPerSecond = (int)(44.4 * 20);
    private double currentPower = 0.3;

    private int elevatorTime = 600;

    private double launchPower = (0.23);

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor          = hardwareMap.dcMotor.get("left motor");
        rightMotor         = hardwareMap.dcMotor.get("right motor");
        elevatorMotor      = hardwareMap.dcMotor.get("elevator");
        leftLauncherMotor  = hardwareMap.dcMotor.get("left launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right launcher");

        // adjust the launching power based off of the battery voltage
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        idle();
        if (voltage >= 13.8)
            launchPower *= 0.95;

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        lineColorSensor = hardwareMap.colorSensor.get("line color sensor");
        beaconColorSensor = hardwareMap.colorSensor.get("beacon color sensor");

        lineColorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x3a));

        leftServo = hardwareMap.servo.get("left servo");
        rightServo = hardwareMap.servo.get("right servo");

        leftServo.setPosition(0);
        rightServo.setPosition(1);

        capBallWinch = hardwareMap.servo.get("cap ball winch");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftLauncherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lineColorSensor.enableLed(false);
        beaconColorSensor.enableLed(true);
        beaconColorSensor.enableLed(false);

        capBallWinch.setPosition(0.675);

        long startCalibration = System.currentTimeMillis();
        while (navx_device.isCalibrating() && opModeIsActive()) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            telemetry.addData("Calibration Time", System.currentTimeMillis() - startCalibration);
            telemetry.update();
            idle();
        }
        navx_device.zeroYaw();
        telemetry.addData("navX-Micro", "Calibrated");
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

//        encoderDriveLaunchAtEnd(0.4, 16, 16, 5, 450);
        encoderDrive(0.4, 16, 16, 5);
        launchBalls();


        encoderRotateWithGyro(-54, 0.3);
        encoderRotateWithGyro(-54, 0.3);
        encoderDrive(0.7, 39.5, 39.5, 5);
        encoderRotateWithGyro(-19, 1.5);
        encoderRotateWithGyro(-19, 1.5);

        //Use the line sensor to drive to the beacon tape.
        lineColorSensor.enableLed(true);
        encoderDrive(0.45, 10, 10, 5);
        sleep(50);
        encoderDriveToTape(5, 0.1, 6, 6, 1);
        lineColorSensor.enableLed(false);
        sleep(50);


        //Turn to 90 degrees (facing the beacon)
        encoderRotateWithGyro(-90, 0.3);
        encoderRotateWithGyro(-90, 0.3);

        // reset the servos
        leftServo.setPosition(1);
        rightServo.setPosition(0);

        //Move closer to the beacon for the sensor.
        encoderDriveToBeacon(3, 0.2, 10, 10, 3);

        //Determine which side the red side is on and move the beacon bumper in front of it.
        if (beaconColorSensor.blue() < beaconColorSensor.red()) {
            leftServo.setPosition(0);
        }
        else {
            rightServo.setPosition(1);
        }

        sleep(100); // wait for the servo to move.

        //Bump the button and back up.
        encoderDrive(0.2, 4, 4, 3);
        encoderDrive(0.45, -4, -4, 2);

        //Reset servo positions.
        leftServo.setPosition(1);
        rightServo.setPosition(0);


        //Turn to the other beacon's tape.
        encoderRotateWithGyro(0, 0.6);
        encoderRotateWithGyro(0, 0.6);

        //Drive up to the other beacon's tape.
//        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lineColorSensor.enableLed(true);
        encoderDrive(0.8, 27, 27, 5);
        encoderDrive(0.8, 0.15, 13, 13, 6);
        encoderDriveToTape(5, 0.15, 10, 10, 3);
        lineColorSensor.enableLed(false);
//        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Turn to 90 degrees (facing the beacon).
        encoderRotateWithGyro(-90, 0.3);
        encoderRotateWithGyro(-90, 0.3);

        leftServo.setPosition(1);
        rightServo.setPosition(0);
        //Move closer to the beacon for the sensor.
        encoderDriveToBeacon(3, .2, 10, 10, 3);

        //Determine which side the red side is on and move the beacon bumper in front of it.
        if (beaconColorSensor.blue() < beaconColorSensor.red()) {
            leftServo.setPosition(0);
        }
        else {
            rightServo.setPosition(1);
        }

        sleep(50);

        //Bump the button and back up.
        encoderDrive(0.2, 4, 4, 3);
        encoderDrive(0.45, -4, -4, 2);

        //Reset servo positions.
        leftServo.setPosition(1);
        rightServo.setPosition(0);

        //Turn toward at the cap ball.

        navx_device.close();

        encoderRotateWithGyro(-35);
        encoderRotateWithGyro(-35);

        //Drive to the cap ball to bump it.
        encoderDrive(0.5, -60, -60, 15);

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

    private void newLaunchBalls() throws InterruptedException {
//        elevatorMotor.setPower(1);
//        sleep(elevatorTime * 2);
//        elevatorMotor.setPower(0);
//        leftLauncherMotor.setPower(0);
//        rightLauncherMotor.setPower(0);

        double p = launchPower;

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

    public void gyroTurn (double speed, double angle, double err, double minSpeed)
            throws InterruptedException {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, 0.05, err, minSpeed)) {
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

    boolean onHeading(double speed, double angle, double PCoeff, double err, double minSpeed) {
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

            rightSpeed  = -speed * steer;
            if (rightSpeed > -minSpeed && rightSpeed < 0) rightSpeed = -minSpeed;
            else if (rightSpeed < minSpeed && rightSpeed > 0) rightSpeed = minSpeed;
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
        robotError = targetAngle - navx_device.getYaw();
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

    public void encoderDrive(double speed1, double speed2,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        int startLeft;
        int startRight;

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

            startLeft = leftMotor.getCurrentPosition();
            startRight = rightMotor.getCurrentPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed1));
            rightMotor.setPower(Math.abs(speed1));

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

                double averagePos = (leftMotor.getCurrentPosition() - startLeft + rightMotor.getCurrentPosition() - startRight) / 2.0;
                double speed = averagePos / (newLeftTarget - startLeft) * (speed2 - speed1) + speed1; // left target and right target are always the same.

                leftMotor.setPower(Math.abs(speed));
                rightMotor.setPower(Math.abs(speed));

                // Allow time for other processes to run.
                idle();
            }

            leftMotor.setPower(speed2);
            rightMotor.setPower(speed2);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderDriveLaunchAtEnd(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, int millisLaunch) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        double timeLeft = 999999999;
        int timeAtLastTest = 0;
        double distAtLastTest = 0;
        long startTime;
        int startLeft;
        int startRight;
        double encoderCountsPerMillis = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            startLeft = leftMotor.getCurrentPosition();
            startRight = rightMotor.getCurrentPosition();

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

            startTime = System.currentTimeMillis();

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                if ((int) (System.currentTimeMillis() - startTime - timeAtLastTest) > 25) {
                    double distCopy = distAtLastTest;
                    long timeCopy = timeAtLastTest;

                    timeAtLastTest = (int) (System.currentTimeMillis() - startTime);
                    distAtLastTest = ((leftMotor.getCurrentPosition() - startLeft)
                            + (rightMotor.getCurrentPosition() - startRight)) / 2;

                    int diffT = (int) (timeAtLastTest - timeCopy);
                    double diffD = distAtLastTest - distCopy;

                    encoderCountsPerMillis = diffD / diffT;

                    double distRemaining = (int) (leftInches * COUNTS_PER_INCH) - distAtLastTest;

                    timeLeft = (distRemaining / encoderCountsPerMillis);

                    if (timeLeft <= millisLaunch) {
                        double p = launchPower;

                        leftLauncherMotor.setPower(p);
                        rightLauncherMotor.setPower(p);
                    }
                }


                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.addData("Time Left", "%.2f", timeLeft);
                telemetry.addData("lastD, lastT", "%.2f, " + timeAtLastTest, distAtLastTest);
                telemetry.addData("Inches per Second", (encoderCountsPerMillis / COUNTS_PER_INCH * 1000));
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
                    (lineColorSensor.red() < threshold && lineColorSensor.blue() < threshold)) {

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

    public void encoderDriveToTapeNew(int threshold, double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        double timeLeft = 999999999;
        int timeAtLastTest = 0;
        double distAtLastTest = 0;
        long startTime;
        int startLeft;
        int startRight;
        double encoderCountsPerMillis = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            startLeft = leftMotor.getCurrentPosition();
            startRight = rightMotor.getCurrentPosition();

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

            startTime = System.currentTimeMillis();

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                if ((int) (System.currentTimeMillis() - startTime - timeAtLastTest) > 25) {
                    double distCopy = distAtLastTest;
                    long timeCopy = timeAtLastTest;

                    timeAtLastTest = (int) (System.currentTimeMillis() - startTime);
                    distAtLastTest = ((leftMotor.getCurrentPosition() - startLeft)
                            + (rightMotor.getCurrentPosition() - startRight)) / 2;

                    int diffT = (int) (timeAtLastTest - timeCopy);
                    double diffD = distAtLastTest - distCopy;

                    encoderCountsPerMillis = diffD / diffT;

                    double distRemaining = (int) (leftInches * COUNTS_PER_INCH) - distAtLastTest;

//                    D = V * T;
                }

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
                                   double timeoutS) throws InterruptedException {
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
//                telemetry.addData("gyro", gyro.getIntegratedZValue());
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
        double degDiff = degrees - navx_device.getYaw();
        encoderDriveCounts(0.22, (int) (encoderTicksPerDegree*degDiff), (int) (-encoderTicksPerDegree*degDiff), Math.abs(degDiff/45.0 ) + 0.5);
    }

    public void encoderRotateWithGyro(int degrees, double accuracy) throws InterruptedException {
        double degDiff = degrees - navx_device.getYaw();
        if (Math.abs(degDiff) > accuracy)
            encoderDriveCounts(0.22, (int) (encoderTicksPerDegree*degDiff), (int) (-encoderTicksPerDegree*degDiff), Math.abs(degDiff/45.0 ) + 0.5);
    }
}