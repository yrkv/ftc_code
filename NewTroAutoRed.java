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
public class NewTroAutoRed extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    static final double PI = 3.14159;
    static final double COUNTS_PER_INCH = 250 / PI;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor elevatorMotor;
    private DcMotor leftLauncherMotor;
    private DcMotor rightLauncherMotor;

    private Servo beaconServo;
    
    private double turnPower = 4.5;

    private ModernRoboticsI2cGyro gyro;
    private ColorSensor lineColorSensor;
    private ColorSensor beaconColorSensor;

    private int launcherCountsPerSecond = (int)(44.4 * 20);
    private double currentPower = 0.3;
    private int elevatorTime = 400;

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

        beaconServo = hardwareMap.servo.get("beacon servo");

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
        lineColorSensor.enableLed(true);
        beaconColorSensor.enableLed(false);

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (gyro.isCalibrating())  {
            Thread.sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        beaconServo.setPosition(0.8);


        leftLauncherMotor.setPower(currentPower);
        rightLauncherMotor.setPower(currentPower);
        sleep(500);

        elevatorMotor.setPower(1);
        sleep(elevatorTime);
        elevatorMotor.setPower(0);

        leftLauncherMotor.setPower(0);
        rightLauncherMotor.setPower(0);
        sleep(250);

        leftLauncherMotor.setPower(currentPower);
        rightLauncherMotor.setPower(currentPower);
        sleep(500);

        elevatorMotor.setPower(1);
        sleep(elevatorTime);
        elevatorMotor.setPower(0);


        /*encoderDrive(0.5, 12, 12, 5);

        p = getAdjustedPower(turnPower);
        gyroTurn(p, 45, 1);
        sleep(500);
        gyroTurn(p * 1.2, 45, 0);

        encoderDriveToTape(5, 0.5, 70, 70, 5);
        encoderDrive(1, -0.2, -0.2, 1);
        encoderDriveToTape(5, 0.2, -5, -5, 3);
        lineColorSensor.enableLed(false);


        p = getAdjustedPower(turnPower);
        gyroTurn(p, -89, 1);
        sleep(500);
        gyroTurn(p * 1.2, -89, 0);

        encoderDrive(0.4, -15, -15, 5);

        encoderDriveToBeacon(3, 0.1, -10, -10, 3);

        beaconServo.setPosition(beaconColorSensor.blue() > beaconColorSensor.red() ? 0 : 1); // adjust for actual servo positions

        encoderDrive(0.5, -2, -2, 1);
        sleep(100);
        encoderDrive(0.5, -4, -4, 1);
        encoderDrive(0.5, 5, 5, 1);*/

        /*gyroTurn(0.3, 180);

        encoderDrive(0.6, -25, -25, 5);

        double degrees = Math.atan((108-20-25)/(49)) / PI * 180;
        gyroTurn(0.3, 180 - Math.round(degrees));

        lineColorSensor.enableLed(true);

        encoderDriveToTape(5, 1, -80, -80, 5);
        encoderDriveToTape(5, 0.2, 4, 4, 3);


        gyroTurn(0.3, 90);

        encoderDrive(0.4, -14, -14, 5);

        encoderDriveToBeacon(5, 0.15, -3, -3, 3);

        beaconServo.setPosition(beaconColorSensor.blue() > beaconColorSensor.red() ? 1 : 0); // adjust for actual servo positions

        encoderDrive(0.5, -2, -2, 1);*/

//        while (!(beaconColorSensor.red() > 5 || beaconColorSensor.blue() > 5)) {
//            gyroDrive(0.06, 0.1, 270);
//        }
//
//        beaconServo.setPosition(beaconColorSensor.red() > beaconColorSensor.blue() ? 1 : 0);

//        gyroDrive(0.1, 2, 270);

        telemetry.addData("Path2",  "Running at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.update();
        telemetry.addData("Path", "Complete");
        //telemetry.addData("power", power);

        telemetry.update();
    }

    public void gyroTurn (  double speed, double angle, double err)
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

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= err) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);

            double multiplier = Math.abs(error) / 30 * 0.7 + 0.3;

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
}
