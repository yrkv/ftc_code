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
public class TroRotation extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    static final double PI = 3.14159;
    static final double COUNTS_PER_INCH = 250 / PI;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private Servo leftServo;
    private Servo rightServo;
    private Servo capBallWinch;

    private int rotation = 0;

    private double turnPower = (0.225);

    private ColorSensor lineColorSensor;
    private ColorSensor beaconColorSensor;
    private ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor          = hardwareMap.dcMotor.get("left motor");
        rightMotor         = hardwareMap.dcMotor.get("right motor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

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

        lineColorSensor.enableLed(false);
        lineColorSensor.enableLed(true);
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

        encoderRotateThenGyro(135);
        sleep(2000);
        encoderRotateThenGyro(90);
        sleep(2000);
        encoderRotateThenGyro(45);
        sleep(2000);
        encoderRotateThenGyro(90);

        telemetry.addData("Path2",  "Running at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.update();
        sleep(10000);
        telemetry.addData("Path", "Complete");

        telemetry.update();
    }

    public void encoderRotateThenGyro(int degrees) throws InterruptedException {
        int left = leftMotor.getCurrentPosition();
        int right = rightMotor.getCurrentPosition();
        rotation += degrees;
        encoderDriveCounts(0.3, -left - (int) (40/3.0*degrees), -right + (int) (40/3.0*degrees), Math.abs(degrees/45.0));
        simpleGyro(0.045, -rotation, 1);
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

    public void encoderDriveCounts(double speed,
                             int leftCounts, int rightCounts,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = rightCounts;
            newRightTarget = leftCounts;
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
                    (runtime.seconds() < timeoutS)) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.addData("gyro", gyro.getIntegratedZValue());
                telemetry.update();

                double leftSpeed = (double) (leftMotor.getCurrentPosition() - newLeftTarget) / leftCounts * speed;
                double rightSpeed = (double) (rightMotor.getCurrentPosition() - newRightTarget) / rightCounts * speed;

                leftMotor.setPower(Math.abs(leftSpeed) + 0.1);
                rightMotor.setPower(Math.abs(rightSpeed) + 0.1);

                // Allow time for other processes to run.
                idle();
            }

            sleep(200);

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void simpleGyro(double power, int degrees, int timeoutS) {
        runtime.reset();

        while (runtime.seconds() < timeoutS & opModeIsActive()) {
            double currentRotation = gyro.getIntegratedZValue();

            if (currentRotation == degrees) break;
            else if (currentRotation < degrees) {
                leftMotor.setPower(-power);
                rightMotor.setPower(power);
            } else if (currentRotation > degrees) {
                leftMotor.setPower(power);
                rightMotor.setPower(-power);
            }

            telemetry.addData("Path2",  "Running at %7d :%7d",
                    leftMotor.getCurrentPosition(),
                    rightMotor.getCurrentPosition());
            telemetry.addData("gyro", gyro.getIntegratedZValue());
            telemetry.update();

            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
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
}
