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

import org.firstinspires.ftc.robotcontroller.external.samples.ftc_code.navx.AHRS;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
public class EncoderCalibrate extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    static final double PI = 3.14159;
    static final double COUNTS_PER_INCH = 250 / PI;

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final int NAVX_DIM_I2C_PORT = 0;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor elevatorMotor;
    private DcMotor leftLauncherMotor;
    private DcMotor rightLauncherMotor;

    private Servo leftServo;
    private Servo rightServo;
    private Servo capBallWinch;

    private double highCounts = 15;
    private double lowCounts = 13;

    private AHRS navx_device;
    private ColorSensor lineColorSensor;
    private ColorSensor beaconColorSensor;

    private int launcherCountsPerSecond = (int)(44.4 * 20);
    private double currentPower = 0.3;

    private int elevatorTime = 400;

    private double launchPower = (0.33);

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor          = hardwareMap.dcMotor.get("left motor");
        rightMotor         = hardwareMap.dcMotor.get("right motor");
        elevatorMotor      = hardwareMap.dcMotor.get("elevator");
        leftLauncherMotor  = hardwareMap.dcMotor.get("left launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right launcher");

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
        capBallWinch = hardwareMap.servo.get("cap ball winch");
        leftServo.scaleRange(0, 1);
        rightServo.scaleRange(0, 1);

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

        long startCalibration = System.currentTimeMillis();
        while (navx_device.isCalibrating()) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            telemetry.addData("Calibration Time", System.currentTimeMillis() - startCalibration);
            telemetry.update();
        }
        navx_device.zeroYaw();
        telemetry.addData("navX-Micro", "Calibrated");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        int currentAngle = 90;
        int c = 0;

        while (opModeIsActive() && c < 20) {
            double mid = (lowCounts + highCounts) / 2;
            encoderRotateWithGyro(currentAngle);
            sleep(100);
            telemetry.addData("gyro", "%.2f", navx_device.getYaw());
            telemetry.update();
            if (navx_device.getYaw() < currentAngle) lowCounts = mid;
            if (navx_device.getYaw() > currentAngle) highCounts = mid;
            if (navx_device.getYaw() == currentAngle) {
                double diff = highCounts - lowCounts;
                lowCounts = mid - diff/2;
                highCounts = mid + diff/2;
            }

            sleep(800);

            navx_device.zeroYaw();
            sleep(1000);
            c++;
        }


        telemetry.addData("mid", (lowCounts+highCounts)/2);

        telemetry.update();
        sleep(60000);
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

        robotError = targetAngle - navx_device.getYaw();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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
                telemetry.addData("gyro", "%.2f", navx_device.getYaw());
                telemetry.addData("mid", (lowCounts+highCounts)/2);
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

    /*public void simpleGyro(double power, int degrees, int timeoutS) {
        runtime.reset();

        while (runtime.seconds() < timeoutS & opModeIsActive()) {
            int currentRotation = gyro.getIntegratedZValue();
            int diff = Math.abs(degrees - currentRotation);
            double turnPower = (diff > 30) ? power : diff / 30 + 0.04;

            if (currentRotation == degrees) break;
            else {
                leftMotor.setPower(-turnPower);
                rightMotor.setPower(turnPower);
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
    }*/

    public void encoderRotateWithGyro(int degrees) throws InterruptedException {
        sleep(50); // optional sleep for increased accuracy
        double degDiff = degrees - navx_device.getYaw();
        double mult = (lowCounts + highCounts) / 2;
        encoderDriveCounts(0.22, (int) (mult*degDiff), (int) (-mult*degDiff), Math.abs(degDiff/45.0) + 2.5);
    }
}