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
public class TroAuto extends LinearOpMode {

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

    private ModernRoboticsI2cGyro gyro;
    private ColorSensor lineColorSensor;
    private ColorSensor beaconColorSensor;

    private double power = 3.00; // 4.34
    private int accelerationTime = 4000;
    private int elevatorTime = 400;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        elevatorMotor = hardwareMap.dcMotor.get("elevator");
       // collectorMotor = hardwareMap.dcMotor.get("collector");
        leftLauncherMotor = hardwareMap.dcMotor.get("left launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right launcher");
        beaconServo = hardwareMap.servo.get("beacon servo");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        lineColorSensor = hardwareMap.colorSensor.get("line color sensor");
        beaconColorSensor = hardwareMap.colorSensor.get("beacon color sensor");

        lineColorSensor.setI2cAddress(I2cAddr.create8bit(0x3a));
        beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
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

        /*double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        double drop = power / 13.8;
        leftLauncherMotor.setPower(power / (voltage - drop));rightLauncherMotor.setPower(-power / (voltage - drop));
        sleep(accelerationTime);
        elevatorMotor.setPower(-1);
        sleep(elevatorTime);
        elevatorMotor.setPower(0);leftLauncherMotor.setPower(0);rightLauncherMotor.setPower(0);
        sleep(8000);

        voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        drop = power / 13.8;
        leftLauncherMotor.setPower(power / (voltage - drop));rightLauncherMotor.setPower(-power / (voltage - drop));
        sleep(accelerationTime);
        elevatorMotor.setPower(-1);
        sleep(elevatorTime + 500);
        elevatorMotor.setPower(0);leftLauncherMotor.setPower(0);rightLauncherMotor.setPower(0);
        sleep(500);*/


        gyroDrive(0.4, 20, 0);

        gyroTurn(0.3, 180);

        gyroDrive(0.4, -25, 180);

        double degrees = Math.atan((108-20-25)/(49)) / PI * 180;
        gyroTurn(0.3, 180 - Math.round(degrees));
//
//        lineColorSensor.enableLed(true);
//        int c = 0;
//        int t = 3;
//        while (!(lineColorSensor.red() > t && lineColorSensor.green() > t && lineColorSensor.blue() > t) || c > 200) {
//            gyroDrive(0.2, 0.1, 180 + degrees);
//            c++;
//        }
//
//        gyroTurn(0.3, 270);
//
//        gyroDrive(0.2, 14, 270);
//
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
        sleep(10000);
        telemetry.addData("Path", "Complete");
        //telemetry.addData("power", power);

        telemetry.update();
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, 0.15);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftMotor.setPower(leftSpeed);
                rightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
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
        }
    }

    public void gyroTurn (  double speed, double angle)
            throws InterruptedException {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, 0.1)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            idle();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) < 0.1) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer * (error / angle * 0.50 + 0.50);
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
