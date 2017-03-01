package org.firstinspires.ftc.robotcontroller.external.samples.ftc_code;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.ftc_code.navx.AHRS;
import org.firstinspires.ftc.robotcontroller.external.samples.ftc_code.navx.navXPIDController;

/*
 * An example linear op mode where the robot will rotate
 * to a specified angle an then stop.
 *
 * This example uses a simple PID controller configuration
 * with a P coefficient, and will likely need tuning in order
 * to achieve optimal performance.
 *
 * Note that for the best accuracy, a reasonably high update rate
 * for the navX-Model sensor should be used.
 */
@TeleOp(name = "Concept: navX Rotate to Angle PID - Linear", group = "Concept")
// @Disabled Comment this in to remove this from the Driver Station OpMode List
public class NavXRotateToAngle extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    /* This is the port on the Core Device Interface Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;


    private boolean calibration_complete = false;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        /* If possible, use encoders when driving, as it results in more */
        /* predictable drive system response.                           */

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* Create a PID Controller which uses the Yaw Angle as input. */
//        yawPIDController = new navXPIDController( navx_device,
//                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
//        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
//        yawPIDController.setContinuous(true);
//        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
//        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
//        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        idle();


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
        waitForStart();


/*
        try {
            yawPIDController.enable(true);

        *//* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         *//*

            final double TOTAL_RUN_TIME_SECONDS = 30.0;
            int DEVICE_TIMEOUT_MS = 500;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            DecimalFormat df = new DecimalFormat("#.##");

            while ( (runtime.time() < TOTAL_RUN_TIME_SECONDS) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        telemetry.addData("PIDOutput", df.format(0.00));
                    } else {
                        double output = yawPIDResult.getOutput();
                        leftMotor.setPower(output);
                        rightMotor.setPower(-output);
                        telemetry.addData("PIDOutput", df.format(output) + ", " +
                                df.format(-output));
                    }
                } else {
			    *//* A timeout occurred *//*
                    Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
            }
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        finally {
            navx_device.close();
            telemetry.addData("LinearOp", "Complete");
        }
        */


        gyroTurn(0.25, 90, 0.15, 0.025);

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


}
