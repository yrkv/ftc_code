package org.firstinspires.ftc.robotcontroller.external.samples.ftc_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
public class TroAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    static final double PI = 3.14159;
    static final double     COUNTS_PER_INCH = 360 / PI;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor elevatorMotor;
    private DcMotor collectorMotor;
    private DcMotor leftLauncherMotor;
    private DcMotor rightLauncherMotor;
    private int reverse = 1; // 1 when normal, -1 when reversed.

    private ColorSensor colorSensor;


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
        collectorMotor = hardwareMap.dcMotor.get("collector");
        leftLauncherMotor = hardwareMap.dcMotor.get("left launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right launcher");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

//        colorSensor = hardwareMap.colorSensor.get("color");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(0.1, 1, 1, 5);
        leftLauncherMotor.setPower(0.37);
        rightLauncherMotor.setPower(-0.37);
        sleep(2700);
        elevatorMotor.setPower(-0.5);
        sleep(400);
        elevatorMotor.setPower(0);
        leftLauncherMotor.setPower(0);
        rightLauncherMotor.setPower(0);
        sleep(10000);

        leftLauncherMotor.setPower(0.37);
        rightLauncherMotor.setPower(-0.37);
        sleep(2700);
        elevatorMotor.setPower(-0.5);
        sleep(1000);
        elevatorMotor.setPower(0);
        leftLauncherMotor.setPower(0);
        leftLauncherMotor.setPower(0);

       // encoderDrive(1,45,45,10);
        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        encoderRotate(0.5, 90, 10);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public boolean redBlue() // true if red, false if blue
    {
        if (colorSensor.red() > colorSensor.blue()) return true;
        return false;
    }
    public void LineFollower()
    {
    }

    public void moveToPosistion(DcMotor motor, int encoderCounts, double power) throws InterruptedException {
        int currPos = motor.getCurrentPosition();
        power = power > 1 ? 1 : power < -1 ? -1 : power;
        if (power > 0) {
            motor.setPower(power);
            while (motor.getCurrentPosition() < currPos + encoderCounts) {
                idle();
            }
            motor.setPower(-power);
            while (motor.getCurrentPosition() > currPos + encoderCounts) {
                idle();
            }
            motor.setPower(0);
        } else {
            motor.setPower(power);
            while (motor.getCurrentPosition() > currPos - encoderCounts) {
                idle();
            }
            motor.setPower(-power);
            while (motor.getCurrentPosition() < currPos - encoderCounts) {
                idle();
            }
            motor.setPower(0);
        }

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
            /*while (opModeIsActive() &&
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
            }*/

            while(leftMotor.isBusy() && rightMotor.isBusy()){
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderRotate(double speed, int degrees, double timeoutS) throws InterruptedException {
        encoderDrive(speed, degrees / 22.5 * PI, -degrees / 22.5 * PI, timeoutS);
    }
}
