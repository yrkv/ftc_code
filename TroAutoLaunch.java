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
public class TroAutoLaunch extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    private DcMotor elevatorMotor;
    private DcMotor leftLauncherMotor;
    private DcMotor rightLauncherMotor;

    private double launcherCountsPerRev = 44.4;
    private double currentPower = 0.3;
    private int elevatorTime = 400;
    private int RPM = 1000;

    private double power = 3.7;

    @Override
    public void runOpMode() throws InterruptedException {
        elevatorMotor      = hardwareMap.dcMotor.get("elevator");
        leftLauncherMotor  = hardwareMap.dcMotor.get("left launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right launcher");

        leftLauncherMotor.setDirection(DcMotor.Direction.REVERSE);


        leftLauncherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double p = getAdjustedPower(power);

        leftLauncherMotor.setPower(p);
        rightLauncherMotor.setPower(p);
        sleep(200);

        p = launch(p);

        elevatorMotor.setPower(1);
        sleep(elevatorTime);
        elevatorMotor.setPower(0);

        sleep(200);

        p = launch(p);

        elevatorMotor.setPower(1);
        sleep(elevatorTime);
        elevatorMotor.setPower(0);
    }

    private double launch(double currentPower) {
        long time = System.currentTimeMillis();
        int[] encoderCount = new int[20]; // 200 millis to full
        encoderCount[0] = (leftLauncherMotor.getCurrentPosition() + rightLauncherMotor.getCurrentPosition()) / 2;
        int speedAccuracy = 10; // speed is RPM +/- this
        while (true) {
            while (System.currentTimeMillis() - time < 10) {
                sleep(1);
            }
            time += 10;
            for (int i = 19; i > 0; i--) {
                encoderCount[i] = encoderCount[i - 1];
            }
            encoderCount[0] = (leftLauncherMotor.getCurrentPosition() + rightLauncherMotor.getCurrentPosition()) / 2;

            leftLauncherMotor.setPower(currentPower);
            rightLauncherMotor.setPower(currentPower);

            int diff = encoderCount[19] - encoderCount[0];

            if (encoderCount[19] != 0)
                if (Math.abs(diff * 5 * 60 - RPM) <= speedAccuracy)
                    break;
                else if (diff * 5 * 60 < RPM)
                    currentPower += (RPM - diff * 5 * 60) / 10000.0;
                else if (diff * 5 * 60 > RPM)
                    currentPower -= (RPM - diff * 5 * 60) / 10000.0;
        }
        return  currentPower;
    }

    private double getAdjustedPower(double p) {
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        double drop = turnPower / 13.8;
        return power / (voltage - drop);
    }
}
