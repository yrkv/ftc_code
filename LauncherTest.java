package org.firstinspires.ftc.robotcontroller.external.samples.ftc_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.io.PrintWriter;

@TeleOp
public class TroTeleOp extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor elevatorMotor;
    private DcMotor leftLauncherMotor;
    private DcMotor rightLauncherMotor;
    private VoltageSensor voltageSensor;
    
    private double power = 1; // number of volts it tries to send.
    private double changeRate = 0.05;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        elevatorMotor = hardwareMap.dcMotor.get("elevator");
        leftLauncherMotor = hardwareMap.dcMotor.get("left launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right launcher");
        voltageSensor = hardwareMap.VoltageSensor.get("voltage sensor");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("power", power);

        if (gamepad1.y) launch();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
    public void launch() {
    	double voltage = voltageSensor.getVoltage();
    	if (voltage > power * 2) {
    		leftLauncherMotor.setPower(power / voltage);
    		rightLauncherMotor.setPower(-power / voltage);
    		long t = System.currentTimeMillis();
    		while (System.currentTimeMillis() < t + 2500) {}
    		elevatorMotor.setPower(-1);
    		long t = System.currentTimeMillis();
    		while (System.currentTimeMillis() < t + 500) {}
    		elevatorMotor.setPower(0);
    		leftLauncherMotor.setPower(0);
    		rightLauncherMotor.setPower(0);
    	}
    	while (!(gamepad1.a || gamepad1.b || gamepad1.x)) {}
    	if (gamepad1.a) power -= changeRate;
    	if (gamepad1.x) power += changeRate;
    }

}
