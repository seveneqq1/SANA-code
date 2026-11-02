package org.firstinspires.ftc.teamcode.bobot.TestOpModes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;

import java.util.List;


@TeleOp( group = "!a")
public class DriveTrainDebug extends OpMode {
	private GamepadEx first_driver;
    IntakeController intakeController;
	private MecanumDrive mecanumDrive;
	private ElapsedTime runtime;
	private List<LynxModule> allHubs;
	private ElapsedTime timer;
	private VoltageSensor voltageSensor;
	Motor frontLeft,frontRight,backLeft,backRight;

	private static double multiplier;
	@Override
	public void init() {
		first_driver =new GamepadEx(gamepad1);
        frontLeft = new Motor(hardwareMap, "leftFront");
		frontRight = new Motor(hardwareMap, "rightFront");
		backLeft = new Motor(hardwareMap, "leftBack");
		 backRight = new Motor(hardwareMap, "rightBack");
		frontLeft.setInverted(true);
		backLeft.setInverted(true);
		frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        backLeft.setRunMode(Motor.RunMode.VelocityControl);
        backRight.setRunMode(Motor.RunMode.VelocityControl);
        intakeController=new IntakeController(hardwareMap);
		voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        mecanumDrive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);
		runtime=new ElapsedTime(ElapsedTime.Resolution.SECONDS);
		timer=new ElapsedTime();
		allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule hub : allHubs)
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

	}
	@Override
	public void init_loop(){

	}

	@Override
	public void start() {
		runtime.reset();
		timer.reset();
	}

	private double lastTimeStamp = 0;
	private double scaleInputLow(double x) {
		return x * (0.12 * x * x + 0.08);
	}

	private double scaleInputMedium(double x) {
		return x * (0.29 * x * x + 0.19);
	}

	private double scaleInputHigh(double x) {
		return x * (0.52 * x * x + 0.48);
	}
    boolean switchara =false;
	@Override
	public void loop() {
		for (LynxModule hub : allHubs)
			hub.clearBulkCache();

		double currentTimeStamp = (double) System.nanoTime() / 1E9;
		if (lastTimeStamp == 0)
			lastTimeStamp = currentTimeStamp;
		double dt = currentTimeStamp - lastTimeStamp;
		lastTimeStamp = currentTimeStamp;
		telemetry.addData("voltage",voltageSensor.getVoltage());
		first_driver.readButtons();
		 
		 
		double turnSpeed = first_driver.getRightX();
		double strafePower = first_driver.getLeftX();
		double forwardPower = first_driver.getLeftY();

        if(first_driver.isDown(GamepadKeys.Button.X)){
			backLeft.set(1);
		}
		if(first_driver.isDown(GamepadKeys.Button.B)){
			backRight.set(1);
		}
		if(first_driver.isDown(GamepadKeys.Button.Y)){
			frontLeft.set(1);
		}
		if(first_driver.isDown(GamepadKeys.Button.A)){
			frontRight.set(1);
		}
		else {
			frontRight.set(0);
			frontLeft.set(0);
			backRight.set(0);
			backLeft.set(0);
		}
		if (first_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0){
		multiplier=0.25;
		}
		else{
			multiplier=1;
		}

		double frequency = 1 / dt;
		telemetry.addData("frequency", frequency);
		telemetry.addData("Multiplier:",multiplier);
		telemetry.update();
	}
}
