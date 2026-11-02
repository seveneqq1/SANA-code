package org.firstinspires.ftc.teamcode.bobot.TestOpModes;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController;

import java.util.List;


@TeleOp( group = "!a")
public class JUST_BASE_6kRPM extends OpMode {
	private GamepadEx first_driver;
    IntakeController intakeController;
	private MecanumDrive mecanumDrive;
	GoBildaPinpointDriver goBildaPinpointDriver;
	private ElapsedTime runtime;
	private List<LynxModule> allHubs;
	private ElapsedTime timer;
	private VoltageSensor voltageSensor;

	private static double multiplier;
	ShooterController shooterController;
	@Override
	public void init() {
		goBildaPinpointDriver=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
		goBildaPinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		shooterController=new ShooterController(hardwareMap);
		first_driver =new GamepadEx(gamepad1);
        Motor frontLeft = new Motor(hardwareMap, "leftFront");
		Motor frontRight = new Motor(hardwareMap, "rightFront");
		Motor backLeft = new Motor(hardwareMap, "leftBack");
		Motor backRight = new Motor(hardwareMap, "rightBack");
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
		goBildaPinpointDriver.resetPosAndIMU();
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
	public static double forward_pod_y_offset=-5.6127;
	public static double strafe_pod_x_offset=-4.15685555;
	@Override
	public void loop() {
		for (LynxModule hub : allHubs)
			hub.clearBulkCache();
		shooterController.update();
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

        if(first_driver.wasJustPressed(GamepadKeys.Button.X)){
            intakeController.setState(IntakeController.State.IN_TAKING_FULL_POWER);
        }
        if(first_driver.wasJustPressed(GamepadKeys.Button.Y)){
            intakeController.setState(IntakeController.State.IN_TAKING_SC_TH_BALL);
        }
        if(first_driver.wasJustPressed(GamepadKeys.Button.A)){
            intakeController.setState(IntakeController.State.ZERO);
        }
		if (first_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0){
		multiplier=0.25;
		}
		else{
			multiplier=1;
		}
		goBildaPinpointDriver.setOffsets(strafe_pod_x_offset,forward_pod_y_offset, DistanceUnit.INCH);
		goBildaPinpointDriver.update();
		telemetry.addData("x",goBildaPinpointDriver.getPosX(DistanceUnit.INCH));
		telemetry.addData("y",goBildaPinpointDriver.getPosY(DistanceUnit.INCH));
        intakeController.update();
		mecanumDrive.driveFieldCentric(strafePower*multiplier,forwardPower*multiplier,turnSpeed*multiplier,0);
		double frequency = 1 / dt;
		telemetry.addData("frequency", frequency);
		telemetry.addData("Multiplier:",multiplier);
		telemetry.update();
	}
}
