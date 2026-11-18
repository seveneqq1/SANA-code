package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
public class Drive extends SubsystemBase {
    private Follower f;
    private MecanumDrive mecanum;
    private GoBildaPinpointDriver pinpoint;

    public void init(HardwareMap hw, Pose start) {
        init(hw);
        setPose(start);
    }

    public void init(HardwareMap hw) {
        f = Constants.create(hw);
        if (f != null) return;

        Motor fl = new Motor(hw, "frontleft");
        Motor fr = new Motor(hw, "frontright");
        Motor bl = new Motor(hw, "backleft");
        Motor br = new Motor(hw, "backright");
        fr.setInverted(true);
        br.setInverted(true);
        for (Motor motor : new Motor[]{fl, fr, bl, br}) {
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
        mecanum = new MecanumDrive(false, fl, fr, bl, br);

        pinpoint = hw.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-154.5, -165.5, DistanceUnit.MM);
        pinpoint.setEncoderResolution(19.894, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void resetPos() {
        setPose(new Pose(135, 9, Math.PI / 2));
    }

    private void setPose(Pose pose) {
        if (f != null) {
            f.setPose(pose);
        } else {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.x(), pose.y(),
                    AngleUnit.RADIANS, pose.heading()));
            pinpoint.update();
        }
    }

    public void drive(double forward, double strafe, double turn) {
        if (f != null) f.manual(forward, strafe, turn);
        else mecanum.driveRobotCentric(strafe, forward, turn);
    }

    public Pose getPose(){
        if (f != null) return f.pose();
        return new Pose(pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public void periodic() {
        if (f != null) f.update();
        else pinpoint.update();
    }
}
