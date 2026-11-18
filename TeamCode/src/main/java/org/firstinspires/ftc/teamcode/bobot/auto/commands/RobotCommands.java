package org.firstinspires.ftc.teamcode.bobot.auto.commands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.bobot.Robot;

@Config
public class RobotCommands {
    public static double readyTimeoutMs = 2000;
    public static double shooterTolerance = 120;
    public static double stopperOpenDelayMs = 150;

    private RobotCommands() {}

    public static Command intake(Robot robot) {
        return robot.intake.in();
    }

    public static Command closeFlower(Robot robot) {
        return new InstantCommand(robot.flower::closeFlower, robot.flower);
    }

    public static Command intakeFlower(Robot robot) {
        return new InstantCommand(robot.flower::openFlower, robot.flower);
    }

    public static Command spinUp(Robot robot) {
        return new InstantCommand(robot.shooter::turnOn, robot.shooter);
    }

    public static Command idle(Robot robot) {
        return new InstantCommand(() -> {
            robot.intake.spinOff();
            robot.shooter.turnOff();
        }, robot.intake, robot.shooter);
    }

    public static Command shoot(Robot robot, double feedMs) {
        return new CommandBase() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean stopperOpen;
            private boolean feeding;
            private boolean done;

            {
                addRequirements(robot.shooter, robot.intake);
            }

            @Override
            public void initialize() {
                stopperOpen = false;
                feeding = false;
                done = false;
                robot.intake.spinOff();
                robot.shooter.setFeeding(false);
                robot.shooter.turnOn();
                timer.reset();
            }

            @Override
            public void execute() {
                if (!stopperOpen) {
                    double target = robot.shooter.getTarget();
                    double velocity = Math.abs(robot.shooter.getVel());
                    if (timer.milliseconds() >= readyTimeoutMs) {
                        done = true;
                    } else if (target > 0 && Math.abs(target - velocity) < shooterTolerance) {
                        robot.shooter.setFeeding(true);
                        stopperOpen = true;
                        timer.reset();
                    }
                } else if (!feeding) {
                    if (timer.milliseconds() >= stopperOpenDelayMs) {
                        robot.intake.spinSlowShoot();
                        feeding = true;
                        timer.reset();
                    }
                } else {
                    done = timer.milliseconds() >= feedMs;
                }
            }

            @Override
            public boolean isFinished() {
                return done;
            }

            @Override
            public void end(boolean interrupted) {
                robot.intake.spinOff();
                robot.shooter.setFeeding(false);
            }
        };
    }
}
