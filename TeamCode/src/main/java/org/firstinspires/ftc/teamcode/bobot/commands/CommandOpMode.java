package org.firstinspires.ftc.teamcode.bobot.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import dev.nextftc.robot.RobotState;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class CommandOpMode extends NextOpMode {
    protected Telemetry robotTelemetry = telemetry;
    private boolean commandsStarted;

    protected CommandOpMode() {
        super(RobotState.INSTANCE.getRobot(), new Lifecycle());
    }

    protected void onInit() {}

    protected void onStart() {}

    protected void onStop() {}

    protected Command createCommand() {
        return Command.NOOP;
    }

    protected Command[] createCommands() {
        return new Command[] {createCommand()};
    }

    protected final void enablePanelsTelemetry() {
        robotTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    @Override
    public final void periodic() {
        if (!commandsStarted) {
            commandsStarted = true;
            onStart();
            Scheduler.schedule(createCommands());
        }
        beforeCommands();
    }

    protected void beforeCommands() {}

    private static final class Lifecycle implements OpModeHook {
        private CommandOpMode opMode;

        @Override
        public void afterConstruction(NextOpMode nextOpMode) {
            opMode = (CommandOpMode) nextOpMode;
            opMode.onInit();
        }

        @Override
        public void afterEnd() {
            if (opMode != null) {
                opMode.onStop();
            }
        }
    }
}
