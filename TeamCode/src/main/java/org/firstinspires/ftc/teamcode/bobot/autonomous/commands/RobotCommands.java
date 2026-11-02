package org.firstinspires.ftc.teamcode.bobot.autonomous.commands;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateController;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
import org.firstinspires.ftc.teamcode.bobot.controllers.FlowerTakerController;

public final class RobotCommands {
    private RobotCommands() { }

    public static Command setMode(StateController stateController, StateController.Mode mode) { return instant(() -> stateController.setMode(mode)); }

    public static Command intake(StateController stateController) { return setMode(stateController, StateController.Mode.INTAKING); }

    public static Command spinUp(StateController stateController) { return setMode(stateController, StateController.Mode.SPINNING_UP); }

    public static Command idle(StateController stateController) { return setMode(stateController, StateController.Mode.IDLE); }

    public static Command openFlowerTaker(FlowerTakerController flowerTaker) { return instant(flowerTaker::open); }

    public static Command closeFlowerTaker(FlowerTakerController flowerTaker) { return instant(flowerTaker::close); }

    public static Command waitUntilReady(StateController stateController) {
        return spinUp(stateController)
                .then(waitUntil(stateController::isReadyToShoot))
                .raceWith(waitMs(RobotConfig.AUTO_READY_TIMEOUT_MS));
    }

    public static Command shoot(StateController stateController, double feedMs) {
        FeedTimer fed = new FeedTimer();
        return Command.build()
                .setStart(() -> {
                    fed.reset();
                    stateController.setMode(StateController.Mode.SHOOTING);
                })
                .setExecute(() -> fed.update(stateController.isFeeding()))
                .setDone(() -> fed.ms >= feedMs)
                .raceWith(waitMs(RobotConfig.AUTO_READY_TIMEOUT_MS + feedMs))
                .then(instant(stateController::stop));
    }

    private static final class FeedTimer {
        private static final double MAX_CREDIT_PER_LOOP_MS = 100;
        double ms;
        private long last;

        void reset() {
            ms = 0;
            last = System.nanoTime();
        }

        void update(boolean feeding) {
            long now = System.nanoTime();
            if (feeding) ms += Math.min((now - last) / 1e6, MAX_CREDIT_PER_LOOP_MS);
            last = now;
        }
    }
}
