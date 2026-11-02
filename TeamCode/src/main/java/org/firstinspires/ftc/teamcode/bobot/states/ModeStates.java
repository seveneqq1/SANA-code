package org.firstinspires.ftc.teamcode.bobot.states;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ModeStates {

    public static boolean isRedAlliance = true;

    public enum GameMode {COMP_MODE, TRAINING_MODE}
    public static GameMode selectedGameMode = GameMode.TRAINING_MODE;
    public enum Colors { RED, BLUE }
}
