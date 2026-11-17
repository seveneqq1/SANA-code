# Sloth with Pedro Pathing 3

This project uses Pedro Pathing core/revhub 3.0.1, Ivy 1.1.1, NextFTC 0.3.0,
SolversLib core 0.3.6, Panels 1.0.20, and Sloth/Load 0.3.2. NextFTC runs Ivy
commands, including Pedro Pathing commands. SolversLib supplies hardware,
drive, and gamepad utilities. The official Panels bundle provides telemetry
and live configuration, including Sloth-aware configurable scanning.

Official references:

- [SolversLib installation](https://docs.seattlesolvers.com/installation)
- [NextFTC installation](https://beta.nextftc.dev/introduction/installation/)
- [Pedro Pathing Ivy installation](https://pedropathing.com/docs/ivy)
- [Panels prerequisites](https://panels.bylazar.com/docs/com.bylazar.docs/Prerequisites/)
- [Sloth installation and Android Studio task setup](https://github.com/Dairy-Foundation/Sloth#installation)
- [Pedro Pathing installation](https://pedropathing.com/docs/pathing/installation)

## First install

Sync Gradle in Android Studio. Connect the robot over USB or ADB Wi-Fi, then
perform a full TeamCode install:

```sh
./gradlew :TeamCode:installDebug
```

## Hot reload

After editing Java classes under `org.firstinspires.ftc.teamcode`, run:

```sh
./gradlew :TeamCode:deploySloth
```

Sloth applies the update after the active OpMode ends. In Android Studio,
create a Gradle run configuration for `:TeamCode:deploySloth`.

For the normal TeamCode Android run configuration, add
`:TeamCode:removeSlothRemote` as the first **Before launch** Gradle task, as
shown in the official Sloth guide. Load 0.3.2 also wires remote removal into
Gradle's `installDebug` and `installRelease` tasks.

Use a full install after changing dependencies, Gradle configuration, files
outside the hot-reloaded package, or `@Pinned` annotations. A hot reload cannot
install new libraries. Robot deployment must be checked on the connected robot.
