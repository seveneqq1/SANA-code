# Repository Guidelines

## Project Structure & Module Organization

This FTC Android project has two Gradle modules:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`: team code. `bobot/` contains autonomous routines, TeleOp OpModes, and hardware controllers; `pedro/` contains pathing configuration and tuning procedures.
- `FtcRobotController/`: SDK controller application, sample OpModes, and Android resources. Prefer implementing team behavior in `TeamCode`.

Android resources live under each module's `src/main/res/`. Read `docs/sloth-setup.md` for deployment and `docs/teleop-state-machines.md` for controller integration. `doc/` holds upstream media/legal assets; `libs/` holds bundled artifacts.

## Build, Test, and Development Commands

Use Android Studio with a compatible Gradle JDK and Android SDK. Keep the SDK path in untracked `local.properties`. Run from the repository root:

- `rtk proxy ./gradlew :TeamCode:assembleDebug`: compile the debug APK.
- `rtk proxy ./gradlew :TeamCode:lintDebug`: run Android lint.
- `rtk proxy ./gradlew :TeamCode:installDebug`: install on a connected robot controller through USB/ADB.
- `rtk proxy ./gradlew :TeamCode:deploySloth`: hot reload team Java changes; updates apply after the active OpMode ends.

Use a full install after dependency, Gradle, out-of-package, or `@Pinned` changes. Prefer module-specific build changes in `TeamCode/build.gradle` over shared SDK build files.

## Coding Style & Naming Conventions

Use Java 8-compatible syntax, four-space indentation, and the existing package layout. Prefer `PascalCase` for new classes and `camelCase` for methods/fields; preserve legacy class names unless renaming is required. Keep hardware names consistent with robot configuration. No dedicated formatter is configured; match surrounding code and use Android Studio formatting.

## Testing Guidelines

No unit-test framework or coverage threshold is configured. Existing `test.java` and Pedro `procedures/Tests.java` are robot-side code, not a host test suite. For new automated tests, use `src/test/java/` and `*Test.java`, adding the required dependencies. Validate robot behavior on hardware and report what was actually checked. Follow documented pose units, turret limits, and `cancelAll()` stop handling.

## Commit & Pull Request Guidelines

History uses short, action-oriented subjects such as “Bump AutoTune version” and “Fix three wheel tuners ignoring custom drive motor names”; no strict Conventional Commits scheme is evident. Keep commits focused. PRs should describe behavior changes, link relevant issues, and list build and hardware checks. Include telemetry, logs, or video when useful. Target the team repository for team-specific changes.

## Agent Instructions

Follow @/Users/akhmetshadmat/.codex/RTK.md: prefix shell commands with `rtk`. Preserve unrelated working-tree changes and exclude generated builds, local configuration, and private signing credentials.
