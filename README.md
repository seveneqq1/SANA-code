# SANA Temporary — код сезона 2026–27 (Regional A)

Это очищенная копия ветки `Temporary` репозитория [seveneqq1/SANA-code](https://github.com/seveneqq1/SANA-code).
Здесь лежит **код этого сезона**, а не Worlds: FTC SDK 12, Pedro Pathing 3, SolversLib 0.3.6, Sloth (hot reload) и FTC Dashboard (`@Config`), не Panels.

Официальный readme FTC SDK: `doc/FTC-Robot-Controller-README.md`.
Как ставить Sloth: `docs/sloth-setup.md`. Как пользоваться state machines: `docs/teleop-state-machines.md`.

Код запускается **только через Android Studio**. Не копируй эти файлы в OnBot Java.

## Как открыть и залить

1. File → Open и выбери папку `SANA-temp-clean` (здесь лежат `settings.gradle` и `TeamCode`).
2. Дождись Gradle Sync (нужен Android SDK; свой путь SDK Android Studio запишет в `local.properties`).
3. Включи Control Hub от батареи и подключи USB-C (data-кабель) или Wi‑Fi робота.
4. Первый раз: полный install `./gradlew :TeamCode:installDebug` (или ▶ Run конфигурации TeamCode).
5. Потом для правок Java можно hot reload: `./gradlew :TeamCode:deploySloth` (применяется после конца текущего OpMode).
6. На Driver Hub выбери **Regional A - Snap**.

## Имена в config на Driver Hub

Имена из **этого** сезона (они другие, чем на Worlds):

**Моторы**

| Имя в config | Что это |
|---|---|
| `frontleft` | передний левый drive |
| `frontright` | передний правый drive |
| `backleft` | задний левый drive |
| `backright` | задний правый drive |
| `shooter1` | первый мотор shooter, goBILDA 6000rpm 1:1 |
| `shooter2` | второй мотор shooter, крутится в другую сторону |
| `intake` | мотор intake |
| `trans` | мотор transfer (подача) |

**Servo**

| Имя в config | Что это |
|---|---|
| `stopper` | blocker / feeder перед shooter |
| `servo1` | первый servo turret |
| `servo2` | второй servo turret |

**Датчики**

| Имя в config | Тип |
|---|---|
| `pinpoint` | goBILDA Pinpoint |

Hood и датчиков мяча в этом коде нет.

## Что делает каждый файл

**Главный матчевый код**

- `bobot/teleop/RegionalATeleOp.java` — матчевый TeleOp `Regional A - Snap`.
- `bobot/controller/shooterController_var2.java` — shooter для матчевых OpMode: постоянная мощность `flywheelPower`, `shooter2` всегда в обратную сторону, `isAtTarget()` сразу `true`.
- `bobot/controller/shooterController.java` — запасной контроллер с замером скорости и опциональным PD + kV (`USE_VELOCITY_CONTROL`). Его использует ранний `blue` TeleOp.
- `bobot/controller/intakeController.java` — `intake` + `trans`.
- `bobot/controller/turretController.java` — turret на `servo1` и `servo2`, центр 180°.
- `bobot/controller/HeadingController.java` — удержание heading (сам датчики не читает).
- `bobot/controller/ShootingStateMachine.java` — общая последовательность intake / spin-up / feed.
- `bobot/controller/SnapStateMachine.java` — поворот turret на 90° кнопками (то, что использует матчевый TeleOp).
- `bobot/controller/TrackingStateMachine.java` — автоприцел turret по одометрии (готово, но в матчевом TeleOp пока не включено).
- `bobot/controller/ShotTable.java` — таблица дистанция → скорость shooter.

**Pedro Pathing 3**

- `pedro/Constants.java` — CAD-замеры расстояния между модулями Pinpoint. `create()` пока возвращает `null` (pathing ещё не собран).
- `pedro/Tuning.java` — заготовка.
- `pedro/procedures/` — официальные тюнеры Pedro 3 (Pinpoint, mecanum, foresight и т.д.). Их не удаляли.

**Заготовки и debug**

- `bobot/teleop/blue.java` — ранний набросок Blue TeleOp (теперь хотя бы едет). Для матчей лучше Snap.
- `bobot/teleop/red.java` — пустая заготовка Red.
- `bobot/autonomous/blue.java` — пустая заготовка Auto.
- `bobot/teleop/teleop.java` — старый тест (`test dont use`).
- `bobot/teleop/servo*.java` — ручные тесты stopper / turret / shooter.

## Кнопки в Regional A - Snap

**gamepad1 (водитель)**

- стики — ехать; правый стик также снимает удержание heading
- **R2** — intake
- **L2** — выплюнуть
- удерживать **△ Y** — стрелять (открыть stopper, когда shooter готов)
- **✕ A** — стоп
- **L1** — медленная езда
- **□ X / ○ B** — повернуть heading на 90°, если hold включён

**gamepad2 (необязательный оператор)**

- D-pad вверх — зафиксировать текущий heading
- L1 / R1 — turret на 90°
- **□ X** — вернуть turret в центр

Числа (`flywheelPower`, `baseVelocity`, позиции stopper, heading gains) крутятся в FTC Dashboard (`@Config`).

## Что обязательно проверить / замерить

1. **Pinpoint offsets.** В `Constants` записаны unsigned расстояния между модулями из Fusion (14.2 mm и 335.8 mm). Это **не** те signed offsets, которые нужны Pedro (`+X` вперёд, `+Y` влево, от центра вращения). Прогони `PinpointTuner` и подставь результат в `create()`.
2. **`create()` всё ещё `null`.** Автономка и любой код, который вызывает `Constants.create()` без проверки, не поедет по pathing. `RegionalATeleOp` pathing не использует, ему достаточно Pinpoint heading.
3. **Направление shooter в матче.** `RegionalATeleOp` и `teleop` используют `shooterController_var2`: `shooter2` всегда получает `-power`. Флаг `REVERSE_SHOOTER2` есть только у запасного `shooterController`.
4. **`flywheelPower` (0.67)** на `shooterController_var2`. `baseVelocity` (2400) использует state machine. У запасного `shooterController` `USE_VELOCITY_CONTROL` выключен.
5. Позиции `stopper`: закрыто `0.05`, открыто `0.5`.
6. Направление drive-моторов: в Snap инвертированы `frontright` и `backright`. Если робот едет не туда — поправь инверсии.
7. Turret `kV`, `center_pos`, лимиты — на новом механизме их надо проверить.
8. Heading gains (`HEADING_KP` / `HEADING_KD`) в RegionalATeleOp помечены как стартовые, не откалиброванные.

## Что починили в этой копии

- Матчевые OpMode снова на `shooterController_var2`, как до очистки. Запасной `shooterController` (замер скорости и опциональный PD + kV) оставлен; его держит `blue` TeleOp.
- Удалены сломанные файлы: `beka_.java` (NPE: `Constants.create()` = null и `startPose` не задан) и пустой `test.java`.
- `red.java` и `autonomous/blue.java` теперь настоящие OpMode, а не классы с аннотацией без `extends OpMode`.
- В `blue` TeleOp джойстик наконец идёт на drive; исправлен heading `Math.toRadians(Math.PI)` (это было ~3°, а не 180°).
- Turret больше не уезжает в позицию `0` на первом `update()`.
- `ShotTable` проверяет, что дистанции строго возрастают.
- В Gradle пояснено, почему вместе стоят `com.pedropathing:core:3.0.1` и `org.solverslib:pedroPathing:0.3.6` (это адаптер, не второй Pedro). Dashboard — Slothboard, без bylazar Panels.

Никуда не пушили. Собирали без Android SDK, полную Gradle-сборку здесь не запускали.
