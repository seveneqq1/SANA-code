# SANA Temporary — код сезона 2026–27 (Regional A)

Это очищенная копия ветки `Temporary` репозитория [seveneqq1/SANA-code](https://github.com/seveneqq1/SANA-code).
Здесь лежит **код этого сезона**, а не Worlds: FTC SDK 12, Pedro Pathing 3, NextFTC 2 с планировщиком Ivy, SolversLib, Panels и Sloth (hot reload).

Официальный readme FTC SDK: `doc/FTC-Robot-Controller-README.md`.
Как ставить Sloth: `docs/sloth-setup.md`. Как пользоваться state machines: `docs/teleop-state-machines.md`.

Код запускается **только через Android Studio**. Не копируй эти файлы в OnBot Java.

## Как открыть и залить

1. File → Open и выбери папку `SANA-code-Temporary` (здесь лежат `settings.gradle` и `TeamCode`).
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

## Структура TeamCode

- `bobot/compOpModes/` — режимы Regional A Snap, Tracking и Point Track, а также Blue и Red TeleOp.
- `bobot/autonomous/` — Blue Auto, заготовка с телеметрией без движения.
- `bobot/TestOpModes/` — ручные тесты и калибровка одометрии.
- `bobot/controllers/` — приводы механизмов и управление скоростью shooter.
- `bobot/states/` — логика прицеливания, последовательность впуска и подачи.
- `bobot/commands/` — жизненный цикл OpMode на NextFTC и запуск команд Ivy.
- `pedroPathing/Constants.java` — раздельные настройки OpMode, позиции, пороги и калибровка Pinpoint.
- `pedroPathing/PedroDrive.java` — команды Pedro Pathing 3 для будущих маршрутов с измеренными параметрами Foresight.

NextFTC запускает Ivy один раз за цикл. SolversLib обслуживает моторы, микшер
Mecanum и gamepad. Существующие TeleOp используют свои режимы Pinpoint и
инверсию моторов. Команды маршрута Pedro используют отдельного владельца
привода и рассчитаны на отдельный OpMode с калибровкой конкретного робота.

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

Параметры heading и позы каждого режима настраиваются отдельно в Panels
в разделах `Snap`, `Tracking` и `PointTrack`. Shooter использует энкодер
`shooter2` и общий PID. Номинальные значения в этой ветке — 2400 TPS,
120 TPS допуска, stopper 0.35 закрыт / 0.5 открыт.

## Проверка

```sh
rtk proxy ./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest :TeamCode:checkDebugDuplicateClasses
```

Для смены зависимостей нужен полный install. После подключения к Wi-Fi Control
Hub открой Panels по адресу `http://192.168.43.1:8001`. Для работы с маршрутами
отдельно измерь параметры Foresight на этом роботе. Подписанные offsets Point Track
и расстояния между модулями из CAD записаны отдельно.
