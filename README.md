# ArduMotorBlimp

Прошивка ArduPilot 4.7 для цилиндрического дирижабля с четырьмя
реверсивными винтами, направленными вдоль корпуса. Это отдельный vehicle
target, а не адаптация штатного `Blimp`: штатный target управляет машущими
плавниками и к этой механике не подходит.

## Что реализовано

- `MANUAL` — стабилизированный ручной полёт: signed collective,
  roll/pitch по углу, yaw-rate напрямую по гироскопу без удержания курса;
- `HOLD` — удержание текущей ориентации с нулевой тягой;
- `GUIDED` — одна постоянная цель `SET_POSITION_TARGET_LOCAL_NED`; цель не
  требует непрерывного потока команд;
- `AUTO` — автономная waypoint-миссия, полностью хранящаяся и исполняющаяся
  на борту;
- EKF3-позиция от одной UWB-метки (`AP_Beacon`/Nooploop), абсолютный курс
  для `HOLD/AUTO/GUIDED` от внешнего компаса;
- 6DoF SITL-модель именно этой моторной компоновки, включая плавучесть,
  анизотропное сопротивление и реактивный roll-момент винтов;
- безопасный реверсивный выход: `1000 = полный реверс`, `1500 = нейтраль`,
  `2000 = полный вперёд`;
- RC, UWB/navigation, compass и battery failsafe: при недействительном входе
  моторы сразу получают нейтраль, старое значение тяги не сохраняется;
- независимый watchdog даёт neutral и disarm при остановке main loop на 200 мс.

SITL протестирован. Аппаратная сборка для `MicoAir743v2` также проходит
cross-build, но ещё не проверена на физической плате. Пошаговая загрузка,
подключение ELRS и безопасный стендовый чек-лист описаны в
[README-HARDWARE.md](README-HARDWARE.md). До полёта обязательны проверки без пропеллеров:
реверс ESC, соответствие M1…M4, знаки моментов и failsafe.

## Геометрия и знаки

Используется система координат ArduPilot FRD: `+X` вперёд по оси цилиндра,
`+Y` вправо, `+Z` вниз. Нумерация при взгляде со стороны носа:

| Мотор | Положение | Реактивный roll-момент |
|---|---|---|
| M1 | сверху | `+roll` |
| M2 | справа | `-roll` |
| M3 | снизу | `+roll` |
| M4 | слева | `-roll` |

Канонический mixer:

```text
M1 = F + roll - pitch
M2 = F - roll - yaw
M3 = F + roll + pitch
M4 = F - roll + yaw
```

Сначала allocator сохраняет направление attitude-команды, при необходимости
одинаково масштабируя roll/pitch/yaw, и только затем ограничивает collective.
Независимый clip четырёх моторов не используется.

## RC и режимы

Все четыре основных канала центрированы на 1500:

| Канал | Команда |
|---|---|
| RC1 | roll angle |
| RC2 | pitch angle |
| RC3 | signed forward/reverse collective |
| RC4 | body yaw rate; в центре — нулевая скорость вращения |

Номера режимов: `MANUAL=0`, `HOLD=4`, `AUTO=10`, `GUIDED=15`.

`MANUAL` может армиться и работать без магнитометра: RC4 задаёт скорость
вращения, измеряемую гироскопом, поэтому абсолютный курс постепенно дрейфует.
`HOLD`, `AUTO` и `GUIDED` требуют здоровый компас и отклоняются без него.

`GUIDED` принимает только `MAV_FRAME_LOCAL_NED`, только одновременно X/Y/Z и
только position target. Velocity, acceleration, yaw и yaw-rate должны быть
помечены ignored; force-set запрещён. Принятая цель удерживается до следующей
цели или смены режима.

`AUTO` пока намеренно принимает только `MAV_CMD_NAV_WAYPOINT`. Mission upload
поддерживается стандартным ArduPilot Mission Protocol. После последней точки
аппарат остаётся в `AUTO` и удерживает последнюю цель; связь с GCS для полёта
не нужна.

## Быстрый запуск SITL

Интеграция привязана к точному ArduPilot SHA
`331c42a50c1f68b0065d4944e55eb688b62fe9c4` (`ArduPilot-4.7`). Репозиторий
нужно разместить внутри checkout ArduPilot под именем `ArduMotorBlimp`:

```sh
git clone --branch ArduPilot-4.7 --recursive \
  https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout 331c42a50c1f68b0065d4944e55eb688b62fe9c4
git submodule update --init --recursive
git clone --branch feat/building-from-zero \
  https://github.com/DaniK-51/ArduMotorBlimp.git ArduMotorBlimp

./ArduMotorBlimp/scripts/patch-ardupilot.sh
./waf configure --board sitl
./waf build --target bin/ardumotorblimp
./Tools/autotest/sim_vehicle.py \
  -v ArduMotorBlimp -f motorblimp -w --console --map
```

Детали прямого запуска и тестов находятся в [README-SITL.md](README-SITL.md).

## UWB + compass для автономного полёта

SITL defaults используют одну моделируемую метку `AP_Beacon_SITL` и компас:

```text
BCN_TYPE          10
GPS1_TYPE          0
AHRS_EKF_TYPE      3
EK3_SRC1_POSXY     4   # Beacon
EK3_SRC1_POSZ      4   # Beacon
EK3_SRC1_VELXY     0
EK3_SRC1_VELZ      0
EK3_SRC1_YAW       1   # Compass
UWB_ERR_MAX       1.0  # maximum accepted direct-fix error, metres
```

Для реального Nooploop на выбранном UART базовая замена выглядит так:

```text
SERIAL1_PROTOCOL  13   # Beacon
SERIAL1_BAUD     115   # 115200, сверить с настройкой tag
BCN_TYPE           3   # Nooploop
GPS1_TYPE          0
EK3_SRC1_POSXY     4
EK3_SRC1_POSZ      4
EK3_SRC1_VELXY     0
EK3_SRC1_VELZ      0
EK3_SRC1_YAW       1
UWB_ERR_MAX       1.0
```

`BCN_LATITUDE`, `BCN_LONGITUDE` и `BCN_ALT` должны задавать локальный origin
зала. Nooploop direct fix считается свежим только при минимум четырёх валидных
anchor-блоках и ошибке не выше `UWB_ERR_MAX`; после 300 мс без валидного fix
AUTO/GUIDED дают neutral. UWB-метку желательно ставить около центра вращения;
текущий драйвер не компенсирует lever arm метки при поворотах корпуса.

## Проверки

```sh
python3 ArduMotorBlimp/tests/run_controller_tests.py \
  --ardupilot .

python3 ArduMotorBlimp/tests/sitl_smoke.py \
  --binary build/sitl/bin/ardumotorblimp \
  --defaults Tools/autotest/default_params/motorblimp.parm \
  --check-guided
```

Тесты проверяют математику controller/mixer, neutral/reverse PWM, знаки всех
трёх моментов, поступательное движение и состояние модели через MAVLink.

## Перед первым реальным запуском

1. Снять пропеллеры и подтвердить, что disarm/failsafe всегда дают 1500 мкс
   либо отсутствие сигнала, которое конкретный bidirectional ESC трактует как
   stop.
2. По одному проверить соответствие физических выходов M1…M4 таблице выше.
3. Проверить знак каждого момента по движению корпуса, а не по неоднозначным
   словам CW/CCW.
4. Проверить достаточность реактивного roll-момента; это самый слабый канал
   управления данной механики.
5. Перед `HOLD/AUTO/GUIDED` подключить и откалибровать внешний компас с
   работающими BLDC, исключив магнитное насыщение.
6. Ограничить `MAN_FWD_MAX`, `NAV_FWD_MAX`, углы и PID до безопасных значений
   перед установкой пропеллеров.

Прошивка не создаёт боковую или вертикальную силу, которой нет в аппарате:
для набора высоты controller сначала направляет ось корпуса вверх с ограничением
`NAV_PIT_MAX`, затем разрешает продольную тягу по мере совпадения ориентации.
