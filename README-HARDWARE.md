# MicoAir743v2-AIO-35A: загрузка и ручное управление

Эта инструкция относится только к плате **MicoAir743v2-AIO-35A**.
Её ArduPilot target называется `MicoAir743v2`. Если на плате написано
`MicoAir743-AIO` без `v2`, остановиться: это другой target.

Аппаратный образ собран и проверен на уровне сборки. Физическая плата,
моторы и ESC ещё не пройдены на стенде. Все первые проверки — со снятыми
пропеллерами и зафиксированным корпусом.

## 1. Первая загрузка

Использовать `ardumotorblimp-MicoAir743v2-with_bl.hex`: в нём есть bootloader.

1. Снять пропеллеры. Отключить LiPo и все мощные нагрузки.
2. Зажать `BOOT`, подключить USB-C, дождаться DFU-устройства и отпустить
   кнопку.
3. Открыть MicoConfigurator в Chrome/Edge, выбрать DFU/локальный файл и target
   **`MicoAir743v2`**. Не выбирать похожий `MicoAir743-AIO`.
4. Загрузить HEX, дождаться erase/program/verify, отключить USB и подключить его
   заново без `BOOT`.
5. Для следующих обновлений через ArduPilot bootloader использовать
   `ardumotorblimp-MicoAir743v2.apj`.

Не загружать на плату SITL-бинарник и не импортировать целиком `mav.parm` из SITL.

## 2. Подключение ELRS

Для RadioMaster EP1/EP2 используется UART6:

| ELRS receiver | MicoAir743v2-AIO-35A |
|---|---|
| `TX` | `RX6` |
| `RX` | `TX6` |
| `5V` | `5V` |
| `GND` | `GND` |

Не ориентироваться на цвета проводов; TX и RX должны быть перекрещены.
После пайки прозвонить питание и убедиться, что 5 V не попадают на сигнальные пады.

На свежих параметрах платы загрузить `motorblimp-micoair743v2-rc-bench.parm` либо задать:

```text
SERIAL6_PROTOCOL 23
SERIAL6_OPTIONS   0
RC_PROTOCOLS      512
RC_OPTIONS        8224
RC_FS_TIMEOUT     1.0
RCMAP_ROLL        1
RCMAP_PITCH       2
RCMAP_THROTTLE    3
RCMAP_YAW         4
RC5_OPTION        153
RC6_OPTION        31
MAN_FWD_MAX       0.10
EK3_SRC1_YAW      0
BATT_AMP_PERVLT   14.14
```

`RC_OPTIONS=8224` включает 420 kbaud ELRS и сохраняет проверку нейтрали RC3. Если
в `RC_OPTIONS` уже есть другие флаги, не заменять всю маску: в UI добавить биты 5 и 13.
Значение `BATT_AMP_PERVLT=14.14` относится к встроенному датчику тока именно этой
AIO-платы; позже его нужно сверить с внешним ваттметром.
После изменения serial/RC параметров перезагрузить плату.

## 3. RadioMaster TX12 MkII

1. Создать новую модель EdgeTX.
2. `Internal RF` = `CRSF`, `External RF` = `Off`.
3. У TX-модуля и EP1/EP2 должны совпадать ELRS major version, regulatory domain и binding
   phrase. Проще всего прошить оба устройства с одной binding phrase.
4. Раскладка AETR: CH1 roll, CH2 pitch, CH3 signed collective, CH4 yaw.
5. CH5 — двухпозиционный arm/disarm. CH6 — отдельный motor emergency stop.
6. Механически включить пружинное центрирование вертикальной оси throttle-стика.
   Здесь CH3 — не обычный газ: `1000=назад`, `1500=стоп`, `2000=вперёд`.
7. В GCS выполнить Radio Calibration. Все CH1…CH4 должны показывать около 1500 при
   отпущенных стиках.

На столе начать с 25 mW. В receiver failsafe нужен `No pulses`/протокольный failsafe,
а не `Hold last` и не `Throttle low`: для этой механики low означает полный реверс.

## 4. Компас и первое arm

На AIO-плате нет встроенного магнитометра. Подключить внешний компас к `SDA`,
`SCL`, питанию и GND, разместить его подальше от ESC, силовых проводов и моторов,
затем откалибровать перед использованием `HOLD`, `AUTO` или `GUIDED`.

Для первой проверки в `MANUAL` компас не нужен. RC4 напрямую задаёт yaw-rate,
который замыкается по гироскопу; в центре требуется нулевая скорость вращения,
но абсолютный курс не удерживается и может дрейфовать. Без здорового компаса
прошивка разрешает обычный arm только в `MANUAL` и отклоняет остальные режимы.
Стендовый файл поэтому задаёт `EK3_SRC1_YAW=0`. После подключения и калибровки
внешнего магнитометра установить `EK3_SRC1_YAW=1` и перезагрузить плату перед
проверкой `HOLD/AUTO/GUIDED`. Значение `1` выбирает источник типа Compass, но
само по себе не доказывает, что выбран именно внешний датчик: проверить
`COMPASS_PRIO1_ID`/`COMPASS_DEV_ID`, внешний статус и успешное yaw alignment.

Стандартного flight-mode switch пока нет; после старта активен `MANUAL`. Для первой
проверки arm и disarm можно выполнить из MAVProxy:

```text
arm throttle
disarm
```

После проверки CH5 и CH6 армиться тумблером. Положения проверить в Radio
Monitor: CH5 low=disarm, high=arm; CH6 high=emergency stop.

## 5. Обязательный ESC gate

Встроенные ESC платы принимают DShot300/600; обычный PWM не подходит. При этом
производитель указывает Bluejay 0.19.2, а в release notes Bluejay сказано, что 3D-режим
был сломан в 0.17.x…0.19.x. В 0.20 его вернули, но там была опасная ошибка startup
protection; для 0.21 сами разработчики не советуют 3D из-за проблем с производительностью.

Поэтому нельзя прошивать ESC случайно выбранным Bluejay HEX или переходить к полёту
с заводской 0.19.2. Нужен один из двух закрытых результатов:

- подтверждённая MicoAir/Bluejay реверсивная прошивка именно для автоопределённых
  layout/MCU/deadtime этого AIO, с безопасными startup settings;
- замена на отдельные ESC, у которых реверсивный DShot подтверждён производителем.

Только после этого gate включать в ArduPilot:

```text
SERVO_BLH_MASK    15
SERVO_BLH_OTYPE   5
SERVO_BLH_3DMASK  15
SERVO_DSHOT_ESC   2
```

`5` — DShot300, `15` — выходы 1…4, `2` — BLHeli_S/Bluejay command set. Для AM32/BLHeli32 значение
`SERVO_DSHOT_ESC` другое; не переносить `2` на другой тип ESC. Перезагрузить плату.

DShot с этой прошивкой работает только на ArduPilot с патчем из `patches/`: штатный
`AP_BLHeli::init` применяет режим выхода раньше, чем узнаёт тип ESC, поэтому для вехиклов
без `AP_Motors` период бита запекается в таймер с таймингами по умолчанию, а ширины бит 0/1
берутся уже BLHeli_S-овские — кадр не декодируется, ESC молчат при внешне корректных
`SERVO_OUTPUT_RAW` и баннере `RCOut: DS300:1-4`. Патч переносит `set_dshot_esc_type`
перед `set_output_mode` (подтверждено на стенде 2026-09-03: до патча моторы не крутились,
после — крутятся; стоковый Copter на той же плате не подвержен).

Затем без пропеллеров по одному проверить M1…M4, stop в 1500, оба направления,
disarm, CH6 e-stop и потерю RC. Не полагаться на названия `forward/reverse`: знак тяги каждого
мотора проверяется физически; после этого составляется `SERVO_BLH_RVMASK` из битов
M1=1, M2=2, M3=4, M4=8.

## 6. Первый стендовый прогон

1. Только USB, LiPo отключен: связь ELRS, движение CH1…CH6, нейтраль CH1…CH4.
2. Пропеллеры сняты, корпус зафиксирован: `MANUAL` arm/disarm без компаса,
   e-stop и RC failsafe; после установки компаса отдельно проверить его failsafe.
3. Только после ESC gate: малая команда CH3 от 1500 в обе стороны, поочерёдная
   проверка выходов и знаков.
4. Потеря связи должна дать neutral/stop, но не автоматический disarm. После потери RC
   выполнить disarm и источник сбоя перед повторным arm.
5. Пропеллеры не ставить, пока не закрыты все пункты и не ограничены PID/углы/тяга.

## Первичные источники

- [MicoAir743v2-AIO-35A User Manual](https://micoair.cn/en/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-35a-manual)
- [MicoConfigurator](https://micoair.com/configurator/)
- [ExpressLRS: ArduPilot setup](https://www.expresslrs.org/quick-start/ardupilot-setup/)
- [ExpressLRS: binding](https://www.expresslrs.org/quick-start/binding/)
- [ArduPilot: reversible DShot ESCs](https://ardupilot.org/copter/docs/common-dshot-escs.html#reversible-dshot-escs)
- [Bluejay release notes](https://github.com/bird-sanctuary/bluejay/releases)
