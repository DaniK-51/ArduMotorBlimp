# CHANGES.md

## Описание всех изменений в проекте ArduMotorBlimp

Дата: 2026-07-16

---

## Обзор

Выполнена полная переработка системы управления моторами и навигации:
1. Замена осциллирующих fin'ов на конфигурируемую матрицу смешивания моторов
2. Интеграция AP_Mission для полноценной навигации по waypoints
3. Добавление режима AUTO с логикой "повернуть-затем-лететь"
4. Адаптация всех режимов полёта под новую систему осей

---

## Новые файлы

### `MotorMix.h`
Класс MotorMix — замена Fins. Управляет 4 статичными моторами через матрицу смешивания.

**Ключевые изменения:**
- 4 оси управления: `yaw_out`, `pitch_out`, `roll_out`, `x_out` (вместо `right_out`, `front_out`, `down_out`, `yaw_out`)
- 16 параметров матрицы смешивания: `M1_YAW`..`M4_X` — вклад каждого мотора в каждую ось
- Простая линейная матрица: `motor_output[m] = Σ(weight[m][axis] * axis_out[axis])`
- Убрана осцилляция синусоидой — теперь прямой вывод на серво
- Поддержка логирования `Write_MOTORI` / `Write_MOTORO`

### `MotorMix.cpp`
Реализация класса MotorMix.

**Логика output():**
```
for each motor m:
    motor_output[m] = motor_yaw[m] * yaw_out
                    + motor_pitch[m] * pitch_out
                    + motor_roll[m] * roll_out
                    + motor_x[m] * x_out
    motor_output[m] = constrain(motor_output[m], -1, 1)
    SRV_Channels::set_output_scaled(motor_function(m), motor_output[m] * MOTOR_SCALE_MAX)
```

### `mode_auto.cpp`
Новый режим AUTO — автономный полёт по waypoints через AP_Mission.

**Логика turn-then-fly:**
1. **Phase 1 (Ориентация):** Если расстояние > 2×radius и угловая ошибка > 10°:
   - Yaw PID: `pid_pos_yaw` → `pid_vel_yaw` → `yaw_out`
   - Pitch PID: `pid_pos_pitch` → `pid_vel_pitch` → `pitch_out`
   - Одновременная ориентация yaw + pitch
   - Без движения вперёд (`x_out = 0`)
2. **Phase 2 (Полёт):** Когда ориентация завершена — `loiter->run()` для навигации к точке

### `mission.cpp`
Callbacks для AP_Mission — обработка команд миссии.

**Обработанные команды:**
- `MAV_CMD_NAV_WAYPOINT` — навигация к точке
- `MAV_CMD_NAV_LAND` — посадка
- `MAV_CMD_NAV_TAKEOFF` — взлёт

**Callbacks:**
- `start_command()` — вызывается при старте новой команды
- `verify_command()` — вызывается для проверки выполнения команды
- `mission_complete()` — вызывается при завершении миссии
- `verify_land()` — возвращает true когда `ap.land_complete || !motors->armed()`
- `verify_takeoff()` — возвращает true сразу (дирижабль плавучий)

---

## Изменённые файлы

### `Blimp.h`
**Изменения:**
- `#include "Fins.h"` → `#include "MotorMix.h"`
- `friend class Fins` → `friend class MotorMix`
- `Fins *motors` → `MotorMix *motors`
- Добавлен `#include <AP_Mission/AP_Mission.h>`
- Добавлен `AP_Mission mission` с callbacks
- Замена PID контроллеров:
  - Удалены: `pid_vel_xy` (AC_PID_2D), `pid_vel_z`, `pid_pos_xy` (AC_PID_2D), `pid_pos_z`
  - Добавлены: `pid_vel_x`, `pid_vel_pitch`, `pid_vel_roll`, `pid_pos_x`, `pid_pos_pitch`, `pid_pos_roll` (все AC_PID_Basic)
  - Сохранены: `pid_vel_yaw`, `pid_pos_yaw`
- Добавлен `ModeAuto mode_auto` и `friend class ModeAuto`
- Добавлены declarations: `start_command`, `verify_command`, `mission_complete`, `do_nav_wp`, `verify_nav_wp`, `do_land`, `verify_land`, `do_takeoff`, `verify_takeoff`
- Замена логирования: `Write_FINI/Write_FINO` → `Write_MOTORI/Write_MOTORO`

### `Parameters.h`
**Изменения:**
- Замена enum PID: `k_param_pid_vel_xy` → `k_param_pid_vel_x`, добавлены `k_param_pid_vel_pitch`, `k_param_pid_vel_roll`, `k_param_pid_pos_pitch`, `k_param_pid_pos_roll`
- Замена переменных скорости/позиции:
  - `max_vel_xy` → `max_vel_x`
  - `max_vel_z` → `max_vel_pitch`
  - Добавлены: `max_vel_roll`, `max_pos_x`, `max_pos_pitch`, `max_pos_roll`

### `Parameters.cpp`
**Изменения:**
- Замена параметров PID:
  - `VELXY_` → `VELX_` (AC_PID_Basic вместо AC_PID_2D)
  - `VELZ_` → `VELPITCH_`
  - Добавлены: `VELROLL_`, `POSX_`, `POSPITCH_`, `POSROLL_`
- Замена параметров скорости/позиции:
  - `MAX_VEL_XY` → `MAX_VEL_XY` (0.5 m/s)
  - `MAX_VEL_Z` → `MAX_VEL_PITCH` (0.4 rad/s)
  - `MAX_POS_XY` → `MAX_POS_X` (0.2 m/s)
  - `MAX_POS_Z` → `MAX_POS_PITCH` (0.15 rad/s)
- Замена группы параметров: `FINS_` → `MOTOR_`
- Обновлён `DIS_MASK`: биты 0:Roll, 1:X, 2:Pitch, 3:Yaw
- Добавлен `AUTO` в `FLTMODE` values

### `Loiter.h`
**Изменения:**
- `scaler_xz` → `scaler_xr` (X + Roll)
- `scaler_yyaw` → `scaler_pyaw` (Pitch + Yaw)

### `Loiter.cpp`
**Полная переработка осей:**

| Старое | Новое | Описание |
|--------|-------|----------|
| `front_out` | `x_out` | Линейное движение по X |
| `right_out` | `roll_out` | Вращение вокруг X |
| `down_out` | `pitch_out` | Вращение вокруг Y |
| `yaw_out` | `yaw_out` | Вращение вокруг Z |

**PID каскад:**
```
Position PID: (target_pos - pos_ned) → target_vel
    pid_pos_x(target_pos.x, pos_ned.x) → target_vel_x
    pid_pos_pitch(target_pos.z, pos_ned.z) → target_vel_pitch
    pid_pos_roll(target_pos.y, pos_ned.y) → target_vel_roll
    pid_pos_yaw(yaw_error) → target_vel_yaw

Velocity PID: (target_vel - vel_ned_filtd) → actuator
    pid_vel_x(target_vel_x, vel_x_filtd) → act_x
    pid_vel_pitch(target_vel_pitch, vel_z_filtd) → act_pitch
    pid_vel_roll(target_vel_roll, vel_y_filtd) → act_roll
    pid_vel_yaw(target_vel_yaw, vel_yaw_filtd) → act_yaw

Output: actuator → motors->*_out
```

### `mode_manual.cpp`
**Изменения mappings:**
```
// Старое:
motors->right_out = pilot.y;    // Roll stick → right
motors->front_out = pilot.x;    // Pitch stick → front
motors->yaw_out = pilot_yaw;    // Yaw stick → yaw
motors->down_out = pilot.z;     // Throttle stick → down

// Новое:
motors->roll_out = pilot.y;     // Roll stick → roll
motors->pitch_out = pilot.z;    // Throttle stick → pitch
motors->x_out = pilot.x;        // Pitch stick → X (forward/backward)
motors->yaw_out = pilot_yaw;    // Yaw stick → yaw
```

### `mode_velocity.cpp`
**Изменения:**
```
target_vel.x *= g.max_vel_x;      // было max_vel_xy
target_vel.y *= g.max_vel_roll;   // было max_vel_xy
target_vel.z *= g.max_vel_pitch;  // было max_vel_z
```

### `mode_loiter.cpp`
**Изменения:**
```
pilot.x *= g.max_pos_x * dt;      // было max_pos_xy
pilot.y *= g.max_pos_roll * dt;   // было max_pos_xy
pilot.z *= g.max_pos_pitch * dt;  // было max_pos_z
```

### `mode_land.cpp`
**Изменения:**
```
// Старое:
motors->right_out = 0;
motors->front_out = 0;
motors->yaw_out = 0;
motors->down_out = 0;

// Новое:
motors->yaw_out = 0;
motors->pitch_out = 0;
motors->roll_out = 0;
motors->x_out = 0;
```

### `mode_rtl.cpp`
Без изменений (использует loiter->run с target {0,0,0}).

### `mode.h`
**Изменения:**
- Добавлен `AUTO = 5` в enum `Mode::Number`
- Упрощён `ModeAuto` — убраны `MissionItem waypoints[]`, добавлены `set_wp_target()`, `has_target()`, `clear_target()`

### `mode.cpp`
**Изменения:**
- Добавлен `case Mode::Number::AUTO: ret = &mode_auto;` в `mode_from_mode_num()`

### `system.cpp`
**Изменения:**
- `allocate_motors()`: `Fins::MOTOR_FRAME_AIRFISH` → `MotorMix::MOTOR_FRAME_MIXED`
- `motors = NEW_NOTHROW Fins(...)` → `motors = NEW_NOTHROW MotorMix(...)`
- `motors->setup_fins()` → `motors->setup_motors()`
- Добавлен `mission.init()`
- Удалена инициализация fin notch фильтров (больше не нужны)
- `get_frame_string()`: `"AIRFISH"` → `"MIXED"`

### `GCS_Mavlink.h`
**Изменения:**
- Удалён `handle_mission_item_int()` (обрабатывается базовым классом)
- PID_SEND enum: `VELY` → `VELPITCH`, `VELZ` → `VELROLL`, `POSY` → `POSPITCH`, `POSZ` → `POSROLL`

### `GCS_Mavlink.cpp`
**Изменения:**
- Удалён кастомный `handle_mission_item_int()` — миссии обрабатываются через `MissionItemProtocol_Waypoints`
- `send_pid_tuning()`: обновлены PID references для новых осей
- `handle_command_int_packet()`: убран `MAV_CMD_MISSION_ITEM_INT` case

### `GCS_Blimp.h`
**Изменения:**
- Добавлен `#include <GCS_MAVLink/MissionItemProtocol_Waypoints.h>`
- Добавлен `void init() override`
- Добавлен `MissionItemProtocol_Waypoints* _mission_item_protocol`

### `GCS_Blimp.cpp`
**Изменения:**
- Добавлен `GCS_Blimp::init()`:
  ```cpp
  void GCS_Blimp::init() {
      GCS::init();
      _mission_item_protocol = NEW_NOTHROW MissionItemProtocol_Waypoints(blimp.mission);
      if (_mission_item_protocol != nullptr) {
          missionitemprotocols[0] = _mission_item_protocol;
      }
  }
  ```
- Подключает AP_Mission к MAVLink protocol для upload/download миссий

### `Log.cpp`
**Изменения:**
- Структуры: `log_FINI` → `log_MOTORI`, `log_FINO` → `log_MOTORO`
- Функции: `Write_FINI()` → `Write_MOTORI()`, `Write_FINO()` → `Write_MOTORO()`
- PID логирование: `pid_vel_xy.get_pid_info_x()` → `pid_vel_x.get_pid_info()`
- Log structures: `FINI` → `MOTORI`, `FINO` → `MOTORO`
- Добавлены `PIDN` и `PIDE` log structures

### `defines.h`
**Изменения:**
- `LOG_FINI_MSG` → `LOG_MOTORI_MSG`
- `LOG_FINO_MSG` → `LOG_MOTORO_MSG`
- Добавлены `LOG_PIDN_MSG`, `LOG_PIDE_MSG`

---

## Удалённые файлы

### `Fins.h` / `Fins.cpp`
Заменены на `MotorMix.h` / `MotorMix.cpp`. Содержали:
- Осциллирующие fin'ы с синусоидальным выходом
- Амплитудно-смещённое управление
- Жёстко заданную конфигурацию 4 fin'ов

### Остатки Fins (удалены в cleanup):
- `mode.h`: `Fins *&motors` → `MotorMix *&motors`
- `Loiter.h`: `friend class Fins` → `friend class MotorMix`
- `config.h`: `Fins::MOTOR_FRAME_TYPE_AIRFISH` → `MotorMix::MOTOR_FRAME_TYPE_MIXED`
- `RC_Channel.h`: `#include "Fins.h"` → `#include "MotorMix.h"`
- `Parameters.h`: переименованы enum values для совместимости

---

## Параметры

### Новые параметры матрицы смешивания (MOTOR_*)
| Параметр | Описание | Диапазон |
|----------|----------|----------|
| `M1_YAW`..`M4_YAW` | Вклад мотора в yaw | -1..1 |
| `M1_PITCH`..`M4_PITCH` | Вклад мотора в pitch | -1..1 |
| `M1_ROLL`..`M4_ROLL` | Вклад мотора в roll | -1..1 |
| `M1_X`..`M4_X` | Вклад мотора в движение по X | -1..1 |

### Новые PID параметры
| Параметр | По умолчанию | Описание |
|----------|-------------|----------|
| `VELX_P/I/D/FF` | 3/0.2/0/0 | Скорость X |
| `VELPITCH_P/I/D/FF` | 3/0.2/0/0 | Скорость pitch |
| `VELROLL_P/I/D/FF` | 3/0.2/0/0 | Скорость roll |
| `POSX_P/I/D/FF` | 1/0.05/0/0 | Позиция X |
| `POSPITCH_P/I/D/FF` | 1/0.05/0/0 | Позиция pitch |
| `POSROLL_P/I/D/FF` | 1/0.05/0/0 | Позиция roll |

### Удалённые параметры
- `VELXY_*` (AC_PID_2D) → заменены на `VELX_*` + `VELROLL_*`
- `VELZ_*` → заменены на `VELPITCH_*`
- `POSXY_*` (AC_PID_2D) → заменены на `POSX_*` + `POSROLL_*`
- `POSZ_*` → заменены на `POSPITCH_*`
- `MAX_VEL_XY` → `MAX_VEL_X` + `MAX_VEL_ROLL`
- `MAX_VEL_Z` → `MAX_VEL_PITCH`
- `MAX_POS_XY` → `MAX_POS_X` + `MAX_POS_ROLL`
- `MAX_POS_Z` → `MAX_POS_PITCH`
- `FINS_FREQ_HZ`, `FINS_TURBO_MODE` — удалены (нет осцилляции)

---

## Архитектура управления

```
RC Input → Mode::get_pilot_input() → [-1, +1]
    │
    ├── Manual: прямой пропуск → motors->*_out
    │
    ├── Velocity: масштабирование → loiter->run_vel()
    │
    ├── Loiter: накопление цели → loiter->run()
    │
    ├── RTL: цель {0,0,0} → loiter->run()
    │
    └── AUTO: AP_Mission.update() → turn-then-fly / loiter->run()
                                        │
                                        ├── Phase 1: yaw + pitch PID → motors->yaw_out, pitch_out
                                        │
                                        └── Phase 2: loiter->run() → все оси

Loiter PID cascade:
    Position PID → target_vel → Velocity PID → actuator → motors->*_out

MotorMix:
    [yaw_out, pitch_out, roll_out, x_out] × matrix → [M1, M2, M3, M4] → SRV_Channels
```

---

## Статус

| Компонент | Статус |
|-----------|--------|
| MotorMix (матрица смешивания) | ✅ Готово |
| PID контроллеры (x, pitch, roll, yaw) | ✅ Готово |
| Loiter controller | ✅ Готово |
| Manual mode | ✅ Готово |
| Velocity mode | ✅ Готово |
| Loiter mode | ✅ Готово |
| RTL mode | ✅ Готово |
| AUTO mode (turn-then-fly) | ✅ Готово |
| AP_Mission integration | ✅ Готово |
| GCS mission protocol | ✅ Готово |
| Logging | ✅ Готово |
| Parameter system | ✅ Готово |
