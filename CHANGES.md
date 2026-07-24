# CHANGES.md

## Описание изменений в проекте ArduMotorBlimp

Ветка: `feat/manual-only`  
ArduPilot: Copter-4.6.3

---

## Текущее состояние

Минимальная сборка для первого тестового полёта: Manual + BRAKE.

**Ключевые особенности:**
- Manual mode не зависит от датчиков (AHRS, EKF, GPS)
- `AP_MotorsBlimp` наследует `AP_Motors` (полная поддержка протоколов)
- Бидирекциональный PWM (1000-2000, 1500=стоп)
- Арминг по кнопке через AUX канал
- Motor protocol: `MOTOR_PWM_TYPE` (DSHOT, OneShot, PWM)

---

## Архитектура

```
RC Input → get_pilot_input() → [-1, +1]
    │
    ├── Manual: motors->set_roll/pitch/yaw/throttle()
    │
    └── BRAKE: все выходы = 0

AP_MotorsBlimp:
    [roll, pitch, yaw, throttle] × matrix → [M1..M4] → PWM 1000-2000
```

---

## Файлы проекта (28 файлов)

| Файл | Описание |
|------|----------|
| `Blimp.cpp/h` | Ядро: scheduler, конструктор |
| `AP_MotorsBlimp.cpp/h` | Наследник AP_Motors, матрица смешивания |
| `AP_Arming.cpp/h` | Арминг (упрощён для manual-only) |
| `mode.cpp/h` | Базовый класс Mode |
| `mode_manual.cpp` | Manual — прямой пропуск |
| `mode_brake.cpp` | BRAKE — аварийная остановка |
| `motors.cpp` | Конвейер вывода |
| `radio.cpp` | RC ввод, failsafe |
| `RC_Channel.cpp/h` | AUX функции (ARMDISARM) |
| `GCS_Blimp.cpp/h` | GCS |
| `GCS_Mavlink.cpp/h` | MAVLink (упрощён) |
| `failsafe.cpp` | Таймер main loop hang |
| `events.cpp` | Обработка failsafe |
| `AP_State.cpp` | Флаги состояния |
| `Log.cpp` | MOTORI/MOTORO логи |
| `Parameters.cpp/h` | Параметры |
| `system.cpp` | init_ardupilot(), allocate_motors() |
| `config.h`, `defines.h`, `version.h` | Конфигурация |
| `wscript` | Сборка |

---

## Удалено (ветка manual-only)

- Режимы: Velocity, Loiter, RTL, AUTO, mission
- Контроллеры: Loiter PID, velocity/position PIDs, notch фильтры
- Файлы: `inertia.cpp`, `sensors.cpp`, `ekf_check.cpp`, `commands.cpp`
- Параметры: PID, velocity, position, SIMPLE_MODE, DIS_MASK

---

## Motor Protocol

`MOTOR_PWM_TYPE`: 0=Normal, 1=OneShot, 2=OneShot125, 3=Brushed, 4=DShot150, 5=DShot300, 6=DShot600, 7=DShot1200

---

## Арминг

AUX_FUNC=31 (ARMDISARM) на канале 5-8. HIGH=арм, LOW=дизарм.

---

## Failsafe

- RC lost → BRAKE
- GCS lost → BRAKE
- Battery → BRAKE
- Main loop hang → output_min()

---

## История коммитов

| Хэш | Описание |
|------|----------|
| `9d13f2d` | refactor: firmware size optimization |
| `6686fdc` | fix: add required libraries for GCS and Lua bindings |
| `c0cea40` | fix: clean up wscript |
| `0ca7dcf` | fix: add missing hal declaration and AP_Motors to wscript |
| `da86754` | fix: revert ModeReason::AUX_SWITCH back to AUX_FUNCTION |
| `0dda457` | fix: AUX_FUNCTION -> AUXSWITCH for Copter 4.6.3 |
| `5fc2d94` | fix: three more compilation errors |
| `e4b113d` | fix: three more compilation errors |
| `9410d1e` | fix: two more compilation errors |
| `e360788` | fix: three more compilation errors |
| `ba79849` | fix: remove AP_InertialNav/AP_AHRS from mode.h |
| `ad4263f` | refactor: full vehicle cleanup for manual-only build |
| `f2cb8a6` | fix: remove send_pid_tuning references to non-existent PIDs |
| `dfc89ff` | fix: comprehensive cleanup of AHRS/sensor dependencies |
| `b90c224` | docs: update AGENTS.md |
| `a4d4d29` | docs: update all documentation |
| `419100a` | fix: use set_output_scaled for protocol-aware motor output |
| `e66a46f` | docs: update README for AP_MotorsBlimp |
| `61b43b6` | refactor: replace MotorMix with AP_MotorsBlimp |
| `9ed6cb1` | feat: button-based arming via AUX channel |
| `9a4eafe` | feat: remove sensor dependencies, enable bidirectional motors |
| `c61ad61` | feat(manual-only): strip down to Manual + BRAKE modes only |
| `1410489` | refactor: rename LAND mode to BRAKE mode |
