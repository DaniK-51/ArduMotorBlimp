# CHANGES.md

## Описание изменений в проекте ArduMotorBlimp

Дата: 2026-07-16  
Ветка: `feat/manual-only`

---

## Текущее состояние

Минимальная сборка для первого тестового полёта. Содержит только Manual + BRAKE режимы.

---

## Удалено (ветка manual-only)

### Режимы полёта
- `mode_velocity.cpp` — Velocity mode
- `mode_loiter.cpp` — Loiter mode
- `mode_rtl.cpp` — RTL mode
- `mode_auto.cpp` — AUTO mode
- `mission.cpp` — AP_Mission callbacks

### Контроллеры
- `Loiter.h/cpp` — Position/velocity PID controller
- PID контроллеры: `pid_vel_x`, `pid_vel_pitch`, `pid_vel_roll`, `pid_vel_yaw`, `pid_pos_x`, `pid_pos_pitch`, `pid_pos_roll`, `pid_pos_yaw`
- Velocity notch фильтры
- AP_Mission объект

### Параметры
- Все PID параметры (`VELX_*`, `VELPITCH_*`, `VELROLL_*`, `POSX_*`, `POSPITCH_*`, `POSROLL_*`, `POSYAW_*`)
- Параметры скорости/позиции (`MAX_VEL_X`, `MAX_VEL_PITCH`, `MAX_VEL_ROLL`, `MAX_POS_X`, `MAX_POS_PITCH`, `MAX_POS_ROLL`)
- `SIMPLE_MODE`, `DIS_MASK`, `PID_DZ`

---

## Осталось

### Режимы полёта
| Режим | Файл | Описание |
|-------|------|----------|
| MANUAL (1) | `mode_manual.cpp` | Прямой пропуск стиков → MotorMix |
| BRAKE (0) | `mode_brake.cpp` | Остановка моторов (failsafe) |

### Моторы
| Файл | Описание |
|------|----------|
| `MotorMix.h/cpp` | Матрица смешивания 4×4 |
| `motors.cpp` | Конвейер вывода, проверка арминга |

### Система
| Файл | Описание |
|------|----------|
| `radio.cpp` | Ввод с RC пульта |
| `system.cpp` | Инициализация |
| `commands.cpp` | Домашняя позиция |
| `failsafe.cpp` | Таймер failsafe |
| `events.cpp` | Обработка событий |
| `ekf_check.cpp` | Проверка EKF |

### GCS
| Файл | Описание |
|------|----------|
| `GCS_Blimp.cpp/h` | Heartbeat, статус |
| `GCS_Mavlink.cpp/h` | MAVLink обработка |

---

## Параметры (MotorMix)

| Параметр | Диапазон | Описание |
|----------|----------|----------|
| `M1_YAW` | -1..1 | Motor 1 yaw |
| `M1_PITCH` | -1..1 | Motor 1 pitch |
| `M1_ROLL` | -1..1 | Motor 1 roll |
| `M1_X` | -1..1 | Motor 1 X |
| ... | ... | ... |
| `M4_X` | -1..1 | Motor 4 X |

---

## Архитектура управления

```
RC Input → get_pilot_input() → [-1, +1]
    │
    ├── Manual: motors->*_out = pilot.*
    │
    └── BRAKE: motors->*_out = 0

MotorMix:
    [yaw, pitch, roll, x] × matrix → [M1..M4] → PWM
```

---

## История коммитов

| Коммит | Описание |
|--------|----------|
| `c61ad61` | feat(manual-only): strip down to Manual + BRAKE modes only |
| `1410489` | refactor: rename LAND mode to BRAKE mode |
