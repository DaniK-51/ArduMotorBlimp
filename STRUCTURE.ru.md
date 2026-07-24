# ArduMotorBlimp — Структура и архитектура

**Версия:** 3.0  
**Дата:** 24 июля 2026  
**Ветка:** `feat/manual-only`  
**Репозиторий:** https://github.com/DaniK-51/ArduMotorBlimp  
**Автор:** DaniK-51 (Daniyar)  
**Лицензия:** GPL-3.0  
**База ArduPilot:** Copter-4.6.3

---

Это ветка **manual-only** — минимальная сборка только с режимами Manual и BRAKE для первого тестового полёта. Все автономные режимы, датчики (AHRS, EKF, GPS) и PID-контроллеры удалены для минимизации размера прошивки.

**AHRS, EKF, GPS, компас, барометр не требуются.** Только RC ввод → MotorMix → Моторы.

---

## Структура репозитория

```
ArduMotorBlimp/
├── Blimp.cpp              # Главный цикл, scheduler, конструктор
├── Blimp.h                # Основной класс
├── AP_MotorsBlimp.cpp     # Наследник AP_Motors, матрица смешивания, вывод на серво
├── AP_MotorsBlimp.h       # Объявление AP_MotorsBlimp
├── AP_Arming.cpp          # Проверки арминга (упрощён)
├── AP_Arming.h
├── AP_State.cpp           # Флаги состояния
├── GCS_Blimp.cpp          # Класс GCS
├── GCS_Blimp.h
├── GCS_Mavlink.cpp        # Обработка MAVLink (упрощён)
├── GCS_Mavlink.h
├── Log.cpp                # Логи MOTORI/MOTORO
├── Parameters.cpp         # Определения параметров
├── Parameters.h
├── RC_Channel.cpp         # AUX функции (ARMDISARM)
├── RC_Channel.h
├── config.h               # Конфигурация при компиляции
├── defines.h              # Константы и enum'ы
├── events.cpp             # Обработка событий failsafe
├── failsafe.cpp           # Таймер failsafe (обнаружение зависания main loop)
├── mode.cpp               # Базовый класс Mode, get_pilot_input()
├── mode.h                 # Перечисление и объявления режимов
├── mode_brake.cpp         # BRAKE (аварийная остановка)
├── mode_manual.cpp        # Manual (прямой пропуск)
├── motors.cpp             # Конвейер вывода
├── radio.cpp              # Чтение RC, failsafe
├── system.cpp             # init_ardupilot(), allocate_motors()
├── version.h
└── wscript                # Конфигурация сборки Waf
```

**28 файлов.**

---

## Основные компоненты

### AP_MotorsBlimp

Наследует `AP_Motors` из ArduPilot. Предоставляет:
- Настраиваемую матрицу смешивания 4×4 (параметры M1_ROLL..M4_THR)
- Выбор протокола моторов через `MOTOR_PWM_TYPE` (DSHOT, OneShot, PWM)
- Бидирекциональный вывод (PWM 1000-2000)
- Управление армингом/дизармингом

**4 оси управления:**

| Ось | API метод | Диапазон | Описание |
|-----|----------|----------|----------|
| Roll | `set_roll()` | [-1, +1] | Вращение вокруг X |
| Pitch | `set_pitch()` | [-1, +1] | Вращение вокруг Y |
| Yaw | `set_yaw()` | [-1, +1] | Вращение вокруг Z |
| X | `set_throttle()` | [0, +1] | Линейное вперёд/назад |

---

## Система управления

### Поток данных (Manual)

```
RC Приёмник → radio.cpp → mode_manual.cpp → AP_MotorsBlimp → Моторы
```

**AHRS, EKF, GPS и другие датчики не требуются.**

---

## Режимы полёта

| Режим | Номер | Описание |
|-------|-------|----------|
| **BRAKE** | 0 | Аварийная остановка — обнуляет все моторы |
| **MANUAL** | 1 | Прямой пропуск стиков через матрицу смешивания |

### Manual

Стики напрямую управляют матрицей смешивания:
```
CH1 (Roll)     → motors->set_roll()
CH2 (Pitch)    → motors->set_throttle()  (вперёд/назад)
CH3 (Throttle) → motors->set_pitch()     (вращение Y)
CH4 (Yaw)      → motors->set_yaw()
```

### BRAKE

Все выходы моторов обнуляются. Используется как цель failsafe.

---

## Безопасность

### Failsafe

```
Потеря RC (> 500мс)  → BRAKE
Потеря GCS            → BRAKE
Зависание main loop   → output_min()
```

### Арминг

По кнопке через AUX канал:
1. Установите AUX_FUNC=31 (ARMDISARM) на канале 5-8
2. Переключатель HIGH → арминг, LOW → дизарминг

---

## Параметры

### Матрица смешивания (MOTOR_*)

16 параметров для матрицы 4×4:

| Мотор | M_ROLL | M_PITCH | M_YAW | M_X |
|-------|--------|---------|-------|-----|
| M1 | -1..1 | -1..1 | -1..1 | -1..1 |
| M2 | -1..1 | -1..1 | -1..1 | -1..1 |
| M3 | -1..1 | -1..1 | -1..1 | -1..1 |
| M4 | -1..1 | -1..1 | -1..1 | -1..1 |

### Протокол моторов

`MOTOR_PWM_TYPE`:

| Значение | Протокол |
|----------|----------|
| 0 | Normal PWM |
| 5 | DShot300 (по умолчанию) |

---

## Система сборки

```bash
./waf configure --board MicoAir743v2
./waf blimp
```

Требуется полная структура репозитория ArduPilot с `libraries/`, `Tools/`, `modules/`.

---

## История изменений

| Дата | Версия | Описание |
|------|--------|----------|
| 2026-07-01 | 1.0 | Начальный документ |
| 2026-07-16 | 2.0 | Ветка manual-only |
| 2026-07-24 | 3.0 | AP_MotorsBlimp, удалены датчики |

**Последнее обновление:** 24 июля 2026
