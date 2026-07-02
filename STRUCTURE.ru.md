# Архитектура и структура репозитория ArduMotorBlimp

**Версия:** 1.1  
**Дата:** 1 июля 2026  
**Репозиторий:** https://github.com/DaniK-51/ArduMotorBlimp  
**Автор:** DaniK-51 (Daniyar)  
**Лицензия:** GPL-3.0

---

## 📋 Содержание

1. [Введение](#введение)
2. [Обзор проекта](#обзор-проекта)
3. [Структура репозитория](#структура-репозитория)
4. [Основные компоненты](#основные-компоненты)
5. [Система сборки](#система-сборки)
6. [Режимы полета](#режимы-полета)
7. [Система управления](#система-управления)
8. [Телеметрия и связь](#телеметрия-и-связь)
9. [Безопасность](#безопасность)
10. [Параметры](#параметры)
11. [Интеграция с ArduPilot](#интеграция-с-ardupilot)
12. [Связь с проектом the_blimp_swp](#связь-с-проектом-the_blimp_swp)

---

## Введение

### Назначение документа

Этот документ описывает архитектуру, структуру и компоненты репозитория **ArduMotorBlimp** — кастомной реализации транспортного средства типа "дирижабль" (Blimp) для платформы ArduPilot.

### Целевая аудитория

- Разработчики системы управления
- Инженеры по интеграции ArduPilot
- Члены команды проекта
- Ревьюеры кода

---

## Обзор проекта

### Что такое ArduMotorBlimp?

**ArduMotorBlimp** — это специализированная реализация дирижабля (Lighter-Than-Air vehicle) на базе официальной платформы ArduPilot. Репозиторий содержит начальное состояние официального модуля `Blimp` с кастомными доработками.

### Ключевые характеристики

| Параметр | Значение |
|----------|----------|
| **Тип аппарата** | Моторизованный дирижабль |
| **Языки** | C++ (96.5%), C (3.2%), Python (0.3%) |
| **Лицензия** | GPL-3.0 |
| **База** | Официальный репозиторий ArduPilot |
| **Статус** | Начальная разработка (Initial commit) |

### Основные возможности

- ✅ Поддержка стандартных режимов полета ArduPilot
- ✅ Интеграция с MAVLink протоколом
- ✅ Система безопасности и failsafe
- ✅ Поддержка различных типов моторных конфигураций
- ✅ Логирование телеметрии
- ✅ EKF (Extended Kalman Filter) проверка

---

## Структура репозитория

### Корневая директория

```
ArduMotorBlimp/
│
├── Основные файлы приложения
│   ├── Blimp.cpp                  # Точка входа (main loop)
│   ├── Blimp.h                    # Главный заголовочный файл
│   ├── config.h                   # Конфигурация системы
│   ├── defines.h                  # Определения констант
│   └── version.h                  # Версия прошивки
│
── Параметры и конфигурация
│   ├── Parameters.cpp             # Реализация параметров
│   ├── Parameters.h               # Объявление параметров
│   └── wscript                    # Скрипт сборки (Waf)
│
├── Режимы полета
│   ├── mode.cpp                   # Базовый класс режимов
│   ├── mode.h                     # Заголовок режимов
│   ├── mode_manual.cpp            # Ручное управление
│   ├── mode_loiter.cpp            # Удержание позиции
│   ├── mode_hold.cpp              # Удержание (Hold)
│   ├── mode_auto.cpp              # Автоматический режим
│   ├── mode_land.cpp              # Посадка
│   ├── mode_rtl.cpp               # Return-To-Launch
│   ├── mode_velocity.cpp          # Контроль скорости
│   └── Loiter.cpp                 # Логика Loiter
│       Loiter.h
│
── Система управления
│   ├── motors.cpp                 # Вывод на моторы и проверки арминга
│   ├── Fins.cpp                   # Смешивание актуаторов (frame-specific motor/servo output)
│   ├── Fins.h                     # Заголовок Fins с выходными полями и матрицей смешивания
│   ├── commands.cpp               # Обработка команд
│   ├── radio.cpp                  # RC радио управление
│   └── inertia.cpp                # Инерциальная навигация
│
├── Сенсоры и навигация
│   ├── sensors.cpp                # Чтение сенсоров
│   └── ekf_check.cpp              # Проверка EKF
│
├── Телеметрия и связь
│   ├── GCS_Blimp.cpp              # Ground Control Station
│   ├── GCS_Blimp.h
│   ├── GCS_MAVLink_Blimp.cpp      # MAVLink обработка
│   ├── GCS_MAVLink_Blimp.h
│   └── Log.cpp                    # Логирование данных
│
├── Безопасность
│   ├── AP_Arming_Blimp.cpp        # Проверки арминга
│   ├── AP_Arming_Blimp.h
│   ├── failsafe.cpp               # Failsafe логика
│   ├── events.cpp                 # Обработка событий
│   ── system.cpp                 # Системные функции
│
├── Состояние системы
│   ├── AP_State.cpp               # Состояние аппарата
│   └── AP_State.h
│
└── RC каналы
    ├── RC_Channel_Blimp.cpp       # Обработка RC каналов
    └── RC_Channel_Blimp.h
```

### Статистика файлов

| Категория | Количество файлов |
|-----------|-------------------|
| **Основные (.cpp)** | 18 файлов |
| **Заголовки (.h)** | 13 файлов |
| **Конфигурация** | 3 файла |
| **Всего** | 42 файла |

---

## Основные компоненты

### 1. Blimp.cpp / Blimp.h

**Назначение:** Точка входа и главный цикл приложения

**Основные функции:**
```cpp
// Главный класс дирижабля
class Blimp : public AP_HAL::HAL::Callbacks {
public:
    // Инициализация системы
    void init() override;
    
    // Главный цикл (вызывается 1000 раз в секунду)
    void loop() override;
    
    // Получение единственного экземпляра
    static Blimp& get_instance();
    
private:
    // Внутренние методы
    void update_loop();
    void read_sensors();
    void update_control();
};
```

**Жизненный цикл:**
```
init() → Настройка всех подсистем
   ↓
loop() → Главный цикл
   ↓
1. read_sensors()
2. update_EKF()
3. update_control()
4. update_motors()
5. send_telemetry()
6. logger.write()
   ↓
[Повтор 1000 Hz]
```

### 2. Parameters.cpp / Parameters.h

**Назначение:** Система параметров для настройки поведения

**Пример параметров:**
```cpp
// Объявление параметров
const AP_Param::GroupInfo Parameters::var_info[] = {
    // Режимы полета
    AP_GROUPINFO("RTL_ALT", 1, Parameters, rtl_altitude, 1500),
    
    // Моторы
    AP_GROUPINFO("MOT_MAX", 2, Parameters, motor_max, 2000),
    AP_GROUPINFO("MOT_MIN", 3, Parameters, motor_min, 1000),
    
    // Безопасность
    AP_GROUPINFO("ARMING_CHECK", 4, Parameters, arming_check, 1),
    
    AP_GROUPEND
};
```

**Категории параметров:**
- **Blimp_** — специфичные параметры дирижабля
- **MOT_** — настройки моторов
- **RTL_** — параметры возврата домой
- **ARMING_** — проверки арминга
- **LOG_** — настройки логирования

### 3. mode.cpp / mode.h

**Назначение:** Базовый класс для всех режимов полета

**Иерархия режимов:**
```
Mode (базовый класс)
   ├── ModeManual        → Ручное управление
   ├── ModeLoiter        → Удержание позиции
   ├── ModeHold          → Удержание точки
   ├── ModeAuto          → Автоматическая миссия
   ├── ModeLand          → Посадка
   ├── ModeRTL           → Возврат домой
   └── ModeVelocity      → Контроль скорости
```

**Интерфейс режима:**
```cpp
class Mode {
public:
    // Инициализация режима
    virtual bool init(bool ignore_checks) = 0;
    
    // Основной цикл режима
    virtual void run() = 0;
    
    // Проверка доступности режима
    virtual bool requires_GPS() const = 0;
    
    // Получение названия
    virtual const char* name() const = 0;
    
protected:
    // Общие методы
    void set_desired_velocity(float vx, float vy, float vz);
    void set_desired_position(float x, float y, float z);
};
```

### 4. motors.cpp

**Назначение:** Вывод на моторы и проверки арминга

**Ключевые функции:**
- `motors_output()` — отправляет финальные PWM-сигналы на серво/моторы через `SRV_Channels`
- `arm_motors_check()` — логика арминга/дизарминга
- Вызывает `motors->output()`, который делегирует работу классу Fins

### 5. Fins.cpp / Fins.h

**Назначение:** Смешивание актуаторов — преобразует выходы управления в сигналы для каждого мотора

**Выходные поля (интерфейс от Loiter):**
```cpp
float forward_out;  // движение вперёд, -1 до +1
float roll_out;     // вращение roll, -1 до +1
float pitch_out;    // вращение pitch, -1 до +1
float yaw_out;      // вращение yaw, -1 до +1
```

**Поддерживаемые типы фреймов:**
| Фрейм | Enum | Описание |
|-------|------|----------|
| `FISHBLIMP` | 1 | 4 серво ластовиков (Back, Front, Right, Left) — синусоидальное движение |
| `FOUR_MOTOR` | 2 | 4 мотора (FrontLeft, FrontRight, Up, Right) — линейное смешивание |
| `ROTARY_BLIMP` | 3 | 4 мотора — forward + 3 вращательные оси |

**Матрица смешивания:** Каждый мотор имеет коэффициенты `_amp_factor`, определяющие какую долю каждого выхода он использует:
```cpp
_thrpos[i] = forward_out * _forward_amp_factor[i]
           + roll_out    * _roll_amp_factor[i]
           + pitch_out   * _pitch_amp_factor[i]
           + yaw_out     * _yaw_amp_factor[i];
```

**Ключевые методы:**
- `setup_rotary()` / `setup_motors()` / `setup_fins()` — определяют матрицу смешивания для каждого фрейма
- `output_rotary()` / `output_motors()` / `output_fins()` — применяют смешивание и отправляют в SRV_Channels
- `output_min()` — обнуляет все выходы
- `get_throttle()` — макс. абсолютное значение по всем осям (индикатор для MAVLink)

### 6. GCS_Blimp.cpp / GCS_MAVLink_Blimp.cpp

**Назначение:** Связь с наземной станцией (Ground Control Station)

**Протоколы:**
- **MAVLink** — основной протокол обмена данными
- **UDP/TCP** — транспортные протоколы

**Передаваемые данные:**
- Позиция (GPS координаты)
- Ориентация (roll, pitch, yaw)
- Скорость и высота
- Состояние батареи
- Статус моторов
- Телеметрия сенсоров

**Принимаемые команды:**
- Команды управления
- Изменение параметров
- Загрузка миссий
- Команды арминга/дизарминга

---

## Система сборки

### Waf build system

Проект использует **Waf** — Python-based build system, стандартную для ArduPilot.

### Файл wscript

```python
def build(bld):
    # Имя транспортного средства
    vehicle = bld.path.name
    
    # Создание статической библиотеки
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'AC_InputManager',
            'AP_Avoidance',
            'AP_LTM_Telem',
            'AP_Devo_Telem',
            'AP_KDECAN',
            'AP_AdvancedFailsafe',   # TODO по какой-то причине компиляция GCS_Common.cpp (в libraries) падает без этого
            'AC_AttitudeControl',    # для логирования PSCx
        ],
    )
    
    # Создание исполняемого файла
    bld.ap_program(
        program_name='blimp',
        program_groups=['bin', 'blimp'],
        use=vehicle + '_libs',
    )
```

### Процесс сборки

```bash
# 1. Клонирование репозитория
git clone https://github.com/DaniK-51/ArduMotorBlimp.git
cd ArduMotorBlimp

# 2. Интеграция с ArduPilot
# (обычно копируется в папку ArduPilot/Blimp/)

# 3. Конфигурация
./waf configure --board sitl

# 4. Сборка
./waf blimp

# 5. Результат
./build/sitl/bin/blimp
```

### Зависимости

**Общие библиотеки ArduPilot:**
- AP_Common — общие утилиты
- AP_HAL — Hardware Abstraction Layer
- AP_Math — математика
- AP_Param — система параметров
- AP_Scheduler — планировщик

**Специфичные библиотеки:**
- AP_AHRS — система ориентации
- AP_NavEKF3 — фильтр Калмана
- AP_Motors — управление моторами
- RC_Channel — RC каналы
- GCS_MAVLink — MAVLink протокол
- AP_Logger — логирование

---

## Режимы полета

### 1. Manual (Ручное управление)

**Файл:** `mode_manual.cpp`

**Описание:** Прямое управление моторами через RC пульт

**Характеристики:**
- Нет стабилизации
- Полный контроль пилота
- Требует постоянного внимания

### 2. Loiter (Удержание позиции)

**Файл:** `mode_loiter.cpp`, `Loiter.cpp`

**Описание:** Автоматическое удержание текущей позиции

**Характеристики:**
- Использует GPS для позиционирования
- PID контроллер для коррекции дрейфа
- Компенсация ветра

**Алгоритм:**
```
1. Запоминаем текущую позицию (GPS)
2. Считываем текущую позицию
3. Вычисляем ошибку (desired - current)
4. PID контроллер → корректировка скорости
5. Отправка команд моторам
```

### 3. Hold (Удержание)

**Файл:** `mode_hold.cpp`

**Описание:** Удержание текущей позиции и высоты

**Отличие от Loiter:**
- Более строгое удержание
- Меньше допустимая ошибка
- Использует барометр для высоты

### 4. Auto (Автоматический)

**Файл:** `mode_auto.cpp`

**Описание:** Выполнение заранее заданной миссии

**Возможности:**
- Следование по точкам (waypoints)
- Выполнение команд (взлет, посадка, задержка)
- Автоматический возврат при потере связи

### 5. Land (Посадка)

**Файл:** `mode_land.cpp`

**Описание:** Автоматическая посадка

**Алгоритм:**
```
1. Снижение с контролируемой скоростью
2. Удержание горизонтальной позиции
3. Обнаружение земли (range finder)
4. Отключение моторов после посадки
```

### 6. RTL (Return-To-Launch)

**Файл:** `mode_rtl.cpp`

**Описание:** Автоматический возврат в точку взлета

**Последовательность:**
```
1. Подъем на безопасную высоту
2. Полет к точке запуска (GPS)
3. Снижение и посадка
```

### 7. Velocity (Контроль скорости)

**Файл:** `mode_velocity.cpp`

**Описание:** Управление через задание скорости

**Применение:**
- Точное управление движением
- Интеграция с внешними системами
- Компенсация ветра

---

## Система управления

### Архитектура управления

```
┌─────────────────────────────────────────────────────────────┐
│  Пилот / GCS                                                │
│  (RC каналы / MAVLink команды)                              │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  Mode (текущий режим полета)                                │
│  • ModeManual    → прямой пилотаж                           │
│  • ModeVelocity  → целевая скорость → Loiter.run_vel()      │
│  • ModeLoiter    → целевая позиция  → Loiter.run()          │
│  • ModeAuto      → миссия (SCurve)  → Loiter.run()         │
│  • ModeRTL       → возврат домой    → Loiter.run()          │
│  • ModeLand      → посадка          → Loiter.run_vel()      │
│  • ModeHold      → полная остановка (все выходы = 0)        │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  Loiter (каскадный PID-контроллер)                          │
│                                                             │
│  ┌───────────────────┐     ┌───────────────────┐            │
│  │  Position PID      │────▶│  Velocity PID      │           │
│  │  pid_pos_{x,y,z,   │     │  pid_vel_{x,y,z,   │           │
│  │        yaw}        │     │        yaw}        │           │
│  │  pos → vel         │     │  vel → force       │           │
│  └───────────────────┘     └────────┬──────────┘            │
│                                     │                        │
│  Scaler: компенсация связи осей    │                        │
│  (если motor0 используется и для   │                        │
│   forward, и для pitch — scaler    │                        │
│   уменьшает вклад каждой оси)      │                        │
└─────────────────────────────────────┼──────────────────────┘
                                      │
              motors->{forward_out, roll_out, pitch_out, yaw_out}
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────┐
│  Fins (смешивание актуаторов)                               │
│                                                             │
│  Берёт 4 числа и матрично смешивает в сигналы моторов:     │
│                                                             │
│  _thrpos[i] = forward_out * _forward_amp_factor[i]          │
│             + roll_out    * _roll_amp_factor[i]             │
│             + pitch_out   * _pitch_amp_factor[i]            │
│             + yaw_out     * _yaw_amp_factor[i]              │
│                                                             │
│  Фреймы: FISHBLIMP (ластовики), FOUR_MOTOR, ROTARY_BLIMP    │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  Моторы / Серво                                             │
│  (PWM сигналы через SRV_Channels)                           │
└─────────────────────────────────────────────────────────────┘
```

### Разделение ответственности

**Loiter** — не знает как устроены моторы. Пишет 4 числа:
`forward_out` (вперёд), `roll_out` (крен), `pitch_out` (тангаж), `yaw_out` (рыскание).

**Fins** — не знает откуда пришли эти числа. Берёт 4 числа и через матрицу `_amp_factor` преобразует в сигналы для каждого мотора/серво.

Благодаря этому разделению Fins interface можно менять не трогая PID, и наоборот.

### Каскад PID (Loiter)

Два режима работы:

1. **Полный каскад** (`run()`): позиция → скорость → моторы. Используется в Loiter, RTL, Auto.
2. **Только скорость** (`run_vel()`): скорость → моторы. Используется в Velocity, Land.

### Параметры Loiter

| Параметр | Назначение |
|----------|------------|
| `LOIT_VEL{X,Y,Z,YAW}_{P,I,D}` | PID скорости по каждой оси |
| `LOIT_POS{X,Y,Z,YAW}_{P,I,D}` | PID позиции по каждой оси |
| `LOIT_MAX_VEL{X,Y,Z,YAW}` | Максимальная скорость |
| `LOIT_MAX_POS{X,Y,Z,YAW}` | Максимальная скорость изменения позиции |
| `LOIT_DIS_MASK` | Отключение осей (битовая маска) |
| `LOIT_PID_DZ` | Мёртвая зона позиционного PID (м) |
| `LOIT_POS_LAG` | Допустимое отставание целевой позиции (с) |

### Примеры работы PID

Все PID-контроллеры реализованы в классе `Loiter` через библиотеку `AC_PID`.

**Position PID (позиция → скорость):**
```cpp
// Loiter::run()
target_vel_ef.x = pid_pos_x.update_all(target_pos.x, blimp.pos_ned.x, dt, limit.x);
target_vel_ef.y = pid_pos_y.update_all(target_pos.y, blimp.pos_ned.y, dt, limit.y);
target_vel_ef.z = pid_pos_z.update_all(target_pos.z, blimp.pos_ned.z, dt, limit.z);
target_vel_yaw  = pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw_ef), dt, limit.yaw);
```

**Velocity PID (скорость → сила/момент):**
```cpp
// Loiter::run_vel()
actuator.x = pid_vel_x.update_all(target_vel_bf_c.x * scaler_x, vel_bf_filtd.x, dt, limit.x);
actuator.y = pid_vel_y.update_all(target_vel_bf_c.y * scaler_y, vel_bf_filtd.y, dt, limit.y);
act_down   = pid_vel_z.update_all(target_vel_bf_c.z * scaler_z, vel_bf_filtd.z, dt, limit.z);
act_yaw    = pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yaw, blimp.vel_yaw_filtd, dt, limit.yaw);
```

**Прямой пилотаж (Manual mode, без PID):**
```cpp
// ModeManual::run()
motors->forward_out = pilot.x * g.max_man_thr;
motors->roll_out    = pilot.y * g.max_man_thr;
motors->pitch_out   = pilot.z * g.max_man_thr;
motors->yaw_out     = pilot_yaw * g.max_man_thr;
```

---

## Телеметрия и связь

### MAVLink протокол

**Основные сообщения (отправка):**

| Сообщение | Частота | Описание |
|-----------|---------|----------|
| **HEARTBEAT** | 1 Hz | Статус системы |
| **ATTITUDE** | 10-50 Hz | Roll, pitch, yaw |
| **GLOBAL_POSITION_INT** | 10 Hz | GPS позиция |
| **SYS_STATUS** | 1 Hz | Состояние системы |
| **VFR_HUD** | 5 Hz | Скорость, высота |
| **RC_CHANNELS** | 5 Hz | RC каналы |
| **SERVO_OUTPUT_RAW** | 5 Hz | PWM выходы |

**Основные команды (прием):**

| Команда | Описание |
|---------|----------|
| **COMMAND_LONG** | Длинные команды (взлет, RTL) |
| **SET_POSITION_TARGET_GLOBAL_INT** | Целевая позиция |
| **SET_ATTITUDE_TARGET** | Целевая ориентация |
| **PARAM_SET** | Установка параметра |
| **MISSION_ITEM** | Элемент миссии |

### GCS_Blimp

**Функции:**
```cpp
// Отправка телеметрии
void GCS_Blimp::send_telemetry() {
    send_heartbeat();
    send_attitude();
    send_position();
    send_battery_status();
    send_rc_channels();
}

// Обработка команд
void GCS_Blimp::handle_message(mavlink_message_t& msg) {
    switch(msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handle_command(msg);
            break;
        case MAVLINK_MSG_ID_PARAM_SET:
            handle_param_set(msg);
            break;
        // ... другие команды
    }
}
```

### Логирование (Log.cpp)

**Типы логов:**
- **ATT** — ориентация (attitude)
- **POS** — позиция (position)
- **MOT** — моторы (motors)
- **BAT** — батарея (battery)
- **GPS** — GPS данные
- **IMU** — IMU данные
- **CMD** — команды

**Формат:**
```cpp
// Запись лога
struct Log_Attitude {
    float roll;
    float pitch;
    float yaw;
    float roll_desired;
    float pitch_desired;
};

logger.Write("ATT", "roll,pitch,yaw,roll_d,pitch_d", 
             "fff,ff", 
             attitude.roll, 
             attitude.pitch, 
             attitude.yaw,
             attitude.roll_desired,
             attitude.pitch_desired);
```

---

## Безопасность

### AP_Arming_Blimp

**Проверки перед армингом:**

```cpp
bool AP_Arming_Blimp::pre_arm_checks() {
    bool success = true;
    
    // 1. Проверка GPS
    if (!gps_ok()) {
        gcs().send_text("PreArm: GPS not ready");
        success = false;
    }
    
    // 2. Проверка EKF
    if (!ekf_ok()) {
        gcs().send_text("PreArm: EKF not ready");
        success = false;
    }
    
    // 3. Проверка батареи
    if (battery_voltage < min_voltage) {
        gcs().send_text("PreArm: Battery too low");
        success = false;
    }
    
    // 4. Проверка RC калибровки
    if (!rc_calibrated()) {
        gcs().send_text("PreArm: RC not calibrated");
        success = false;
    }
    
    // 5. Проверка сенсоров
    if (!sensors_ok()) {
        gcs().send_text("PreArm: Sensors not healthy");
        success = false;
    }
    
    return success;
}
```

### Failsafe

**Сценарии failsafe:**

1. **Потеря RC связи:**
   ```
   1. Обнаружение потери сигнала (> 500ms)
   2. Переключение в RTL режим
   3. Возврат домой и посадка
   ```

2. **Низкий заряд батареи:**
   ```
   1. Предупреждение (warning level)
   2. Критическое предупреждение (critical level)
   3. Автоматическая посадка (emergency level)
   ```

3. **GPS потеря:**
   ```
   1. Переключение в режим без GPS
   2. Удержание позиции по барометру
   3. Предупреждение пилота
   ```

4. **EKF ошибка:**
   ```
   1. Обнаружение расхождения EKF
   2. Переключение на резервный EKF
   3. Если не помогает — посадка
   ```

### EKF Check (ekf_check.cpp)

**Проверки EKF:**
```cpp
bool EKF_Check::healthy() {
    // Проверка дисперсии позиции
    if (pos_variance > threshold) {
        return false;
    }
    
    // Проверка дисперсии скорости
    if (vel_variance > threshold) {
        return false;
    }
    
    // Проверка сходимости
    if (!ekf_converged()) {
        return false;
    }
    
    return true;
}
```

---

## Параметры

### Категории параметров

**Blimp параметры:**
```cpp
// Конфигурация
Blimp_TYPE           // Тип дирижабля
Blimp_ENABLE         // Включение системы
Blimp_OPTIONS        // Опции

// Моторы
Blimp_MOT_MAX        // Максимальный PWM
Blimp_MOT_MIN        // Минимальный PWM
Blimp_MOT_IDLE       // Холостой ход
Blimp_MOT_SPIN_MIN   // Минимальные обороты

// Управление
Blimp_RTL_ALT        // Высота возврата (см)
Blimp_LAND_SPEED       // Скорость посадки
Blimp_LOITER_SPEED     // Скорость в Loiter

// Безопасность
Blimp_FS_ENABLE      // Включение failsafe
Blimp_FS_THR_ENABLE  // Failsafe по газу
Blimp_FS_THR_VALUE   // Порог газа
```

### Загрузка/сохранение параметров

```bash
# В MAVProxy
param load blimp_params.parm    # Загрузить параметры
param save blimp_params.parm    # Сохранить параметры
param show                      # Показать все параметры
param set Blimp_RTL_ALT 2000    # Установить параметр
param diff                      # Показать отличия от дефолта
```

---

## Интеграция с ArduPilot

### Связь с основным репозиторием

**ArduMotorBlimp** — это **форк/модуль** официального репозитория ArduPilot.

**Структура интеграции:**
```
ardupilot/                    # Основной репозиторий ArduPilot
└── Blimp/                    # Наш модуль (копия ArduMotorBlimp)
    ├── Blimp.cpp
    ├── mode.cpp
    └── ...
    
libraries/                    # Общие библиотеки
├── AP_Motors/
├── AP_AHRS/
├── AP_NavEKF3/
└── ...
```

### Отличия от стандартного Blimp

**ArduMotorBlimp** содержит:
- Начальную реализацию дирижабля
- Кастомные режимы полета
- Fins interface с 3 типами фреймов (FISHBLIMP, FOUR_MOTOR, ROTARY_BLIMP)
- Каскадный PID-контроллер (Loiter) с позиционными/скоростными PID и scaler осей
- Расширенные проверки безопасности

### Процесс обновления

```bash
# Синхронизация с ArduPilot
git remote add ardupilot https://github.com/ArduPilot/ardupilot.git
git fetch ardupilot
git merge ardupilot/master
git push origin main
```

---

## Связь с проектом the_blimp_swp

### Архитектурная связь

```
┌─────────────────────────────────────────────────────────────┐
│  the_blimp_swp                                              │
│  (репозиторий команды)                                      │
│                                                             │
│  ├── sitl/                      # Docker + SITL             │
│  │   ├── Dockerfile                                         │
│  │   ├── docker-compose.yml                                 │
│  │   ├── params/blimp.parm                                  │
│  │   └── scripts/blimp_motor_control.lua                    │
│  │                                                          │
│  └── docs/                      # Документация              │
│      └── REPOSITORY_STRUCTURE.md                            │
└────────────────────┬────────────────────────────────────────┘
                     │ Тестирует и использует
                     ▼
┌─────────────────────────────────────────────────────────────┐
│  ArduMotorBlimp                                             │
│  (репозиторий прошивки)                                     │
│                                                             │
│  ├── Blimp.cpp                  # Main application          │
│  ├── mode.cpp                   # Flight modes              │
│  ├── motors.cpp                 # Motor output              │
│  └── ...                        # Все компоненты            │
└────────────────────┬────────────────────────────────────────┘
                     │ Основан на
                     ▼
┌─────────────────────────────────────────────────────────────┐
│  ArduPilot                                                  │
│  (официальный репозиторий)                                  │
│                                                             │
│  ├── libraries/                 # Общие библиотеки          │
│  ├── Tools/                     # Инструменты сборки        │
│  └── wscript                    # Build system              │
└─────────────────────────────────────────────────────────────┘
```

---

## Полезные команды

### Сборка и запуск

```bash
# Сборка для SITL
./waf configure --board sitl
./waf blimp

# Запуск
./build/sitl/bin/blimp --model +

# С кастомными параметрами
./build/sitl/bin/blimp --model + --add-param-file=blimp.parm
```

### Отладка

```bash
# Просмотр логов
mavlog.py *.bin

# Анализ параметров
param.py show

# Проверка состояния
mavproxy.py --master=127.0.0.1:14550
```

### Git workflow

```bash
# Создание ветки
git checkout -b feature/new-mode

# Внесение изменений
git add .
git commit -m "feat: add new flight mode"

# Отправка
git push origin feature/new-mode

# Создание PR
# (на GitHub)
```

---

## Приложения

### A. Словарь терминов

| Термин | Определение |
|--------|-------------|
| **Arming** | Включение моторов (подготовка к полету) |
| **Disarming** | Выключение моторов |
| **Failsafe** | Автоматическая реакция на аварийную ситуацию |
| **EKF** | Extended Kalman Filter — фильтр для навигации |
| **GCS** | Ground Control Station — наземная станция |
| **Loiter** | Удержание позиции |
| **RTL** | Return-To-Launch — возврат домой |
| **PWM** | Pulse Width Modulation — управление моторами |
| **RC** | Radio Control — радио управление |
| **MAVLink** | Протокол связи с автопилотом |
| **SITL** | Software In The Loop — симуляция |
| **PID** | Proportional-Integral-Derivative — регулятор |

### B. Полезные ссылки

**Официальные ресурсы:**
- [ArduPilot.org](https://ardupilot.org/)
- [Developer Wiki](https://ardupilot.org/dev/)
- [ArduMotorBlimp GitHub](https://github.com/DaniK-51/ArduMotorBlimp)
- [Discord](https://discord.com/invite/ardupilot)

**Документация:**
- [Building ArduPilot](https://ardupilot.org/dev/docs/building-the-code.html)
- [Parameters List](https://ardupilot.org/copter/docs/parameters.html)
- [MAVLink Protocol](https://mavlink.io/)

### C. Чеклист перед коммитом

- [ ] Код компилируется без ошибок
- [ ] Нет предупреждений компилятора
- [ ] Все тесты проходят
- [ ] Документация обновлена
- [ ] Параметры задокументированы
- [ ] Лицензия соблюдается (GPL-3.0)

---

## История изменений

| Дата | Версия | Описание | Автор |
|------|--------|----------|-------|
| 2026-07-01 | 1.0 | Начальная версия документа | Команда Blimp |
| 2026-07-01 | 1.1 | Убраны разделы Workflow и План развития | Команда Blimp |

---

**Этот документ будет обновляться по мере развития проекта ArduMotorBlimp.**

**Последнее обновление:** 1 июля 2026
