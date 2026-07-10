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
│   ├── mode_land.cpp              # Посадка
│   ├── mode_rtl.cpp               # Return-To-Launch
│   ├── mode_velocity.cpp          # Контроль скорости
│   ├── Loiter.cpp                 # Логика Loiter
│   └── Loiter.h
│
── Система управления
│   ├── motors.cpp                 # Управление моторами
│   ├── Fins.cpp                   # Управление плавниками/рулями
│   ├── Fins.h                     # Заголовок Fins
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
│   ├── GCS_Mavlink.cpp            # MAVLink обработка
│   ├── GCS_Mavlink.h
│   └── Log.cpp                    # Логирование данных
│
├── Безопасность
│   ├── AP_Arming.cpp              # Проверки арминга
│   ├── AP_Arming.h
│   ├── failsafe.cpp               # Failsafe логика
│   ├── events.cpp                 # Обработка событий
│   ── system.cpp                 # Системные функции
│
├── Состояние системы
│   └── AP_State.cpp               # Состояние аппарата
│
├── RC каналы
│   ├── RC_Channel.cpp             # Обработка RC каналов
│   └── RC_Channel.h
│
└── Документация
    ├── README.md                  # Описание проекта
    ├── COPYING.txt                # Лицензия (GPL-3.0)
    ├── AGENTS.md                  # Руководство для AI
    ├── STRUCTURE.md               # Этот документ (англ.)
    └── STRUCTURE.ru.md            # Этот документ (рус.)
```

### Статистика файлов

| Категория | Количество файлов |
|-----------|-------------------|
| **Основные (.cpp)** | 25 файлов |
| **Заголовки (.h)** | 12 файлов |
| **Конфигурация** | 1 файл |
| **Документация** | 5 файлов |
| **Всего** | 43 файла |

---

## Основные компоненты

### 1. Blimp.cpp / Blimp.h

**Назначение:** Точка входа и главный цикл приложения

**Основные функции:**
```cpp
// Главный класс дирижабля
class Blimp : public AP_Vehicle {
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

**Назначение:** Управление моторами дирижабля

**Особенности:**
- Поддержка различных конфигураций моторов
- Смешивание сигналов управления
- Ограничение PWM диапазонов
- Компенсация напряжения батареи

**Пример использования:**
```cpp
// Инициализация моторов
motors->init();

// Установка значений
motors->set_roll(roll_input);
motors->set_pitch(pitch_input);
motors->set_yaw(yaw_input);
motors->set_throttle(throttle_input);

// Вывод на моторы
motors->output();
```

### 5. Fins.cpp / Fins.h

**Назначение:** Управление плавниками/рулями дирижабля

**Функции:**
- Управление аэродинамическими поверхностями
- Стабилизация курса
- Компенсация ветра

### 6. GCS_Blimp.cpp / GCS_Mavlink.cpp

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
            'AC_InputManager',        # Управление входами
            'AP_InertialNav',         # Инерциальная навигация
            'AP_Avoidance',           # Избегание препятствий
            'AP_LTM_Telem',          # LTM телеметрия
            'AP_Devo_Telem',         # Devo телеметрия
            'AP_KDECAN',             # KDECAN поддержка
            'AP_AdvancedFailsafe',   # Расширенный failsafe
            'AC_AttitudeControl',    # Контроль ориентации
        ],
    )
    
    # Создание исполняемого файла
    bld.ap_program(
        program_name='ardublimp',
        program_groups=['bin', 'ardumotorblimp'],
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
./build/sitl/bin/ardublimp
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

### 3. Land (Посадка)

**Файл:** `mode_land.cpp`

**Описание:** Автоматическая посадка

**Алгоритм:**
```
1. Снижение с контролируемой скоростью
2. Удержание горизонтальной позиции
3. Обнаружение земли (range finder)
4. Отключение моторов после посадки
```

### 4. RTL (Return-To-Launch)

**Файл:** `mode_rtl.cpp`

**Описание:** Автоматический возврат в точку взлета

**Последовательность:**
```
1. Подъем на безопасную высоту
2. Полет к точке запуска (GPS)
3. Снижение и посадка
```

### 5. Velocity (Контроль скорости)

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
┌─────────────────────────────────────────────────────────
│  Пилот / GCS                                            │
│  (RC каналы / MAVLink команды)                          │
└──────────────────┬──────────────────────────────────────┘
                   ▼
─────────────────────────────────────────────────────────┐
│  RC_Channel                                                  │
│  • Чтение RC каналов                                    │
│  • Фильтрация сигналов                                  │
│  • Проверка failsafe                                    │
└──────────────────┬──────────────────────────────────────┘
                   ▼
┌─────────────────────────────────────────────────────────┐
│  Mode (текущий режим полета)                            │
│  • Обработка команд                                     │
│  • Генерация целевых значений                           │
└──────────────────┬──────────────────────────────────────┘
                   ▼
┌─────────────────────────────────────────────────────────┐
│  AC_AttitudeControl / AC_PositionControl                │
│  • PID контроллер ориентации                            │
│  • PID контроллер позиции                               │
│  • PID контроллер скорости                              │
└──────────────────┬──────────────────────────────────────┘
                   ▼
┌─────────────────────────────────────────────────────────┐
│  motors.cpp / Fins.cpp                                  │
│  • Смешивание сигналов                                  │
│  • Ограничение диапазонов                               │
│  • Компенсация батареи                                  │
└──────────────────┬──────────────────────────────────────┘
                   ▼
┌─────────────────────────────────────────────────────────┐
│  Моторы / Плавники                                      │
│  (PWM сигналы 1000-2000 мкс)                            │
─────────────────────────────────────────────────────────┘
```

### PID контроллеры

**Attitude Control (ориентация):**
```cpp
// Roll контроллер
roll_error = desired_roll - current_roll;
roll_output = Kp * roll_error + Ki * integral + Kd * derivative;

// Pitch контроллер
pitch_error = desired_pitch - current_pitch;
pitch_output = Kp * pitch_error + Ki * integral + Kd * derivative;

// Yaw контроллер
yaw_error = desired_yaw - current_yaw;
yaw_output = Kp * yaw_error + Ki * integral;
```

**Position Control (позиция):**
```cpp
// Горизонтальная позиция
pos_error_x = desired_x - current_x;
vel_desired_x = Kp * pos_error_x;

pos_error_y = desired_y - current_y;
vel_desired_y = Kp * pos_error_y;

// Вертикальная позиция (высота)
alt_error = desired_alt - current_alt;
vel_desired_z = Kp * alt_error;
```

**Velocity Control (скорость):**
```cpp
vel_error_x = desired_vel_x - current_vel_x;
accel_output_x = Kp * vel_error_x + Ki * integral;

vel_error_y = desired_vel_y - current_vel_y;
accel_output_y = Kp * vel_error_y + Ki * integral;
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

### AP_Arming

**Проверки перед армингом:**

```cpp
bool AP_Arming::pre_arm_checks() {
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
- ✅ Начальную реализацию дирижабля
- ✅ Кастомные режимы полета
- ✅ Специфичную логику управления моторами
- ✅ Интеграцию с Fins (плавниками)
- ✅ Расширенные проверки безопасности

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
│  ├── motors.cpp                 # Motor control             │
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
./build/sitl/bin/ardublimp --model +

# С кастомными параметрами
./build/sitl/bin/ardublimp --model + --add-param-file=blimp.parm
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
