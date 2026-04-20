from micropython import const
from machine import I2C, Pin
from sensor_pack_2.bus_service import I2cAdapter
import opt3001mod
import time


SINGLE_SHOT_LUX_RANGE_INDEX = const(10)
AUTO_SHOT_LUX_RANGE_INDEX = const(12)

# --- Настройка прерываний ---
GPIO_NUM = const(3)  # Подключите вывод INT датчика OPT3001 к GPIOX (где X-свободный вывод GPIO)
# для подтяжки использую встроенный в МК резистор и подключаю его к VCC (Pin.PULL_UP)
int_pin = Pin(GPIO_NUM, Pin.IN, Pin.PULL_UP)
int_triggered = False
irq_3001_handled: int = 0

def int_opt3001_handler(pin):
    """Обработчик прерывания. Минимум логики, только установка флага."""
    global int_triggered, irq_3001_handled
    int_triggered = True
    irq_3001_handled += 1

def show_header(info: str, width: int = 32):
    print(width * "-")
    print(info)
    print(width * "-")

def show_meas_val(data) -> None:
    """Выводит результат измерения OPT3001 в читаемом виде."""
    if data is None or data.lux is None:
        print("Данные не готовы, переполнение или ошибка экспоненты!")
    else:
        print(f"{data.lux:.2f} lux (FSR: {data.full_scale_range:.2f} lux)")

def delay_ms(val: int):
    time.sleep_ms(val)

ID_I2C = const(1)
SDA_PIN_N = const(6)
SCL_PIN_N = const(7)
FREQ_I2C = const(400_000)

if __name__ == '__main__':
    i2c = I2C(id=ID_I2C, scl=Pin(SCL_PIN_N), sda=Pin(SDA_PIN_N), freq=FREQ_I2C)  # on Raspberry Pi Pico
    adapter = I2cAdapter(i2c)

    als = opt3001mod.OPT3001(adapter)
    _id = als.get_id()
    print(f"Manufacturer id: 0x{_id.manufacturer_id:x}; device id: 0x{_id.device_id:x}")

    show_header("Настройка прерываний: окно X-Y люкс")
    als.set_limits(low_lux=100.0, high_lux=500.0)
    als.fault_count_index = 3  # 8 последовательных событий
    als.latch = True  # режим защелки
    als.polarity = False  # active-low INT
    print(f"Пороги: {als.get_limits()} lux")

    show_header("Однократный режим работы! Запуск измерения вручную! Автоматический выбор предела измерения датчиком!")
    als.long_conversion_time = False
    als.start_measurement(continuously=False, lx_range_index=SINGLE_SHOT_LUX_RANGE_INDEX, refresh=False)
    cycle_time = als.get_conversion_cycle_time()
    print(f"cycle time мс: {cycle_time}")
    print(als.read_config_from_sensor(return_value=True))
    repeat_count = 11
    for _ in range(repeat_count):
        delay_ms(cycle_time)
        ds = als.get_data_status()
        if ds.conversion_ready:
            # обработанные данные!
            value = als.get_measurement_value(value_index=1)
            show_meas_val(value)
        else:
            print(f"Данные не готовы для считывания!")
        # запуск измерения вручную
        als.start_measurement(continuously=False, lx_range_index=SINGLE_SHOT_LUX_RANGE_INDEX, refresh=False)

    show_header("Автоматический запуск измерения! Автоматический выбор предела измерения датчиком!")
    als.long_conversion_time = True
    als.start_measurement(continuously=True, lx_range_index=AUTO_SHOT_LUX_RANGE_INDEX, refresh=False)
    print(als.read_config_from_sensor(return_value=True))
    cycle_time = als.get_conversion_cycle_time()
    print(f"cycle time мс: {cycle_time}; увеличенное время преобразования: {als.long_conversion_time}")
    for _ in range(repeat_count):
        delay_ms(cycle_time)
        ds = als.get_data_status()
        if ds.conversion_ready:
            # обработанные данные!
            value = als.get_measurement_value(value_index=1)
            show_meas_val(value)
        else:
            print(f"Данные не готовы для считывания!")

    # Конфигурация датчика для оконного компаратора
    als.set_limits(low_lux=50.0, high_lux=350.0)
    als.fault_count_index = 1  # 2^1 = 2 последовательных события
    als.latch = True  # Защёлкнутый режим (L=1) → INT сбрасывается чтением конфига
    als.polarity = False  # Active-low (POL=0) → срабатывание на спадающий фронт
    als.long_conversion_time = False  # 100 мс для быстрой реакции в тесте
    als.start_measurement(continuously=True, lx_range_index=12, refresh=False)

    # Регистрация прерывания
    int_pin.irq(trigger=Pin.IRQ_FALLING, handler=int_opt3001_handler)

    wl = als.get_limits()
    show_header(f"Ожидание прерываний на GPIO{GPIO_NUM}...")
    print(f"Поднесите/уберите источник света, чтобы выйти за пределы окна {wl[0]}–{wl[1]} lux")

    MAX_IRQ_HANDLED = const(10)
    curr_handled = 0

    try:
        while True:
            if int_triggered:
                int_triggered = False

                # Чтение результата
                # Сначала читаю данные (если нужно знать, что именно вызвало прерывание)
                val = als.get_measurement_value(value_index=1)
                show_meas_val(val)
                # в latched-режиме чтение регистра конфигурации
                # Сбрасываю latch/INT чтением конфига (Table 2)
                _ = als.get_cfg_reg()

                # диагностика
                ds = als.get_data_status()
                if ds.overflow:
                    print("Переполнение: освещённость > выбранного диапазона")
                elif ds.conversion_ready:
                    # В continuous-режиме CRF быстро снова станет True
                    pass
            # пауза между проверками флага
            time.sleep_ms(10)
            #
            if irq_3001_handled != curr_handled:
                print(f"IRQ handled: {irq_3001_handled}")
                curr_handled = irq_3001_handled
            #
            if irq_3001_handled > MAX_IRQ_HANDLED:
                break

    except KeyboardInterrupt:
        print("\nТест прерываний завершён пользователем.")
        print(f"\nОбработано прерываний от OPT3001: {irq_3001_handled}.")
    finally:
        # отключение прерывания
        int_pin.irq(handler=None)