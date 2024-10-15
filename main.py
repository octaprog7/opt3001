# import sys
from machine import I2C, Pin
from sensor_pack_2.bus_service import I2cAdapter
import opt3001mod
import time

def show_header(info: str, width: int = 32):
    print(width * "-")
    print(info)
    print(width * "-")

def delay_ms(val: int):
    time.sleep_ms(val)

if __name__ == '__main__':
    i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)  # on Raspberry Pi Pico
    adapter = I2cAdapter(i2c)

    als = opt3001mod.OPT3001(adapter)
    _id = als.get_id()
    print(f"manufacturer id: 0x{_id.manufacturer_id:x}; device id: 0x{_id.device_id:x}")

    show_header("Однократный режим работы! Запуск измерения вручную! Автоматический выбор предела измерения датчиком!")
    als.long_conversion_time = False
    als.start_measurement(continuously=False, lx_range_index=10, refresh=False)
    cycle_time = als.get_conversion_cycle_time()
    print(f"cycle time мс: {cycle_time}")
    print(als.read_config_from_sensor(return_value=True))
    _repeat_count = 10
    for _ in range(_repeat_count):
        delay_ms(cycle_time + 50)
        ds = als.get_data_status()
        if ds.conversion_ready:
            # обработанные данные!
            value = als.get_measurement_value(value_index=1)
            print(value)
        else:
            print(f"Данные не готовы для считывания!")
        # запуск измерения вручную
        als.start_measurement(continuously=False, lx_range_index=12, refresh=False)

    show_header("Автоматический запуск измерения! Автоматический выбор предела измерения датчиком!")
    als.long_conversion_time = True
    als.start_measurement(continuously=True, lx_range_index=12, refresh=False)
    print(als.read_config_from_sensor(return_value=True))
    cycle_time = als.get_conversion_cycle_time()
    print(f"cycle time мс: {cycle_time}; увеличенное время преобразования: {als.long_conversion_time}")
    for _ in range(10 * _repeat_count):
        delay_ms(cycle_time)
        ds = als.get_data_status()
        if ds.conversion_ready:
            # обработанные данные!
            value = als.get_measurement_value(value_index=1)
            print(value)
        else:
            print(f"Данные не готовы для считывания!")
