"""Модуль для управления датчиком внешней освещенности OPT3001 от TI."""

# micropython
# MIT license

from micropython import const
from sensor_pack_2 import bus_service
from sensor_pack_2.base_sensor import DeviceEx, IBaseSensorEx, Iterator, check_value
from collections import namedtuple
from sensor_pack_2.bitfield import bit_field_info
from sensor_pack_2.bitfield import BitFields


opt3001_id = namedtuple("opt3001_id", "manufacturer_id device_id")
opt3001_config = namedtuple("config_opt3001", "RN CT M OVF CRF FH FL L POL ME FC")
# результат измерения освещенности датчиком
# exponent и fractional два поля сырых данных
# lux - освещенность в Люксах (https://en.wikipedia.org/wiki/Lux)
opt3001_meas_data = namedtuple("opt3001_meas_data", "lux full_scale_range")
# 'сырые' измеренные данные
opt3001_meas_raw = namedtuple("opt3001_meas_raw", "exponent fractional")
# FSR - Full Scale Range в Люксах! Предельная освещенность, на которую настроен датчик
# LSB - цена наименее значащего бита в Люксах!
_opt3001_lsb_fsr = namedtuple("_opt3001_lsb_fsr", "LSB FSR")
# готовы ли данные для считывания?
OPT3001_data_status = namedtuple("OPT3001_data_status", "conversion_ready overflow")

class OPT3001(IBaseSensorEx, Iterator):
    """Представление OPT3001, ALS от именитого производителя."""

    FMT_UINT16 = "H"
    ADDR_MEAS_RAW_REG = const(0x00)
    ADDR_CFG_REG = const(0x01)
    #
    ADDR_LOW_LIMIT_REG = const(0x02)
    ADDR_HIGH_LIMIT_REG = const(0x03)
    #
    ADDR_MAN_ID_REG = const(0x7E)
    ADDR_DEV_ID_REG = const(0x7F)
    #
    FN_RANGE_NUMBER = const("RN")
    FN_CONV_TIME = const("CT")
    FN_CONV_MODE = const("M")
    FN_OVF_FLAG = const("OVF")
    FN_CONV_READY = const("CRF")
    FN_FLAG_HIGH = const("FH")
    FN_FLAG_LOW = const("FL")
    FN_LATCH_FIELD = const("L")
    FN_POL = const("POL")
    FN_MASK_EXP = const("ME")
    FN_FAULT_COUNT = const("FC")

    _config_reg_opt3001 = (bit_field_info(name=FN_RANGE_NUMBER, position=range(12, 16), valid_values=range(13), description="Range number field."),  # Reset Bit
                          bit_field_info(name=FN_CONV_TIME, position=range(11, 12), valid_values=None, description="Conversion time field."),
                          bit_field_info(name=FN_CONV_MODE, position=range(9, 11), valid_values=range(4), description="Mode of conversion operation field."),
                          bit_field_info(name=FN_OVF_FLAG, position=range(8, 9), valid_values=None, description="Overflow flag."),
                          bit_field_info(name=FN_CONV_READY, position=range(7, 8), valid_values=None, description="Conversion ready."),
                          bit_field_info(name=FN_FLAG_HIGH, position=range(6, 7), valid_values=None, description='Flag high'),
                          bit_field_info(name=FN_FLAG_LOW, position=range(5, 6), valid_values=None, description='Flag low'),
                          bit_field_info(name=FN_LATCH_FIELD, position=range(4, 5), valid_values=None, description='Latch field'),
                          bit_field_info(name=FN_POL, position=range(3, 4), valid_values=None, description='Polarity'),
                          bit_field_info(name=FN_MASK_EXP, position=range(2, 3), valid_values=None, description='Mask exponent'),
                          bit_field_info(name=FN_FAULT_COUNT, position=range(2), valid_values=None, description='Fault count'),
                          )

    @staticmethod
    def _validate_limits(low_lux: float, high_lux: float) -> None:
        """Проверяет корректность пары порогов для оконного компаратора.

        Высокий порог должен быть строго больше низкого.

        Args:
            low_lux: Нижний порог в люксах.
            high_lux: Верхний порог в люксах.

        Raises:
            ValueError: Если high_lux <= low_lux.
        """
        if high_lux <= low_lux:
            raise ValueError(f"Верхний порог ({high_lux} lux) должен быть строго больше нижнего порога ({low_lux} lux)")

    @staticmethod
    def _validate_lux_limit(value: float, name: str = "Порог"):
        if value < 0:
            raise ValueError(f"{name} не может быть отрицательным")
        if value > 83865.60:
            raise ValueError(f"{name} ({value}) превышает максимальное значение датчика (83865.60 lux)")

    @staticmethod
    def _lux_to_raw(lux: float) -> int:
        """Упаковывает значение в люксах в 16-битный формат OPT3001 (E[3:0] + R[11:0])."""
        if lux <= 0:
            return 0x0000
        if lux >= 83865.60:
            return 0xBFFF  # Максимальное значение датчика (E=11, R=0xFFF)

        # Поиск оптимальной экспоненты, чтобы мантисса не превышала 4095
        for exp in range(12):
            max_lux_for_exp = 40.95 * (2 ** exp)
            if lux <= max_lux_for_exp:
                fractional = int(round(lux / (0.01 * (2 ** exp))))
                # Ограничиваю мантиссу 12 битами
                fractional = min(fractional, 0x0FFF)
                return (exp << 12) | fractional
        return 0xBFFF

    @staticmethod
    def _raw_to_lux(raw: int) -> float:
        """Распаковывает 16-битное значение регистра в люксы."""
        exp = (raw >> 12) & 0x0F
        frac = raw & 0x0FFF
        # Спецрежим 11b для Low-Limit не имеет физического смысла в люксах
        if exp >= 12:
            return float('nan')
        return (0.01 * (2 ** exp)) * frac

    def __init__(self, adapter: bus_service.BusAdapter, address=0x44):
        check_value(address, range(0x44, 0x48), f"Неверное значение адреса I2C устройства: 0x{address:x}")
        self._connection = DeviceEx(adapter=adapter, address=address, big_byte_order=True)
        # для удобства работы с настройками
        # информация о полях регистра конфигурации устройства = OPT3001._config_reg_opt3001
        self._bit_fields = BitFields(fields_info=OPT3001._config_reg_opt3001)
        self._buf_2 = bytearray(2)

    def _set_reg(self, addr: int, format_value: str | None = FMT_UINT16, value: int | None = None) -> int:
        """Возвращает (при value is None)/устанавливает (при not value is None) содержимое регистра с адресом addr.
        разрядность регистра 16 бит!"""
        buf = self._buf_2
        _conn = self._connection
        if value is None:
            # читаю из Register устройства в буфер два байта
            if format_value is None:
                raise ValueError("При чтении из регистра не задан формат его значения!")
            _conn.read_buf_from_mem(address=addr, buf=buf, address_size=1)
            return _conn.unpack(fmt_char=format_value, source=buf)[0]
        #
        return self._connection.write_reg(reg_addr=addr, value=value, bytes_count=len(buf))


    def _get_config_field(self, field_name: str | None = None) -> int | bool:
        """Возвращает значение поля по его имени, field_name, из сохраненной конфигурации.
        Если field_name is None, будут возвращены все поля конфигурации в виде int"""
        bf = self._bit_fields
        if field_name is None:
            return bf.source
        return bf[field_name]

    def _set_config_field(self, value: int, field_name: str | None = None):
        """Устанавливает значение поля, value, по его имени, field_name, в сохраненной конфигурации.
        Если field_name is None, будут установлены значения всех полей конфигурации."""
        bf = self._bit_fields
        if field_name is None:
            bf.source = value
            return
        bf[field_name] = value

    @staticmethod
    def _get_lsb_fsr(exp_raw: int) -> _opt3001_lsb_fsr:
        # check_value(exp_raw, range(12), f"Неверное значение exp_raw: {exp_raw}")
        _lsb, _fsr = None, None
        if exp_raw < 0 or exp_raw > 11:
            return _opt3001_lsb_fsr(_lsb, _fsr) # ошибка
        _lsb = 0.01 * 2 ** exp_raw
        _fsr = 40.95 * 2 ** exp_raw
        return _opt3001_lsb_fsr(_lsb, _fsr)


    def get_cfg_reg(self) -> int:
        """Возвращает 'сырую' конфигурацию из регистра. Get raw configuration from register"""
        # return self.read_reg_16(0x01)
        return self._set_reg(addr=self.ADDR_CFG_REG, format_value=self.FMT_UINT16)

    def set_cfg_reg(self, value: int) -> None:
        """Установить сырую конфигурацию в регистре. Set raw configuration in register."""
        # return self.write_reg_16(0x01, value)
        self._set_reg(addr=self.ADDR_CFG_REG, format_value=None, value=value)

    def get_config_hr(self) -> opt3001_config:
        """"Возвращает настройки датчика"""
        return opt3001_config(RN=self.lux_range_index, CT=self.long_conversion_time, M=self.mode, OVF=self.overflow,
                              CRF=self.conversion_ready, FH=self.flag_high, FL=self.flag_low, L=self.latch,
                              POL=self.polarity, ME=self.mask_exponent, FC=self.fault_count_index)

    def read_config_from_sensor(self, return_value: bool = False) -> None | opt3001_config:
        """Возврат текущей конфигурации датчика в виде кортежа.
        Вызовите этот метод, когда считаете, что нужно обновить конфигурацию в полях класса!!!"""
        raw = self.get_cfg_reg()
        self._set_config_field(value=raw, field_name=None)
        if return_value:
            return self.get_config_hr()
        return None

    def write_config_to_sensor(self) -> int:
        """Настраивает датчик в соответствии с настройками в полях класса.
        Возвращает значение настроек в сыром(!) виде"""
        _cfg = self._get_config_field(field_name=None)
        self.set_cfg_reg(_cfg)
        return _cfg

    # BaseSensorEx
    def get_id(self) -> opt3001_id:
        # man_id, dev_id = self.read_reg_16(0x7E), self.read_reg_16(0x7F)
        frmt = self.FMT_UINT16
        man_id, dev_id = self._set_reg(addr=self.ADDR_MAN_ID_REG, format_value=frmt), self._set_reg(addr=self.ADDR_DEV_ID_REG, format_value=frmt)
        return opt3001_id(manufacturer_id=man_id, device_id=dev_id)

    @property
    def lux_range_index(self) -> int:
        """Возвращает измеряемый диапазон освещенности в 'сыром/raw' виде:
        index       lux         lux_per_LSB
        0           40.95       0.01
        1           81.90       0.02
        2           163.80      0.04
        3           327.60      0.08
        4           655.20      0.16
        5           1310.40     0.32
        6           2620.80     0.64
        7           5241.60     1.28
        8           10483.20    2.56
        9           20966.40    5.12
        10          41932.80    10.24
        11          83865.60    20.48
        12          automatic full-scale range
        """
        return self._get_config_field(self.FN_RANGE_NUMBER)

    @lux_range_index.setter
    def lux_range_index(self, value: int):
        """Устанавливает измеряемый диапазон освещенности"""
        check_value(value, range(13), error_msg=f"Неверное значение lux_range_index: {value}")
        self._set_config_field(value, field_name=self.FN_RANGE_NUMBER)

    @property
    def long_conversion_time(self) -> bool:
        """Возвращает время преобразования датчиком кол-ва света в цифровой код. False - 100 мс. True - 800 мс."""
        return self._get_config_field(self.FN_CONV_TIME)

    @long_conversion_time.setter
    def long_conversion_time(self, value: bool):
        """Устанавливает время преобразования датчиком кол-ва света в цифровой код. False - 100 мс. True - 800 мс."""
        self._set_config_field(value, field_name=self.FN_CONV_TIME)

    @property
    def mode(self) -> int:
        """Возвращает режим работы датчика:
        * 0 - режим экономии энергии датчиком
        * 1 - одиночный режим работы
        * 2, 3 - автоматический режим работы. После завершения одного измерения, начинается другое!"""
        return self._get_config_field(self.FN_CONV_MODE)

    @mode.setter
    def mode(self, value: int):
        """Устанавливает режим работы датчика"""
        self._set_config_field(value, field_name=self.FN_CONV_MODE)

    @property
    def overflow(self) -> bool:
        """Флаг переполнения указывает на возникновение состояния переполнения в процессе преобразования данных,
        обычно из-за того, что свет, освещающий устройство, превышает запрограммированный полный диапазон шкалы устройства."""
        return self._get_config_field(self.FN_OVF_FLAG)

    @property
    def conversion_ready(self) -> bool:
        """Указывает, когда завершено преобразование освещенности в цифровой код."""
        return self._get_config_field(self.FN_CONV_READY)

    @property
    def flag_high(self) -> bool:
        """Возвращает флаг сравнения.
        Этот флаг указывает на то, что результат преобразования превышает указанный уровень интереса."""
        return self._get_config_field(self.FN_FLAG_HIGH)

    @property
    def flag_low(self) -> bool:
        """Флаг низкого порога (FL, бит 5, только чтение).

        Указывает, что результат преобразования освещённости ниже заданного
        уровня интереса (низкий порог в регистре Low-Limit, адрес 02h).

        Флаг устанавливается в 1, когда результат измерения остаётся ниже
        значения низкого порога в течение количества последовательных измерений,
        заданного полем Fault Count (FC[1:0]).

        Поведение сброса флага зависит от режима прерываний (поле L):
        - L=1 (latched): сбрасывается при чтении регистра конфигурации
        - L=0 (transparent): отражает текущее состояние сравнения

        Returns:
            bool: True, если сработал флаг низкого порога.
        """
        return self._get_config_field(self.FN_FLAG_LOW)

    @property
    def latch(self) -> bool:
        """Поле защёлки прерываний (L, бит 4, чтение/запись).

        Управляет стилем отчёта механизмов прерываний (вывод INT, флаги FH и FL):

        - False (0): прозрачный режим с гистерезисом (transparent hysteresis-style).
          Флаги и INT напрямую отражают результат сравнения с порогами.
          Сброс пользователем не требуется.

        - True (1): защёлкнутый оконный режим (latched window-style).
          Флаги и INT фиксируются при срабатывании и удерживаются до
          чтения регистра конфигурации (или SMBus Alert Response для INT).

        Default: True (latched mode).

        Returns/Args:
            bool: Режим работы механизмов прерываний.
        """
        return self._get_config_field(self.FN_LATCH_FIELD)

    @latch.setter
    def latch(self, value: bool):
        """Устанавливает режим защёлки прерываний (см. docstring свойства)."""
        self._set_config_field(value, field_name=self.FN_LATCH_FIELD)

    @property
    def polarity(self) -> bool:
        """Полярность вывода прерывания INT (POL, бит 3, чтение/запись).

        Определяет активный уровень сигнала на выводе INT:

        - False (0): активный низкий уровень (active-low).
        При срабатывании прерывания вывод подтягивается к GND.

        - True (1): активный высокий уровень (active-high).
        При срабатывании прерывания вывод переходит в высокоимпедансное
        состояние, позволяя внешнему pull-up резистору установить высокий уровень.

        Вывод INT имеет open-drain структуру и требует внешнего pull-up резистора.

        Default: False (active-low).

        Returns/Args:
            bool: Полярность сигнала прерывания.
        """
        return self._get_config_field(self.FN_POL)

    @polarity.setter
    def polarity(self, value: bool):
        """Устанавливает полярность вывода INT (см. docstring свойства)."""
        self._set_config_field(value, field_name=self.FN_POL)

    @property
    def mask_exponent(self) -> bool:
        """Маскирование экспоненты результата (ME, бит 2, чтение/запись).

        При установке в True и ручном выборе диапазона (RN[3:0] < 12)
        принудительно обнуляет поле экспоненты (биты 15:12) в регистре
        результата (00h), упрощая обработку данных:

        - Результат всегда имеет формат: 0000b + 12-битная мантисса
        - Расчёт освещённости: lux = LSB_Size × R[11:0],
          где LSB_Size фиксирован для выбранного диапазона

        Важно:
        - Маскирование применяется только к регистру результата.
        - Сравнение с порогами (Low-Limit/High-Limit) для прерываний
          НЕ затрагивается этим полем.
        - Не влияет на автоматический выбор диапазона (RN=12).

        Default: False.

        Returns/Args:
            bool: Режим маскирования экспоненты.
        """
        return self._get_config_field(self.FN_MASK_EXP)

    @mask_exponent.setter
    def mask_exponent(self, value: bool):
        """Устанавливает режим маскирования экспоненты (см. docstring свойства)."""
        self._set_config_field(value, field_name=self.FN_MASK_EXP)

    @property
    def fault_count_index(self) -> int:
        """
        Счётчик последовательных событий для триггера прерываний (биты 1:0).
        Значения: 0=1 событие, 1=2, 2=4, 3=8 событий (по умолчанию 0).
        """
        return self._get_config_field(self.FN_FAULT_COUNT)

    @fault_count_index.setter
    def fault_count_index(self, value: int):
        """Устанавливает счётчик ошибок для триггера прерываний (см. таблицу ниже).

        Значения:
            0 = 1 последовательное событие (по умолчанию)
            1 = 2 события
            2 = 4 события
            3 = 8 событий
        """
        check_value(value, range(4), f"Неверное значение fault_count: {value}")
        self._set_config_field(value, field_name=self.FN_FAULT_COUNT)

    def set_limits(self, low_lux: float | None = None, high_lux: float | None = None) -> None:
        """Устанавливает один или оба порога прерывания.

        Метод позволяет задать только нижний, только верхний или оба порога сразу.
        Если порог не требуется изменять, передайте None.

        Args:
            low_lux: Нижний порог в люксах (0.01–83865.60) или None.
            high_lux: Верхний порог в люксах (0.01–83865.60) или None.

        Raises:
            ValueError: Если оба параметра равны None, значения выходят за допустимый диапазон
                        или high_lux <= low_lux (когда оба указаны).

        Пример:
            >>> sensor.set_limits(low_lux=100.0)             # Только нижний
            >>> sensor.set_limits(high_lux=1000.0)           # Только верхний
            >>> sensor.set_limits(low_lux=100, high_lux=1000)# Оба сразу
            >>> sensor.set_limits()                          # ❌ ValueError
            >>> sensor.set_limits(500, 100)                  # ❌ ValueError
        """
        if low_lux is None and high_lux is None:
            raise ValueError("Необходимо указать хотя бы один порог (low_lux или high_lux)")

        # Пошаговая валидация диапазонов
        if low_lux is not None:
            OPT3001._validate_lux_limit(value=low_lux, name='Нижний порог')
        if high_lux is not None:
            OPT3001._validate_lux_limit(value=high_lux, name='Верхний порог')

        # Валидация логической связи (только при одновременной установке обоих)
        if low_lux is not None and high_lux is not None:
            OPT3001._validate_limits(low_lux=low_lux, high_lux=high_lux)

        # Атомарная запись только изменённых регистров
        if low_lux is not None:
            self._set_reg(addr=self.ADDR_LOW_LIMIT_REG, format_value=None, value=OPT3001._lux_to_raw(low_lux))
        if high_lux is not None:
            self._set_reg(addr=self.ADDR_HIGH_LIMIT_REG, format_value=None, value=OPT3001._lux_to_raw(high_lux))

    def get_limits(self) -> tuple[float, float]:
        """Возвращает текущие пороги прерывания (низкий, высокий) в люксах.

        Returns:
            tuple[float, float]: Кортеж (low_lux, high_lux).
            Значение может быть float('nan'), если регистр настроен
            в специальный режим (например, end-of-conversion, LE[3:2]=11b).

        Пример:
            >>> low, high = sensor.get_limits()
            >>> print(f"Окно: {low:.2f} - {high:.2f} lux")
        """
        low_raw = self._set_reg(addr=self.ADDR_LOW_LIMIT_REG, format_value=self.FMT_UINT16)
        high_raw = self._set_reg(addr=self.ADDR_HIGH_LIMIT_REG, format_value=self.FMT_UINT16)
        return OPT3001._raw_to_lux(low_raw), OPT3001._raw_to_lux(high_raw)

    # IBaseSensorEx

    def get_conversion_cycle_time(self) -> int:
        """Возвращает время в мс преобразования сигнала в цифровой код и готовности его для чтения по шине!
        Для текущих настроек датчика. При изменении настроек следует заново вызвать этот метод!"""
        if self.long_conversion_time:
            return 803
        return 103

    def start_measurement(self, continuously: bool = True, lx_range_index: int = 12, refresh: bool = False):
        """Настраивает параметры датчика и запускает процесс измерения.
        Если refresh is True, то после настройки датчика, обновляется содержимое полей, хранящих настройки датчика!
        lx_range_index - индекс диапазона освещенности, 12 - автоматический(!) выбор. Смотри lux_range_index,
        читай 7.4.1 Automatic Full-Scale Setting Mode"""
        self.mode = 3 if continuously else 1
        self.lux_range_index = lx_range_index
        #
        self.write_config_to_sensor()
        if refresh:
            self.read_config_from_sensor(return_value=False)

    def get_measurement_value(self, value_index: int = 0) -> None | opt3001_meas_raw | opt3001_meas_data:
        """Возвращает измеренное датчиком значение(значения) по его индексу/номеру."""
        # raw = self.read_reg_16(0, signed=False)
        raw = self._set_reg(addr=self.ADDR_MEAS_RAW_REG, format_value=self.FMT_UINT16)
        _exponent, _fractional = (raw & 0xF000) >> 12, raw & 0x0FFF
        if 0 == value_index:
            return opt3001_meas_raw(exponent=_exponent, fractional=_fractional)
        if 1 == value_index:
            # обработанные данные в Люксах
            _data = OPT3001._get_lsb_fsr(_exponent)
            if _data.LSB is None:  # экспонента вне диапазона 0-11
                return opt3001_meas_data(lux=None, full_scale_range=None)
            _lux = _data.LSB * _fractional
            return opt3001_meas_data(lux=_lux, full_scale_range=_data.FSR)
        return None

    def get_data_status(self, raw: bool = True) -> OPT3001_data_status:
        self.read_config_from_sensor(return_value=False)
        return OPT3001_data_status(conversion_ready=self.conversion_ready, overflow=self.overflow)

    def is_single_shot_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме однократных измерений,
        каждое из которых запускается методом start_measurement"""
        return 1 == self.mode

    def is_continuously_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме многократных измерений,
        производимых автоматически. Процесс запускается методом start_measurement"""
        return 2 == self.mode or 3 == self.mode

#   Iterator
    def __iter__(self):
        return self

    def __next__(self) -> None | opt3001_meas_data:
        """Возвращает измеренные значения. Кортеж, число."""
        ds = self.get_data_status() # обновляю содержимое полей экземпляра класса
        if self.is_continuously_mode():
            if ds.conversion_ready:
                return self.get_measurement_value(1)
        return None
