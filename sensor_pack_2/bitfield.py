# micropython
# MIT license
# Copyright (c) 2024 Roman Shevchik   goctaprog@gmail.com
"""Представление битового поля"""
import micropython
from collections import namedtuple
from sensor_pack_2.base_sensor import check_value, get_error_str

# Информация о битовом поле в виде именованного кортежа
# name: str  - имя
# position: range - место в номерах битах, position.start = первый бит, position.stop-1 - последний бит;
# valid_values: range | tuple - диапазон допустимых значений, если проверка не требуется, следует передать None;
# description: str - читаемое описание значения, хранимого в битовом поле, если описания не требуется, следует передать None;
bit_field_info = namedtuple("bit_field_info", "name position valid_values description")


@micropython.native
#def _bitmask(bit_rng: range) -> int:
#    """возвращает битовую маску по занимаемым битам"""
#    # if bit_rng.step < 0 or bit_rng.start <= bit_rng.stop:
#    #    raise ValueError(f"_bitmask: {bit_rng.start}; {bit_rng.stop}; {bit_rng.step}")
#    return sum(map(lambda x: 2 ** x, bit_rng))

@micropython.native
def _bitmask(bit_rng: range) -> int:
    start = bit_rng.start
    stop = bit_rng.stop
    step = bit_rng.step

    if step == 0:
        raise ValueError("step не может быть нулевым!")

    if step > 0:
        if start >= stop:
            raise ValueError("range. пустой диапазон!")
        lo = start
        hi = stop - 1  # последний элемент
    else:  # step < 0
        if start <= stop:
            raise ValueError("range. пустой диапазон!")
        lo = stop + 1  # первый элемент снизу
        hi = start

    # Теперь lo <= hi — реальные границы битов
    return ((1 << (hi - lo + 1)) - 1) << lo

#def _bitmask(bit_rng: range) -> int:
#    """Возвращает битовую маску по занимаемым битам (только для step=1 и start < stop)."""
#    check_value(value=bit_rng.step, valid_range=(1,), error_str="Поддерживается только step=1!")
#    start, stop = bit_rng.start, bit_rng.stop
#    if start >= stop:
#        raise ValueError("rng.start должен быть меньше rng.stop!")
#    return ((1 << (stop - start)) - 1) << start

class BitFields:
    """Хранилище информации о битовых полях с доступом по индексу."""

    @staticmethod
    def _iter_fields(items: tuple[bit_field_info, ...] | tuple[tuple[bit_field_info, ...], ...]) -> bit_field_info:
        """Генератор, возвращающий все bit_field_info из:
        - плоского кортежа: (все bit_field_info, все bit_field_info, все bit_field_info), или
        - вложенного: ((все bit_field_info, все bit_field_info), (все bit_field_info, все bit_field_info))."""
        for item in items:
            if isinstance(item, tuple) and item and type(item[0]) is bit_field_info:
                # Это группа полей — обходим её
                for field in item:
                    yield field
            else:
                # Это отдельное поле
                yield item

    @staticmethod
    def _check(items: tuple[bit_field_info, ...] | tuple[tuple[bit_field_info, ...], ...]):
        """Проверки на правильность информации!"""
        if not items:
            raise ValueError("Кортеж пуст!")
        for field in BitFields._iter_fields(items):
            if not field.name:
                raise ValueError(f"Нулевая длина строки имени битового поля!; position: {field.position}")
            if not field.position:
                raise ValueError(f"Нулевая длина ('в битах') битового поля!; name: {field.name}")

    def __init__(self, fields_info: tuple[bit_field_info, ...] | tuple[tuple[bit_field_info, ...], ...]):
        BitFields._check(fields_info)
        self._fields_info = fields_info
        # print(f"DBG:__init__.self._fields_info: {self._fields_info}")
        # имя битового поля, которое будет параметром у методов get_value/set_value
        # first_elem = None
        first_elem = next(BitFields._iter_fields(fields_info))
        self._active_field_name = first_elem.name
        # значение, из которого будут извлекаться битовые поля
        self._source_val = 0

    def _by_name(self, name: str) -> bit_field_info | None:
        """возвращает информацию о битовом поле по его имени (поле name именованного кортежа) или None"""

        def _find_in_tuple(items: tuple[bit_field_info, ...]) -> bit_field_info | None:
            """Ищет поле с заданным именем в одном кортеже bit_field_info."""
            for _item in items:
                # print(f"DBG:_find_in_tuple: {type(_item)}, {type(items)}")
                if name == _item.name:
                    return _item
            return None

        for item in self._fields_info:
            # print(f"DBG:_by_name: {type(item)}, {type(self._fields_info)}")
            if type(item) is bit_field_info:
                # item — это отдельное поле
                if name == item.name:
                    return item
            else:
                # item — это кортеж, содержащий bit_field_info: (f1, f2, ...), перебираю
                found = _find_in_tuple(item)
                if found is not None:
                    return found
        return None

    def _get_field(self, key: str | int | None) -> bit_field_info | None:
        """Возвращает bit_field_info по имени (str), индексу (int) или self.field_name (если key is None)."""
        if key is None:
            return self._by_name(self.field_name)

        if isinstance(key, str):
            return self._by_name(key)

        if isinstance(key, int):
            # Поддержка индекса — только если структура плоская
            items = self._fields_info
            if not items:
                raise IndexError("Кортеж пуст!")
            if isinstance(items[0], bit_field_info):
                # Плоский кортеж — индексация разрешена
                return items[key]
            # Вложенная структура — индексация не поддерживается
            raise TypeError(
                "Доступ по индексу не поддерживается для вложенных кортежей. Вместо этого используйте имя поля!")

        raise TypeError(f"Неподдерживаемый тип ключа: {type(key)}")

    def get_field_value(self, field_name: str = None, validate: bool = False) -> int | bool:
        """возвращает значение битового поля, по его имени(self.field_name), из self.source."""
        item = self._get_field(field_name)
        if item is None:
            raise ValueError(f"get_field_value. Поле с именем {field_name} не существует!")
        pos = item.position
        # print(f"DBG:pos = item.position: {pos}")
        bitmask = _bitmask(pos)
        # print(f"DBG:bitmask =: {bitmask}")
        val = (self.source & bitmask) >> pos.start  # выделение маской битового диапазона и его сдвиг вправо
        if item.valid_values and validate:
            raise NotImplementedError(
                "Если вы решили проверить значение поля при его возвращении, то делайте это самостоятельно!!!")
        if 1 == len(pos):
            return 0 != val  # bool
        return val  # int

    def set_field_value(self, value: int, source: int | None = None, field: str | int | None = None,
                        validate: bool = True) -> int:
        """Записывает value в битовый диапазон, определяемый параметром field, в source.
        Возвращает значение с измененным битовым полем.
        Если field is None, то имя поля берется из свойства self._active_field_name.
        Если source is None, то значение поля, подлежащее изменению, изменяется в свойстве self._source_val"""
        item = self._get_field(key=field)  # *
        rng = item.valid_values
        if rng and validate:
            check_value(value, rng, get_error_str(self.field_name, value, rng))
        pos = item.position
        bitmask = _bitmask(pos)
        src = self._get_source(source) & ~bitmask  # чистка битового диапазона
        src |= (value << pos.start) & bitmask  # установка битов в заданном диапазоне
        if source is None:
            self._source_val = src
        return src

    def __getitem__(self, key: int | str) -> int | bool:
        """возвращает значение битового поля из значения в self.source по его имени/индексу"""
        # print(f"DBG:__getitem__: key: {key}")
        _bfi = self._get_field(key)
        return self.get_field_value(_bfi.name)

    def __setitem__(self, field_name: str, value: int | bool):
        """Волшебный метод, вызывает set_field_value.
        До его вызова нужно установить свойства BitField source"""
        self.set_field_value(value=value, source=None, field=field_name, validate=True)  # *

    def _get_source(self, source: int | None) -> int:
        return source if source else self._source_val

    @property
    def source(self) -> int:
        """Значение, из которого будут извлекаться значения битовых полей.
        Значение, в котором будут меняться, значения битовых полей."""
        return self._source_val

    @source.setter
    def source(self, value):
        """значение, из которого будут извлекаться/изменятся битовые поля"""
        self._source_val = value

    @property
    def field_name(self) -> str:
        """имя битового поля, значение которого извлекается/изменяется методами get_value/set_value, если их
        параметр field is None"""
        return self._active_field_name

    @field_name.setter
    def field_name(self, value):
        """имя битового поля, значение которого извлекается/изменяется методами get_value/set_value, если их
        параметр field is None"""
        self._active_field_name = value

    def __contains__(self, name: str) -> bool:
        """
        Возвращает True, если битовое поле с указанным именем существует
        в текущей конфигурации, иначе False.

        Позволяет использовать оператор in:
            if "SHUTDOWN" in bit_fields:
            ...

        Аргументы:
            name (str): имя битового поля.

        Возвращает:
            bool: True — поле найдено, False — не найдено.
        """
        return self._by_name(name) is not None

    def __len__(self) -> int:
        return sum(1 for _ in BitFields._iter_fields(self._fields_info))

    # протокол итератора
    def __iter__(self):
        return BitFields._iter_fields(self._fields_info)


def make_namedtuple(source: BitFields) -> tuple:
    """
    Создаёт именованный кортеж из текущих значений всех битовых полей в BitFields.
    Входные параметры:
        bf (BitFields): экземпляр BitFields.
    Возвращает:
        namedtuple: кортеж с именованными полями и их текущими значениями.
    """
    # Получаем все имена полей через итерацию
    field_names = tuple(info.name for info in source)
    # print(f"DBG:make_namedtuple:field_names: {field_names}")
    #for name in field_names:
    #    print(f"name: {name}")
    #    print(f"DBG:source[name]:{name}-{source[name]}")
    # Получаем соответствующие значения
    field_values = tuple(source[name] for name in field_names)
    #print(f"DBG:make_namedtuple:field_values: {field_values}")
    # Создаём класс namedtuple
    NT = namedtuple("BitFieldsSnapshot", field_names)
    # Возвращаем экземпляр
    return NT(*field_values)
