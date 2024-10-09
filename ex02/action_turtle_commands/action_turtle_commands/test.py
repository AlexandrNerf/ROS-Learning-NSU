def parse_command(input_str):
    # Регулярное выражение для парсинга строки
    pattern = r"(?P<a>\w+)(?:\s-b\s(?P<b>\d+))?(?:\s-c\s(?P<c>\d+))?"
    match = re.match(pattern, input_str)

    if match:
        a = match.group("a")
        b = match.group("b")
        c = match.group("c")

        # Преобразуем b и c в int, если они есть
        b = int(b) if b else None
        c = int(c) if c else None

        return a, b, c
    else:
        raise ValueError("Неверный формат команды")

# Пример использования
input_str = "test -b 5 -c 10"
a, b, c = parse_command(input_str)
print(f"Значение a: {a}, Значение b: {b}, Значение c: {c}")
