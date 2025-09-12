# Makefile для компиляции прошивки STM32 Motor Controller

# Настройки компилятора
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

# Флаги компиляции
CFLAGS = -mcpu=cortex-m3 -mthumb -Wall -Wextra -O2 -g
CFLAGS += -DSTM32F103xB -DUSE_HAL_DRIVER
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-common -fmessage-length=0

# Флаги линковщика
LDFLAGS = -mcpu=cortex-m3 -mthumb -specs=nano.specs
LDFLAGS += -T STM32F103C8Tx_FLASH.ld
LDFLAGS += -Wl,--gc-sections -Wl,--print-memory-usage

# Исходные файлы
SOURCES = stm32_motor_controller.c
SOURCES += system_stm32f1xx.c
SOURCES += startup_stm32f103c8tx.s

# Объектные файлы
OBJECTS = $(SOURCES:.c=.o)
OBJECTS := $(OBJECTS:.s=.o)

# Целевые файлы
TARGET = motor_controller
ELF_FILE = $(TARGET).elf
BIN_FILE = $(TARGET).bin
HEX_FILE = $(TARGET).hex

# Пути к библиотекам HAL
HAL_DIR = /usr/lib/arm-none-eabi/include
INCLUDES = -I. -I$(HAL_DIR)

# Основная цель
all: $(BIN_FILE) $(HEX_FILE)
	@echo "=== Компиляция завершена ==="
	@$(SIZE) $(ELF_FILE)

# Создание .bin файла
$(BIN_FILE): $(ELF_FILE)
	@echo "Создание $(BIN_FILE)..."
	$(OBJCOPY) -O binary $< $@

# Создание .hex файла
$(HEX_FILE): $(ELF_FILE)
	@echo "Создание $(HEX_FILE)..."
	$(OBJCOPY) -O ihex $< $@

# Создание .elf файла
$(ELF_FILE): $(OBJECTS)
	@echo "Линковка $(ELF_FILE)..."
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# Компиляция .c файлов
%.o: %.c
	@echo "Компиляция $<..."
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Компиляция .s файлов
%.o: %.s
	@echo "Ассемблирование $<..."
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Очистка
clean:
	@echo "Очистка файлов..."
	rm -f $(OBJECTS) $(ELF_FILE) $(BIN_FILE) $(HEX_FILE)

# Установка зависимостей
install-deps:
	@echo "Установка зависимостей для компиляции STM32..."
	sudo apt update
	sudo apt install -y gcc-arm-none-eabi libnewlib-arm-none-eabi

# Проверка зависимостей
check-deps:
	@echo "Проверка зависимостей..."
	@which $(CC) > /dev/null || (echo "Ошибка: $(CC) не найден. Выполните: make install-deps" && exit 1)
	@which $(OBJCOPY) > /dev/null || (echo "Ошибка: $(OBJCOPY) не найден. Выполните: make install-deps" && exit 1)
	@echo "✓ Все зависимости установлены"

# Прошивка (требует stm32flash)
flash: $(BIN_FILE)
	@echo "Прошивка $(BIN_FILE)..."
	./flash_motor_controller.sh $(BIN_FILE)

# Тестирование связи
test:
	@echo "Тестирование связи с hoverboard..."
	python3 test_hoverboard_connection.py

# Интерактивное тестирование
test-interactive:
	@echo "Интерактивное тестирование..."
	python3 test_hoverboard_connection.py --interactive

# Справка
help:
	@echo "Доступные команды:"
	@echo "  make              - Компиляция прошивки"
	@echo "  make clean        - Очистка файлов"
	@echo "  make install-deps - Установка зависимостей"
	@echo "  make check-deps   - Проверка зависимостей"
	@echo "  make flash        - Прошивка платы"
	@echo "  make test         - Тестирование связи"
	@echo "  make test-interactive - Интерактивное тестирование"
	@echo "  make help         - Показать эту справку"

.PHONY: all clean install-deps check-deps flash test test-interactive help