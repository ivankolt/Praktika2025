/*
 * Простая прошивка STM32 для управления моторами через UART
 * Принимает команды: steer, speed (по 2 байта каждое)
 * Протокол: [steer_low][steer_high][speed_low][speed_high]
 * 
 * Для STM32F103C8T6 (Blue Pill)
 * UART1: PA9 (TX), PA10 (RX)
 * PWM: PA8 (TIM1_CH1), PA11 (TIM1_CH4)
 * Direction: PA12, PA15
 */

#include <stdint.h>

// Адреса регистров STM32F103
#define RCC_APB2ENR    (*((volatile uint32_t*)0x40021018))
#define GPIOA_CRH      (*((volatile uint32_t*)0x40010804))
#define GPIOA_CRL      (*((volatile uint32_t*)0x40010800))
#define GPIOA_ODR      (*((volatile uint32_t*)0x4001080C))
#define GPIOA_IDR      (*((volatile uint32_t*)0x40010808))

#define USART1_SR      (*((volatile uint32_t*)0x40013800))
#define USART1_DR      (*((volatile uint32_t*)0x40013804))
#define USART1_BRR     (*((volatile uint32_t*)0x40013808))
#define USART1_CR1     (*((volatile uint32_t*)0x4001380C))

#define TIM1_CR1       (*((volatile uint32_t*)0x40012C00))
#define TIM1_CCMR1     (*((volatile uint32_t*)0x40012C18))
#define TIM1_CCMR2     (*((volatile uint32_t*)0x40012C1C))
#define TIM1_CCER      (*((volatile uint32_t*)0x40012C20))
#define TIM1_ARR       (*((volatile uint32_t*)0x40012C2C))
#define TIM1_CCR1      (*((volatile uint32_t*)0x40012C34))
#define TIM1_CCR4      (*((volatile uint32_t*)0x40012C40))
#define TIM1_PSC       (*((volatile uint32_t*)0x40012C28))

// Битовые маски
#define USART_SR_RXNE  (1 << 5)
#define USART_SR_TXE   (1 << 7)

// Структура команды
typedef struct {
    int16_t steer;  // -1000 до 1000
    int16_t speed;  // -1000 до 1000
} motor_command_t;

// Глобальные переменные
motor_command_t current_command = {0, 0};
uint8_t rx_buffer[4];
uint8_t rx_index = 0;

// Функция задержки
void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 8000; i++);
}

// Инициализация GPIO
void gpio_init(void) {
    // Включаем тактирование GPIOA
    RCC_APB2ENR |= (1 << 2);
    
    // PA9 (TX) - альтернативная функция, push-pull, 50MHz
    GPIOA_CRH &= ~(0xF << 4);
    GPIOA_CRH |= (0xB << 4);
    
    // PA10 (RX) - вход с подтяжкой
    GPIOA_CRH &= ~(0xF << 8);
    GPIOA_CRH |= (0x8 << 8);
    
    // PA8 (PWM1) - альтернативная функция
    GPIOA_CRH &= ~(0xF << 0);
    GPIOA_CRH |= (0xB << 0);
    
    // PA11 (PWM2) - альтернативная функция
    GPIOA_CRH &= ~(0xF << 12);
    GPIOA_CRH |= (0xB << 12);
    
    // PA12, PA15 - выходы для направления
    GPIOA_CRH &= ~(0xF << 16);
    GPIOA_CRH |= (0x3 << 16);
    GPIOA_CRH &= ~(0xF << 28);
    GPIOA_CRH |= (0x3 << 28);
}

// Инициализация UART1
void uart_init(void) {
    // Включаем тактирование USART1
    RCC_APB2ENR |= (1 << 14);
    
    // Настройка скорости (115200 baud при 72MHz)
    // BRR = 72MHz / (16 * 115200) = 39.0625
    USART1_BRR = 39 << 4;  // 39.0625 * 16 = 625
    
    // Включаем USART, прием и передачу
    USART1_CR1 = (1 << 13) | (1 << 3) | (1 << 2);
}

// Инициализация таймера для PWM
void timer_init(void) {
    // Включаем тактирование TIM1
    RCC_APB2ENR |= (1 << 11);
    
    // Настройка предделителя и периода
    TIM1_PSC = 72 - 1;  // 1MHz
    TIM1_ARR = 1000 - 1; // 1kHz PWM
    
    // Настройка каналов PWM
    TIM1_CCMR1 |= (0x6 << 4);  // PWM mode 1 для канала 1
    TIM1_CCMR2 |= (0x6 << 4);  // PWM mode 1 для канала 4
    
    // Включаем каналы
    TIM1_CCER |= (1 << 0);  // CC1E
    TIM1_CCER |= (1 << 12); // CC4E
    
    // Включаем таймер
    TIM1_CR1 |= (1 << 0);
}

// Отправка байта через UART
void uart_send_byte(uint8_t byte) {
    while (!(USART1_SR & USART_SR_TXE));
    USART1_DR = byte;
}

// Отправка строки через UART
void uart_send_string(const char* str) {
    while (*str) {
        uart_send_byte(*str++);
    }
}

// Чтение байта из UART
uint8_t uart_read_byte(void) {
    while (!(USART1_SR & USART_SR_RXNE));
    return USART1_DR;
}

// Установка скорости мотора
void set_motor_speed(int16_t left_speed, int16_t right_speed) {
    // Ограничение значений
    if (left_speed > 1000) left_speed = 1000;
    if (left_speed < -1000) left_speed = -1000;
    if (right_speed > 1000) right_speed = 1000;
    if (right_speed < -1000) right_speed = -1000;
    
    // Установка направления (PA12, PA15)
    if (left_speed >= 0) {
        GPIOA_ODR |= (1 << 12);  // PA12 = 1
    } else {
        GPIOA_ODR &= ~(1 << 12); // PA12 = 0
        left_speed = -left_speed;
    }
    
    if (right_speed >= 0) {
        GPIOA_ODR |= (1 << 15);  // PA15 = 1
    } else {
        GPIOA_ODR &= ~(1 << 15); // PA15 = 0
        right_speed = -right_speed;
    }
    
    // Установка PWM
    TIM1_CCR1 = left_speed;
    TIM1_CCR4 = right_speed;
}

// Обработка принятой команды
void process_command(void) {
    // Преобразуем в команды для моторов
    // Простая модель: speed - линейная скорость, steer - разность скоростей
    int16_t left_speed = current_command.speed - current_command.steer/2;
    int16_t right_speed = current_command.speed + current_command.steer/2;
    
    set_motor_speed(left_speed, right_speed);
    
    // Отправляем подтверждение
    uart_send_string("OK\r\n");
}

// Основная функция
int main(void) {
    // Инициализация
    gpio_init();
    uart_init();
    timer_init();
    
    // Отправляем сообщение о готовности
    uart_send_string("STM32 Motor Controller Ready\r\n");
    
    // Основной цикл
    while (1) {
        // Проверяем наличие данных в UART
        if (USART1_SR & USART_SR_RXNE) {
            uint8_t byte = USART1_DR;
            
            // Сохраняем байт в буфер
            rx_buffer[rx_index] = byte;
            rx_index++;
            
            // Если получили 4 байта, обрабатываем команду
            if (rx_index >= 4) {
                // Извлекаем команду
                current_command.steer = rx_buffer[0] | (rx_buffer[1] << 8);
                current_command.speed = rx_buffer[2] | (rx_buffer[3] << 8);
                
                // Обрабатываем команду
                process_command();
                
                // Сбрасываем индекс
                rx_index = 0;
            }
        }
        
        // Небольшая задержка
        delay_ms(1);
    }
}

// Обработчики прерываний (пустые для простоты)
void SysTick_Handler(void) {
    // Пустой обработчик
}

void USART1_IRQHandler(void) {
    // Пустой обработчик
}
