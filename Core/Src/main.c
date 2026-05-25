#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* CẤU HÌNH VÀ BIẾN */
#define MOVING_AVERAGE_SIZE 5
#define I2C_LCD_ADDR (0x27 << 1)
#define UART_BAUDRATE 9600U
#define I2C_TIMEOUT 100000U

typedef enum { BOOT_MODE = 0, AUTO_MODE, MANUAL_MODE } Mode_t;
typedef enum { STATE_SAFE = 0, STATE_WARNING, STATE_DANGER } Safety_t;

volatile Mode_t current_mode = BOOT_MODE;
volatile Safety_t safety_state = STATE_SAFE;

volatile uint32_t rawDistance = 150;
volatile float filtered_distance = 150.0f;
float previous_distance = 150.0f;

uint32_t last_sensor_time = 0;
uint32_t last_uart_time = 0;
uint32_t last_lcd_time = 0;
uint32_t last_velocity_time = 0;
uint32_t last_boot_tick = 0;
uint32_t brake_start_time = 0;

volatile uint8_t is_braking_active = 0;
float current_speed_m_s = 0.0f;
volatile float d_safe = 30.0f;

volatile char rx_buffer = 0;
volatile uint8_t rx_flag = 0;
volatile int32_t boot_countdown = 10;

uint8_t hal_rx_byte = 0;

/* ========================================================================== */
/* PROTOTYPES */
/* ========================================================================== */
void Timebase_Init_RegisterLevel(void);
uint32_t Millis(void);
void Delay_ms(uint32_t ms);
void Delay_us(volatile uint32_t au32microseconds);

void GPIO_Init_RegisterLevel(void);
void Timers_Init(void);
void Local_MX_USART1_UART_Init(void);
void Local_MX_I2C1_Init(void);

void LCD_Init(void);
void LCD_Write(uint8_t data, uint8_t control_bits);
void LCD_Send_Cmd(char cmd);
void LCD_Send_Data(char data);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print_Fixed(const char *str);
void LCD_Print_Num(uint32_t num);

void UART1_PollReceive(void);
void UART_SendString(const char *str);
void Set_Motor(int8_t speed_A, int8_t speed_B);
void Set_Hardware_Alerts(Safety_t state);
void Update_Safety_And_Motors(void);
const char* Get_Safety_Text(Safety_t state);

/* ========================================================================== */
/* TIMEBASE - DWT */
/* ========================================================================== */
void Timebase_Init_RegisterLevel(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t Millis(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000U);
}

void Delay_ms(uint32_t ms) {
    uint32_t start = Millis();
    while ((Millis() - start) < ms);
}

void Delay_us(volatile uint32_t au32microseconds) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = au32microseconds * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

/* ========================================================================== */
/* INIT FUNCTIONS */
/* ========================================================================== */
void GPIO_Init_RegisterLevel(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    /* PA0: Buzzer, PA1: TRIG, PA5: LED, PA6: ECHO/TIM3_CH1 */
    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER0_Pos); GPIOA->MODER |= (1 << GPIO_MODER_MODER0_Pos);
    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER1_Pos); GPIOA->MODER |= (1 << GPIO_MODER_MODER1_Pos);
    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER5_Pos); GPIOA->MODER |= (1 << GPIO_MODER_MODER5_Pos);

    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER6_Pos); GPIOA->MODER |= (2 << GPIO_MODER_MODER6_Pos);
    GPIOA->AFR[0] &= ~(0xF << (6*4)); GPIOA->AFR[0] |= (2 << (6*4));

    /* PWM PA8, PA9 */
    GPIOA->MODER &= ~((3 << GPIO_MODER_MODER8_Pos) | (3 << GPIO_MODER_MODER9_Pos));
    GPIOA->MODER |= (2 << GPIO_MODER_MODER8_Pos) | (2 << GPIO_MODER_MODER9_Pos);
    GPIOA->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOA->AFR[1] |= (1 << 0) | (1 << 4);

    /* UART1: PB6 TX - PA10 RX */
    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER10_Pos); GPIOA->MODER |= (2 << GPIO_MODER_MODER10_Pos);
    GPIOA->AFR[1] &= ~(0xF << (10*4 - 32)); GPIOA->AFR[1] |= (7 << (10*4 - 32));

    GPIOB->MODER &= ~(3 << GPIO_MODER_MODER6_Pos); GPIOB->MODER |= (2 << GPIO_MODER_MODER6_Pos);
    GPIOB->AFR[0] &= ~(0xF << (6*4)); GPIOB->AFR[0] |= (7 << (6*4));

    GPIOA->OSPEEDR |= (3 << (10*2));
    GPIOB->OSPEEDR |= (3 << (6*2));

    /* LED PC6, PC8, PC9 */
    GPIOC->MODER &= ~((3 << (6*2)) | (3 << (8*2)) | (3 << (9*2)));
    GPIOC->MODER |= ((1 << (6*2)) | (1 << (8*2)) | (1 << (9*2)));

    /* TB6612: PC0, PC1, PC2, PC3 */
    GPIOC->MODER &= ~((3 << (0*2)) | (3 << (1*2)) | (3 << (2*2)) | (3 << (3*2)));
    GPIOC->MODER |= ((1 << (0*2)) | (1 << (1*2)) | (1 << (2*2)) | (1 << (3*2)));

    /* I2C1: PB7 SDA, PB8 SCL */
    GPIOB->MODER &= ~((3 << (7*2)) | (3 << (8*2)));
    GPIOB->MODER |= ((2 << (7*2)) | (2 << (8*2)));
    GPIOB->OTYPER |= (1 << 7) | (1 << 8);
    GPIOB->OSPEEDR |= (3 << (7*2)) | (3 << (8*2));
    GPIOB->PUPDR &= ~((3 << (7*2)) | (3 << (8*2)));
    GPIOB->PUPDR |= ((1 << (7*2)) | (1 << (8*2)));
    GPIOB->AFR[0] &= ~(0xF << 28); GPIOB->AFR[0] |= (4 << 28);
    GPIOB->AFR[1] &= ~(0xF << 0);  GPIOB->AFR[1] |= (4 << 0);
}

void Timers_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 159; TIM1->ARR = 99;
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 15; TIM3->ARR = 0xFFFF;
    TIM3->CCMR1 |= (1 << TIM_CCMR1_CC1S_Pos);
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;
    TIM3->DIER |= TIM_DIER_CC1IE;
    NVIC_SetPriority(TIM3_IRQn, 0);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;
}

void Local_MX_USART1_UART_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    USART1->BRR = (SystemCoreClock + (UART_BAUDRATE / 2U)) / UART_BAUDRATE;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;
}

void Local_MX_I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    uint32_t pclk1_mhz = SystemCoreClock / 1000000U;
    if (pclk1_mhz < 2U) pclk1_mhz = 2U;

    I2C1->CR2 = pclk1_mhz;
    I2C1->CCR = SystemCoreClock / (2U * 100000U);
    if (I2C1->CCR < 4U) I2C1->CCR = 4U;
    I2C1->TRISE = pclk1_mhz + 1U;

    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_PE;
}

/* ========================================================================== */
/* LCD I2C */
/* ========================================================================== */
static uint8_t I2C1_Wait_SR1_Set(uint32_t flag) {
    uint32_t timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & flag)) {
        if (--timeout == 0) return 0;
    }
    return 1;
}

static uint8_t I2C1_Wait_SR2_Clear(uint32_t flag) {
    uint32_t timeout = I2C_TIMEOUT;
    while (I2C1->SR2 & flag) {
        if (--timeout == 0) return 0;
    }
    return 1;
}

static void I2C1_Write(uint8_t dev_addr, uint8_t *data, uint16_t len) {
    volatile uint32_t temp;

    if (!I2C1_Wait_SR2_Clear(I2C_SR2_BUSY)) return;

    I2C1->CR1 |= I2C_CR1_START;
    if (!I2C1_Wait_SR1_Set(I2C_SR1_SB)) return;

    I2C1->DR = dev_addr & 0xFEU;
    if (!I2C1_Wait_SR1_Set(I2C_SR1_ADDR)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return;
    }

    temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void)temp;

    for (uint16_t i = 0; i < len; i++) {
        if (!I2C1_Wait_SR1_Set(I2C_SR1_TXE)) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return;
        }
        I2C1->DR = data[i];
    }

    if (!I2C1_Wait_SR1_Set(I2C_SR1_BTF)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return;
    }

    I2C1->CR1 |= I2C_CR1_STOP;
}

void LCD_Write(uint8_t data, uint8_t control_bits) {
    uint8_t data_u = data & 0xF0;
    uint8_t data_l = (data << 4) & 0xF0;
    uint8_t pkt[4] = {
        data_u | control_bits | 0x0C,
        data_u | control_bits | 0x08,
        data_l | control_bits | 0x0C,
        data_l | control_bits | 0x08
    };
    I2C1_Write(I2C_LCD_ADDR, pkt, 4);
}

void LCD_Send_Cmd(char cmd)  { LCD_Write(cmd, 0x00); }
void LCD_Send_Data(char data){ LCD_Write(data, 0x01); }

void LCD_Init(void) {
    Delay_ms(50);
    LCD_Send_Cmd(0x02); Delay_ms(10);
    LCD_Send_Cmd(0x28); Delay_ms(5);
    LCD_Send_Cmd(0x0C); Delay_ms(5);
    LCD_Send_Cmd(0x01); Delay_ms(20);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    LCD_Send_Cmd((row == 0) ? (0x80 + col) : (0xC0 + col));
}

void LCD_Print_Fixed(const char *str) {
    uint8_t i = 0;
    while (str[i] != '\0' && i < 16) LCD_Send_Data(str[i++]);
    while (i < 16) { LCD_Send_Data(' '); i++; }
}

void LCD_Print_Num(uint32_t num) {
    char buf[8]; int i = 0;
    if (num == 0) { LCD_Send_Data('0'); return; }
    while (num && i < 8) {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }
    while (i--) LCD_Send_Data(buf[i]);
}

/* ========================================================================== */
/* UART & MOTOR & SAFETY */
/* ========================================================================== */
static void UART1_SendChar(char ch) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint8_t)ch;
}

void UART_SendString(const char *str) {
    while (*str) {
        UART1_SendChar(*str++);
    }
    while (!(USART1->SR & USART_SR_TC));
}

void UART1_PollReceive(void) {
    if (USART1->SR & USART_SR_RXNE) {
        char temp_char = (char)(USART1->DR & 0xFF);
        hal_rx_byte = (uint8_t)temp_char;

        if (temp_char >= 32 && temp_char <= 126) {
            rx_buffer = temp_char;
            rx_flag = 1;

            char debug[20];
            sprintf(debug, "RX:%c\r\n", temp_char);
            UART_SendString(debug);
        }
    }
}

void Set_Motor(int8_t speed_A, int8_t speed_B) {
    if (speed_A > 0) { GPIOC->BSRR = (1 << 0) | (1 << (1 + 16)); TIM1->CCR1 = speed_A; }
    else if (speed_A < 0) { GPIOC->BSRR = (1 << 1) | (1 << (0 + 16)); TIM1->CCR1 = -speed_A; }
    else { GPIOC->BSRR = (1 << (0 + 16)) | (1 << (1 + 16)); TIM1->CCR1 = 0; }

    if (speed_B > 0) { GPIOC->BSRR = (1 << 2) | (1 << (3 + 16)); TIM1->CCR2 = speed_B; }
    else if (speed_B < 0) { GPIOC->BSRR = (1 << 3) | (1 << (2 + 16)); TIM1->CCR2 = -speed_B; }
    else { GPIOC->BSRR = (1 << (2 + 16)) | (1 << (3 + 16)); TIM1->CCR2 = 0; }
}

const char* Get_Safety_Text(Safety_t state) {
    if (state == STATE_SAFE) return "SAFE";
    else if (state == STATE_WARNING) return "WARN";
    else return "DANGER";
}

void Set_Hardware_Alerts(Safety_t state) {
    GPIOC->BSRR = (1 << (6 + 16)) | (1 << (8 + 16)) | (1 << (9 + 16));
    GPIOA->BSRR = (1 << (0 + 16));
    if (state == STATE_SAFE) GPIOC->BSRR = (1 << 6);
    else if (state == STATE_WARNING) GPIOC->BSRR = (1 << 8);
    else if (state == STATE_DANGER) {
        GPIOC->BSRR = (1 << 9);
        GPIOA->BSRR = (1 << 0);
    }
}

void Update_Safety_And_Motors(void) {
    float cur_d = filtered_distance;
    if (cur_d >= 100.0f) safety_state = STATE_SAFE;
    else if (cur_d >= 50.0f) safety_state = STATE_WARNING;
    else safety_state = STATE_DANGER;

    Set_Hardware_Alerts(safety_state);

    if (current_mode == AUTO_MODE) {
        if (cur_d <= d_safe) {
            if (!is_braking_active) {
                is_braking_active = 1;
                brake_start_time = Millis();
            }
            if (Millis() - brake_start_time <= 120)
                Set_Motor(-70, -70);
            else
                Set_Motor(0, 0);
        } else {
            is_braking_active = 0;
            TIM1->CCR1 = 40;
            TIM1->CCR2 = 40;
            GPIOC->BSRR = (1u<<0)|(1u<<(1+16))|(1u<<2)|(1u<<(3+16));
        }
    }
}

/* ========================================================================== */
/* INTERRUPTS */
/* ========================================================================== */
void TIM3_IRQHandler(void) {
    static uint32_t t_start = 0;
    if (TIM3->SR & TIM_SR_CC1IF) {
        uint32_t t_capture = TIM3->CCR1;
        if (GPIOA->IDR & GPIO_IDR_IDR_6) {
            t_start = t_capture;
        } else {
            uint32_t t_echo = (t_capture >= t_start) ? (t_capture - t_start) : (0xFFFF - t_start + t_capture + 1);
            rawDistance = t_echo / 58;

            static float buf[MOVING_AVERAGE_SIZE] = {150,150,150,150,150};
            static uint8_t idx = 0;
            if (rawDistance > 0 && rawDistance < 400) {
                buf[idx] = (float)rawDistance;
                idx = (idx + 1) % MOVING_AVERAGE_SIZE;
            }
            float sum = 0;
            for(int i = 0; i < MOVING_AVERAGE_SIZE; i++) sum += buf[i];
            filtered_distance = sum / MOVING_AVERAGE_SIZE;

            Update_Safety_And_Motors();
        }
        TIM3->SR &= ~TIM_SR_CC1IF;
    }
}

/* ========================================================================== */
/* MAIN */
/* ========================================================================== */
int main(void) {
    Timebase_Init_RegisterLevel();
    GPIO_Init_RegisterLevel();
    Timers_Init();
    Local_MX_I2C1_Init();
    Local_MX_USART1_UART_Init();

    LCD_Init();

    UART_SendString("\r\n==================================\r\n");
    UART_SendString(" ROBOT ADAS CORE ONLINE!\r\n");
    UART_SendString("==================================\r\n");
    UART_SendString("BOOT MODE - Send 'A' or 'M' to start\r\n");

    while (1) {
        uint32_t current_time = Millis();

        UART1_PollReceive();

        /* Sensor Trigger */
        if (current_time - last_sensor_time >= 60) {
            GPIOA->BSRR = GPIO_BSRR_BS1;
            Delay_us(10);
            GPIOA->BSRR = GPIO_BSRR_BR1;
            last_sensor_time = current_time;
        }

        /* Tính d_safe */
        if (current_time - last_velocity_time >= 1000) {
            float delta_d = previous_distance - filtered_distance;
            current_speed_m_s = (delta_d > 0.1f) ? (delta_d / 100.0f) : 0.0f;
            float calculated = (current_speed_m_s * 0.5f) + (current_speed_m_s * current_speed_m_s / 5.0f);
            d_safe = calculated * 100.0f;
            if (d_safe < 22.0f) d_safe = 22.0f;
            if (d_safe > 45.0f) d_safe = 45.0f;
            previous_distance = filtered_distance;
            last_velocity_time = current_time;
        }

        /* ==================== XỬ LÝ CHẾ ĐỘ MANUAL (LUÔN QUÉT AN TOÀN) ==================== */
        if (current_mode == MANUAL_MODE) {

            uint8_t has_new_cmd = 0;
            char cmd = 0;

            if (rx_flag) {
                rx_flag = 0;
                cmd = rx_buffer;
                rx_buffer = 0;
                has_new_cmd = 1;
            }

            if (filtered_distance <= d_safe) {

                if (has_new_cmd) {
                    if (cmd == 'C' || cmd == 'c') {
                        current_mode = AUTO_MODE;
                    }
                    else if (cmd == 'A' || cmd == 'a') {
                        /* Bỏ qua lệnh tiến khi khoảng cách nguy hiểm */
                    }
                    else if (cmd == 'B' || cmd == 'b') {
                        Set_Motor(-50, -50);
                        is_braking_active = 0;
                    }
                    else if (cmd == 'R' || cmd == 'r') {
                        Set_Motor(-80, 80);
                        is_braking_active = 0;
                    }
                    else if (cmd == 'L' || cmd == 'l') {
                        Set_Motor(80, -80);
                        is_braking_active = 0;
                    }
                    else if (cmd == 'S' || cmd == 's') {
                        Set_Motor(0, 0);
                        is_braking_active = 0;
                    }
                }

                if (cmd == 0 || cmd == 'A' || cmd == 'a') {
                    if (!is_braking_active) {
                        is_braking_active = 1;
                        brake_start_time = Millis();
                    }
                    if (Millis() - brake_start_time <= 120) {
                        Set_Motor(-70, -70);
                    } else {
                        Set_Motor(0, 0);
                    }
                }
            }
            else {
                is_braking_active = 0;

                if (has_new_cmd) {
                    if (cmd == 'C' || cmd == 'c') {
                        current_mode = AUTO_MODE;
                    }
                    else if (cmd == 'A' || cmd == 'a') {
                        Set_Motor(40, 40);
                    }
                    else if (cmd == 'B' || cmd == 'b') {
                        Set_Motor(-40, -40);
                    }
                    else if (cmd == 'R' || cmd == 'r') {
                        Set_Motor(-50, 50);
                    }
                    else if (cmd == 'L' || cmd == 'l') {
                        Set_Motor(50, -50);
                    }
                    else if (cmd == 'S' || cmd == 's') {
                        Set_Motor(0, 0);
                    }
                }
            }
        }

        /* ==================== XỬ LÝ LỆNH BLUETOOTH CHO CÁC CHẾ ĐỘ CÒN LẠI ==================== */
        if (rx_flag && current_mode != MANUAL_MODE) {
            rx_flag = 0;
            char cmd = rx_buffer;
            rx_buffer = 0;

            if (current_mode == BOOT_MODE) {
                if (cmd == 'M' || cmd == 'm') current_mode = MANUAL_MODE;
                else if (cmd == 'A' || cmd == 'a') current_mode = AUTO_MODE;
            }
            else if (current_mode == AUTO_MODE) {
                if (cmd == 'C' || cmd == 'c') current_mode = MANUAL_MODE;
            }
        }

        /* Boot Mode */
        if (current_mode == BOOT_MODE) {
            Set_Motor(0, 0);
            if (current_time - last_boot_tick >= 1000) {
                boot_countdown--;
                last_boot_tick = current_time;
                if (boot_countdown <= 0) current_mode = AUTO_MODE;
            }
        }

        /* LCD Display */
        if (current_time - last_lcd_time >= 300) {
            LCD_SetCursor(0, 0);
            if (current_mode == BOOT_MODE)
                LCD_Print_Fixed("MODE: BOOT ");
            else if (current_mode == AUTO_MODE)
                LCD_Print_Fixed("MODE: AUTO ");
            else
                LCD_Print_Fixed("MODE: MANUAL ");

            LCD_SetCursor(1, 0);
            if (current_mode == BOOT_MODE) {
                LCD_Print_Fixed("WAIT: ");
                LCD_SetCursor(1, 6);
                LCD_Print_Num(boot_countdown);
                LCD_Send_Data('s');
            } else {
                char lcd_line[17];
                snprintf(lcd_line, sizeof(lcd_line), "DIST:%lucm %s",
                         (unsigned long)((uint32_t)filtered_distance),
                         Get_Safety_Text(safety_state));
                LCD_Print_Fixed(lcd_line);
            }
            last_lcd_time = current_time;
        }
    }
}
