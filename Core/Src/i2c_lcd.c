/** 
 * I2C LCD Library for STM32 
 * Object-oriented version with multiple LCD support
 */

#include "i2c_lcd.h"

/**
 * @brief  Sends a command to the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  cmd: Command byte to send
 * @retval None
 */
void lcd_send_cmd(I2C_LCD_HandleTypeDef *lcd, char cmd)
{
    char upper_nibble, lower_nibble;
    uint8_t data_t[4];

    upper_nibble = (cmd & 0xF0);            // Extract upper nibble
    lower_nibble = ((cmd << 4) & 0xF0);     // Extract lower nibble
    
    data_t[0] = upper_nibble | 0x0C;  // RS=0, EN=1, BL=1
    data_t[1] = upper_nibble | 0x08;  // RS=0, EN=0, BL=1
    data_t[2] = lower_nibble | 0x0C;  // RS=0, EN=1, BL=1
    data_t[3] = lower_nibble | 0x08;  // RS=0, EN=0, BL=1

    HAL_I2C_Master_Transmit(lcd->hi2c, lcd->address, data_t, 4, 100);
}

/**
 * @brief  Sends data (character) to the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  data: Data byte to send
 * @retval None
 */
void lcd_send_data(I2C_LCD_HandleTypeDef *lcd, char data)
{
    char upper_nibble, lower_nibble;
    uint8_t data_t[4];

    upper_nibble = (data & 0xF0);            // Extract upper nibble
    lower_nibble = ((data << 4) & 0xF0);     // Extract lower nibble
    
    data_t[0] = upper_nibble | 0x0D;  // RS=1, EN=1, BL=1
    data_t[1] = upper_nibble | 0x09;  // RS=1, EN=0, BL=1
    data_t[2] = lower_nibble | 0x0D;  // RS=1, EN=1, BL=1
    data_t[3] = lower_nibble | 0x09;  // RS=1, EN=0, BL=1

    HAL_I2C_Master_Transmit(lcd->hi2c, lcd->address, data_t, 4, 100);
}

/**
 * @brief  Clears the LCD display.
 * @param  lcd: Pointer to the LCD handle
 * @retval None
 */
void lcd_clear(I2C_LCD_HandleTypeDef *lcd)
{
    lcd_send_cmd(lcd, 0x80);  // Move cursor to the home position
    // Clear all characters
    // 16x4 = 64 characters
    // 20x4 = 80 characters
    // So 80 character clearing is enough for both 16x2, 16x4, 20x2 and 20x5 displays
    for (int i = 0; i < 80; i++)
    {
        lcd_send_data(lcd, ' ');  // Write a space on each position
    }
}

/**
 * @brief  Moves the cursor to a specific position on the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  col: Column number (0-15)
 * @param  row: Row number (0 or 1)
 * @retval None
 */
void lcd_gotoxy(I2C_LCD_HandleTypeDef *lcd, int col, int row)
{
    uint8_t address;

    switch (row)
    {
        case 0: address = 0x80 + col; break;  // First row
        case 1: address = 0xC0 + col; break;  // Second row
        case 2: address = 0x94 + col; break;  // Third row
        case 3: address = 0xD4 + col; break;  // Fourth row
        default: return;  // Ignore invalid row numbers
    }

    lcd_send_cmd(lcd, address);  // Send command to move the cursor
}

/**
 * @brief  Initializes the LCD in 4-bit mode.
 * @param  lcd: Pointer to the LCD handle
 * @retval None
 */
void lcd_init(I2C_LCD_HandleTypeDef *lcd)
{
    HAL_Delay(50);  // Wait for LCD power-up
    lcd_send_cmd(lcd, 0x30);  // Wake up command
    HAL_Delay(5);
    lcd_send_cmd(lcd, 0x30);  // Wake up command
    HAL_Delay(1);
    lcd_send_cmd(lcd, 0x30);  // Wake up command
    HAL_Delay(10);
    lcd_send_cmd(lcd, 0x20);  // Set to 4-bit mode
    HAL_Delay(10);

    // LCD configuration commands
    lcd_send_cmd(lcd, 0x28);  // 4-bit mode, 2 lines, 5x8 font
    HAL_Delay(1);
    lcd_send_cmd(lcd, 0x08);  // Display off, cursor off, blink off
    HAL_Delay(1);
    lcd_send_cmd(lcd, 0x01);  // Clear display
    HAL_Delay(2);
    lcd_send_cmd(lcd, 0x06);  // Entry mode: cursor moves right
    HAL_Delay(1);
    lcd_send_cmd(lcd, 0x0C);  // Display on, cursor off, blink off
}

/**
 * @brief  Sends a string to the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  str: Null-terminated string to display
 * @retval None
 */
void lcd_puts(I2C_LCD_HandleTypeDef *lcd, char *str)
{
    while (*str) lcd_send_data(lcd, *str++);  // Send each character in the string
}

/**
 * @brief  Sends a single character to the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  ch: Character to send
 * @retval None
 */
void lcd_putchar(I2C_LCD_HandleTypeDef *lcd, char ch)
{
    lcd_send_data(lcd, ch);  // Send the character to the display
}

