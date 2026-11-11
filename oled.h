#define OLED_ADDR   (0x3C << 1)
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_PAGES  (OLED_HEIGHT / 8)

HAL_StatusTypeDef OLED_Init(void);
HAL_StatusTypeDef OLED_Update(void);

void OLED_Clear(void);
void OLED_DrawPixel(int x, int y, uint8_t color);
void OLED_DrawChar(int x, int y, char c);
void OLED_DrawString(int x, int y, const char *str);
void OLED_DrawLine(int x0, int y0, int x1, int y1);

void OLED_DrawCompactChargeBar(int charge);
void OLED_DrawRescueBar(int x, int y, int seconds);

void OLED_DrawLaserAnimation(uint32_t elapsed);
void OLED_ShowAttacked(int secondsLeft, uint8_t warningVisible);

#endif
