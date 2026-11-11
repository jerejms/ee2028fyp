#include "oled.h"

#include <stdio.h>
#include <string.h>

/*
 * Use the I2C handle already declared in main.c.
 * This does not create another handle.
 */
extern I2C_HandleTypeDef hi2c1;

/* Stores the image before sending it to the OLED. */
static uint8_t oledBuffer[OLED_WIDTH * OLED_PAGES];

/* Your friend's uppercase letter font. */
static const uint8_t oledFont[26][5] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x41, 0x41, 0x7F, 0x41, 0x41}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}  // Z
};

static const uint8_t oledDigits[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}  // 9
};


/* -------------------------------------------------------
 * Send an OLED command
 * ------------------------------------------------------- */

static HAL_StatusTypeDef OLED_Command(uint8_t cmd)
{
    uint8_t data[2] = {0x80, cmd};

    return HAL_I2C_Master_Transmit(
        &hi2c1,
        OLED_ADDR,
        data,
        sizeof(data),
        100
    );
}


/* -------------------------------------------------------
 * Initialise OLED
 * ------------------------------------------------------- */

HAL_StatusTypeDef OLED_Init(void)
{
    /* Initialisation sequence from your friend's file. */
    const uint8_t commands[] = {
        0xAE,       // Display OFF
        0xD5, 0x80, // Clock divide
        0xA8, 0x3F, // Multiplex ratio
        0xD3, 0x00, // Display offset
        0x40,       // Start line
        0x8D, 0x14, // Charge pump
        0xA1,       // Segment remap
        0xC8,       // COM scan direction
        0xDA, 0x12, // COM pins
        0x81, 0x7F, // Contrast
        0xD9, 0xF1, // Precharge
        0xDB, 0x40, // VCOMH
        0xA4,       // Display follows RAM
        0xA6,       // Normal display
        0xAF        // Display ON
    };

    for (uint32_t i = 0; i < sizeof(commands); i++)
    {
        HAL_StatusTypeDef status = OLED_Command(commands[i]);

        if (status != HAL_OK)
        {
            return status;
        }
    }

    return HAL_OK;
}


/* -------------------------------------------------------
 * Clear the drawing buffer
 * ------------------------------------------------------- */

void OLED_Clear(void)
{
    memset(oledBuffer, 0, sizeof(oledBuffer));
}


/* -------------------------------------------------------
 * Draw one pixel
 * ------------------------------------------------------- */

void OLED_DrawPixel(int x, int y, uint8_t color)
{
    if (x < 0 || x >= OLED_WIDTH ||
        y < 0 || y >= OLED_HEIGHT)
    {
        return;
    }

    uint16_t index = x + (y / 8) * OLED_WIDTH;
    uint8_t mask = (uint8_t)(1U << (y % 8));

    if (color)
    {
        oledBuffer[index] |= mask;
    }
    else
    {
        oledBuffer[index] &= (uint8_t)~mask;
    }
}


/* -------------------------------------------------------
 * Send the drawing buffer to the OLED
 * ------------------------------------------------------- */

HAL_StatusTypeDef OLED_Update(void)
{
    HAL_StatusTypeDef status;

    uint8_t setup[] = {
        0x00, 0xA4, 0xA6
    };

    status = HAL_I2C_Master_Transmit(
        &hi2c1,
        OLED_ADDR,
        setup,
        sizeof(setup),
        100
    );

    if (status != HAL_OK)
    {
        return status;
    }

    uint8_t packet[OLED_WIDTH + 1];
    packet[0] = 0x40;

    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        uint8_t pageCmd[] = {
            0x00,
            (uint8_t)(0xB0 | page),
            0x00,
            0x10
        };

        status = HAL_I2C_Master_Transmit(
            &hi2c1,
            OLED_ADDR,
            pageCmd,
            sizeof(pageCmd),
            100
        );

        if (status != HAL_OK)
        {
            return status;
        }

        memcpy(
            &packet[1],
            &oledBuffer[page * OLED_WIDTH],
            OLED_WIDTH
        );

        status = HAL_I2C_Master_Transmit(
            &hi2c1,
            OLED_ADDR,
            packet,
            sizeof(packet),
            200
        );

        if (status != HAL_OK)
        {
            return status;
        }
    }

    return HAL_OK;
}


/* -------------------------------------------------------
 * Draw one character
 * ------------------------------------------------------- */

void OLED_DrawChar(int x, int y, char c)
{
    if (c == '.')
    {
        OLED_DrawPixel(x + 2, y + 6, 1);
        return;
    }

    if (c == '-')
    {
        OLED_DrawLine(x, y + 3, x + 4, y + 3);
        return;
    }
    if (c == ' ')
    {
        return;
    }

    if (c >= 'a' && c <= 'z')
    {
        c -= 32;
    }

    const uint8_t *glyph;

    if (c >= 'A' && c <= 'Z')
    {
        glyph = oledFont[c - 'A'];
    }
    else if (c >= '0' && c <= '9')
    {
        glyph = oledDigits[c - '0'];
    }
    else
    {
        return;
    }

    for (int col = 0; col < 5; col++)
    {
        for (int row = 0; row < 7; row++)
        {
            uint8_t pixel = (glyph[col] >> row) & 1U;

            OLED_DrawPixel(
                x + col,
                y + row,
                pixel
            );
        }
    }
}


/* -------------------------------------------------------
 * Draw a string
 * ------------------------------------------------------- */

void OLED_DrawString(int x, int y, const char *str)
{
    while (*str != '\0')
    {
        OLED_DrawChar(x, y, *str);

        x += 6;
        str++;
    }
}


/* -------------------------------------------------------
 * Draw a line
 * ------------------------------------------------------- */

void OLED_DrawLine(int x0, int y0, int x1, int y1)
{
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (1)
    {
        OLED_DrawPixel(x0, y0, 1);

        if (x0 == x1 && y0 == y1)
        {
            break;
        }

        int e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}


/* -------------------------------------------------------
 * Small charge bar at bottom of screen
 * ------------------------------------------------------- */

void OLED_DrawCompactChargeBar(int charge)
{
    if (charge < 0) charge = 0;
    if (charge > 10) charge = 10;

    for (int segment = 0; segment < 10; segment++)
    {
        int x = 5 + segment * 12;

        for (int dy = 0; dy < 7; dy++)
        {
            for (int dx = 0; dx < 10; dx++)
            {
                int border =
                    dx == 0 ||
                    dx == 9 ||
                    dy == 0 ||
                    dy == 6;

                int filled =
                    segment < charge &&
                    dx > 1 &&
                    dx < 8 &&
                    dy > 1 &&
                    dy < 5;

                OLED_DrawPixel(
                    x + dx,
                    54 + dy,
                    (border || filled) ? 1 : 0
                );
            }
        }
    }
}


/* -------------------------------------------------------
 * Rescue countdown bar
 * ------------------------------------------------------- */

void OLED_DrawRescueBar(int x, int y, int seconds)
{
    const int width = 118;
    const int height = 10;

    if (seconds < 0) seconds = 0;
    if (seconds > 10) seconds = 10;

    for (int dy = 0; dy < height; dy++)
    {
        for (int dx = 0; dx < width; dx++)
        {
            int border =
                dx == 0 ||
                dx == width - 1 ||
                dy == 0 ||
                dy == height - 1;

            int fillWidth = (seconds * (width - 2)) / 10;

            int filled =
                dx > 0 &&
                dx <= fillWidth &&
                dy > 0 &&
                dy < height - 1;

            OLED_DrawPixel(
                x + dx,
                y + dy,
                (border || filled) ? 1 : 0
            );
        }
    }
}

/* -------------------------------------------------------
 * Laser animation
 * elapsed is milliseconds since this animation started.
 * ------------------------------------------------------- */

void OLED_DrawLaserAnimation(uint32_t elapsed)
{
    /* Laser emitter */
    OLED_DrawLine(5, 26, 24, 26);
    OLED_DrawLine(5, 38, 24, 38);
    OLED_DrawLine(24, 26, 32, 32);
    OLED_DrawLine(24, 38, 32, 32);

    if (elapsed < 60)
    {
        /* Muzzle flash */
        OLED_DrawLine(33, 32, 45, 20);
        OLED_DrawLine(33, 32, 45, 44);
        OLED_DrawLine(33, 32, 50, 32);
    }
    else if (elapsed < 180)
    {
        /* Laser travels left to right */
        int beamX = 40 + (int)((elapsed - 60) * 75 / 120);

        OLED_DrawLine(33, 31, beamX, 31);
        OLED_DrawLine(33, 32, beamX, 32);
        OLED_DrawLine(33, 33, beamX, 33);

        OLED_DrawLine(beamX, 27, beamX + 5, 32);
        OLED_DrawLine(beamX, 37, beamX + 5, 32);
    }
    else if (elapsed < 260)
    {
        /* Impact explosion */
        OLED_DrawLine(112, 32, 125, 20);
        OLED_DrawLine(112, 32, 125, 44);
        OLED_DrawLine(112, 32, 100, 18);
        OLED_DrawLine(112, 32, 100, 46);
        OLED_DrawLine(98, 32, 126, 32);
        OLED_DrawLine(112, 16, 112, 48);
    }
}

// Called while elapsed is below the 800 ms shot duration.
void OLED_DrawLaserAnimation(uint32_t elapsed)
{
    int x = 12 + (int)(elapsed * 90U / 800U);

    OLED_DrawLine(x - 10, 32, x + 10, 32);
}


void OLED_ShowAttacked(int secondsLeft, uint8_t warningVisible)
{
    char text[16];

    OLED_Clear();

    OLED_DrawString(5, 4, "DRONE CONTROL");

    if (warningVisible)
    {
        OLED_DrawString(5, 16, "ATTACKED");
    }

    OLED_DrawString(5, 28, "RESCUE REQUIRED");

    snprintf(text, sizeof(text), "%d SEC", secondsLeft);
    OLED_DrawString(5, 43, text);

    OLED_DrawRescueBar(5, 54, secondsLeft);
    OLED_Update();
}
