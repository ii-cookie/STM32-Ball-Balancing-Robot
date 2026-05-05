#include "bsp_ov7725.h"
#include "bsp_sccb.h"
#include "lcd.h"
#include <stdio.h>

// Define a structure for register information
typedef struct Reg
{
    uint8_t Address;
    uint8_t Value;
} Reg_Info;

//Sensor config
Reg_Info Sensor_Config[] =
{
	{CLKRC, 0x00}, /*clock config*/
	{COM7, 0x46}, /*QVGA RGB565*/
	{HSTART, 0x3f},
	{HSIZE, 0x50},
	{VSTRT, 0x03},
	{VSIZE, 0x78},
	{HREF, 0x00},
	{HOutSize, 0x14},	// CHANGED TO 80
	{VOutSize, 0x1E},	// CHANGED to 60
	{EXHCH, 0x00},

	/*DSP control*/
	{TGT_B, 0x7f},
	{FixGain, 0x09},
	{AWB_Ctrl0, 0xE0},
	{DSP_Ctrl1, 0xff},
	{DSP_Ctrl2, 0x0f},	// enable down sampling and zoom out
	{DSP_Ctrl3, 0x00},
	{DSP_Ctrl4, 0x00},

	/*AGC AEC AWB - Manual brightness (disable auto)*/
	{COM8, 0x00}, /* disable AGC/AEC auto modes*/
	{AEW, 0x14}, /* fixed exposure window*/
	{AEB, 0x14}, /* fixed exposure window end */
	{VPT, 0x20}, /* fixed exposure time (vertical blanking) */
	{BRIGHT, 0x00}, /* fixed brightness offset*/
	{USAT, 0x80},
	{VSAT, 0x65},
	{DSPAuto, 0x00}, /*turn off DSP auto-brightness*/
	{AWBCtrl3, 0xaa},
	{AWBCtrl1, 0x5d},

	{EDGE1, 0x0a},
	{DNSOff, 0x01},
	{EDGE2, 0x01},
	{EDGE3, 0x01},
	{MTX1, 0x5f},
	{MTX2, 0x53},
	{MTX3, 0x11},
	{MTX4, 0x1a},
	{MTX5, 0x3d},
	{MTX6, 0x5a},
	{MTX_Ctrl, 0x1e},
	{BRIGHT, 0x00},
	{CNST, 0x25},
	{USAT, 0x65},
	{VSAT, 0x65},
	{UVADJ0, 0x81},
	{SDE, 0x06},

	{SCAL0, 0x0A},	// DOWN SAMPLING 1/4 vertical, 1/4 horizontal

	/*GAMMA config*/
	{GAM1, 0x0c},
	{GAM2, 0x16},
	{GAM3, 0x2a},
	{GAM4, 0x4e},
	{GAM5, 0x61},
	{GAM6, 0x6f},
	{GAM7, 0x7b},
	{GAM8, 0x86},
	{GAM9, 0x8e},
	{GAM10, 0x97},
	{GAM11, 0xa4},
	{GAM12, 0xaf},
	{GAM13, 0xc5},
	{GAM14, 0xd7},
	{GAM15, 0xe8},
	{SLOP, 0x20},
	{HUECOS, 0x80},
	{HUESIN, 0x80},
	{DSPAuto, 0xff},
	{DM_LNL, 0x00},
	{BDBase, 0x99},
	{BDMStep, 0x03},
	{LC_RADI, 0x00},
	{LC_COEF, 0x13},
	{LC_XC, 0x08},
	{LC_COEFB, 0x14},
	{LC_COEFR, 0x17},
	{LC_CTR, 0x05},
	{COM3, 0xd0},/*Horizontal mirror image*/
	/*night mode auto frame rate control*/
	{COM5, 0xf5}, /*auto reduce rate*/
};

// Number of registers to configure
uint8_t OV7725_REG_NUM = sizeof(Sensor_Config) / sizeof(Sensor_Config[0]);
extern uint8_t Ov7725_vsync;
#define IMG_WIDTH 80
#define IMG_HEIGHT 60
// Global centroid for PID
uint16_t centroid_x = 37;
uint16_t centroid_y = 37;
uint32_t orange_pixel_count = 0;

/************************************************
 * Sensor_Init
 ************************************************/
ErrorStatus Ov7725_Init(void)
{
    uint16_t i = 0;
    uint8_t Sensor_IDCode = 0;
    if(0 == SCCB_WriteByte(0x12, 0x80)) /* reset sensor */
    {
        return ERROR;
    }
    if(0 == SCCB_ReadByte(&Sensor_IDCode, 1, 0x0B)) /* read sensor ID */
    {
        return ERROR;
    }
    // DEBUG("Sensor ID is 0x%x", Sensor_IDCode);
    if(Sensor_IDCode == OV7725_ID)
    {
        for(i = 0; i < OV7725_REG_NUM; i++)
        {
            if(0 == SCCB_WriteByte(Sensor_Config[i].Address, Sensor_Config[i].Value))
            {
                return ERROR;
            }
        }
    }
    else
    {
        return ERROR;
    }
    return SUCCESS;
}

static inline uint8_t is_orange(uint16_t pix)
{
    const uint8_t r5 = (pix >> 11) & 0x1F;
    const uint8_t g6 = (pix >> 5) & 0x3F;
    const uint8_t b5 = pix & 0x1F;

    const uint8_t R = (r5 << 3) | (r5 >> 2);
    const uint8_t G = (g6 << 2) | (g6 >> 4);
    const uint8_t B = (b5 << 3) | (b5 >> 2);

    if (R <= G + 20 || R <= B + 20)
        return 0;

    // Manhattan distance to target (255,165,0)
    int dr = 255 - R;
    int dg = 165 - G;
    int db = 0 - B;
    int dist = (dr > 0 ? dr : -dr) + (dg > 0 ? dg : -dg) + (db > 0 ? db : -db);

    return (dist < 100); // threshold
}

void ImagDisp(void)
{
    uint16_t i, j;
    uint16_t Camera_Data;
    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    uint32_t count = 0;
    LCD_Cam_Gram();

    for (i = 0; i < IMG_HEIGHT; i++) {
        for (j = 0; j < IMG_WIDTH; j++) {
            READ_FIFO_PIXEL(Camera_Data);
            if (is_orange(Camera_Data)) {
                LCD_Write_Data(0xF800);  // highlight orange pixels in red
                sum_x += j;
                sum_y += i;
                count++;
            } else {
                LCD_Write_Data(Camera_Data);
            }
        }
    }

    if (count > 50) {
        centroid_x = (uint16_t)(sum_x / count);
        centroid_y = (uint16_t)(sum_y / count);
        orange_pixel_count = count;

        LCD_Clear(centroid_x - 2, centroid_y - 2, 4, 4, GREEN);
        LCD_Clear(centroid_x + 80, centroid_y, 2, 2, BLACK);

        char buf[40];
        sprintf(buf, "Orange: %lu", count);
        LCD_DrawString(4, 105, (uint8_t *)buf);
    } else {
        orange_pixel_count = 0;
        LCD_DrawString(4, 105, (uint8_t *)"No orange   ");
    }
}
