/*****************************************************************************
 * | File      	:	LCD_GUI.h
 * | Author      :   Waveshare team
 * | Function    :	Achieve drawing: draw points, lines, boxes,
 *circles and their size, solid dotted line, solid rectangle hollow
 *					rectangle, solid circle hollow
 *circle. | Info        : Achieve display characters: Display a single
 *character, string, number Achieve time display: adaptive size
 *display time minutes and seconds
 *----------------
 * |	This version:   V1.0
 * | Date        :   2017-08-16
 * | Info        :   Basic version
 *
 ******************************************************************************/

/****************************Upper application
 * layer**************************/
#ifndef __LCD_GUI_H
#define __LCD_GUI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "LCD_Driver.h"
#include "fonts.h"

#define LOW_Speed_Show 0
#define HIGH_Speed_Show 1
/********************************************************************************
function:
                        dot pixel
********************************************************************************/
typedef enum
{
    DOT_PIXEL_1X1 = 1, // dot pixel 1 x 1
    DOT_PIXEL_2X2,     // dot pixel 2 X 2
    DOT_PIXEL_3X3,     // dot pixel 3 X 3
    DOT_PIXEL_4X4,     // dot pixel 4 X 4
    DOT_PIXEL_5X5,     // dot pixel 5 X 5
    DOT_PIXEL_6X6,     // dot pixel 6 X 6
    DOT_PIXEL_7X7,     // dot pixel 7 X 7
    DOT_PIXEL_8X8,     // dot pixel 8 X 8
} DOT_PIXEL;
#define DOT_PIXEL_DFT DOT_PIXEL_1X1 // Default dot pilex

/********************************************************************************
function:
                        dot Fill style
********************************************************************************/
typedef enum
{
    DOT_FILL_AROUND = 1, // dot pixel 1 x 1
    DOT_FILL_RIGHTUP,    // dot pixel 2 X 2
} DOT_STYLE;
#define DOT_STYLE_DFT DOT_FILL_AROUND // Default dot pilex
/********************************************************************************
function:
                        solid line and dotted line
********************************************************************************/
typedef enum
{
    LINE_SOLID = 0,
    LINE_DOTTED,
} LINE_STYLE;

/********************************************************************************
function:
                        DRAW Internal fill
********************************************************************************/
typedef enum
{
    DRAW_EMPTY = 0,
    DRAW_FULL,
} DRAW_FILL;

/********************************************************************************
function:
        time
********************************************************************************/
typedef struct
{
    uint16_t Year; // 0000
    uint8_t Month; // 1 - 12
    uint8_t Day;   // 1 - 30
    uint8_t Hour;  // 0 - 23
    uint8_t Min;   // 0 - 59
    uint8_t Sec;   // 0 - 59
} DEV_TIME;
extern DEV_TIME sDev_time;

/********************************************************************************
function:
                        Defines commonly used colors for the display
********************************************************************************/
#define LCD_BACKGROUND WHITE  // Default background color
#define FONT_BACKGROUND WHITE // Default font background color
#define FONT_FOREGROUND GRED  // Default font foreground color

#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40
#define BRRED 0XFC07
#define GRAY 0X8430

/********************************************************************************
function:
                        Macro definition variable name
********************************************************************************/
// Clear
void GUI_Clear(COLOR Color);

// Drawing
void GUI_DrawPoint(POINT Xpoint, POINT Ypoint, COLOR Color, DOT_PIXEL Dot_Pixel,
                   DOT_STYLE Dot_FillWay);
void GUI_DrawLine(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,
                  COLOR Color, LINE_STYLE Line_Style, DOT_PIXEL Dot_Pixel);
void GUI_DrawRectangle(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,
                       COLOR Color, DRAW_FILL Filled, DOT_PIXEL Dot_Pixel);
void GUI_DrawCircle(POINT X_Center, POINT Y_Center, LENGTH Radius, COLOR Color,
                    DRAW_FILL Draw_Fill, DOT_PIXEL Dot_Pixel);

// Picture
void GUI_Disbitmap(POINT Xpoint, POINT Ypoint, const unsigned char* pMap,
                   POINT Width, POINT Height);
void GUI_DisGrayMap(POINT Xpoint, POINT Ypoint, const unsigned char* pBmp);

// Display string
void GUI_DisChar(POINT Xstart, POINT Ystart, const char Acsii_Char, sFONT* Font,
                 COLOR Color_Background, COLOR Color_Foreground);
void GUI_DisString_EN(POINT Xstart, POINT Ystart, const char* pString,
                      sFONT* Font, COLOR Color_Background,
                      COLOR Color_Foreground);
void GUI_DisNum(POINT Xpoint, POINT Ypoint, int32_t Nummber, sFONT* Font,
                COLOR Color_Background, COLOR Color_Foreground);
void GUI_Showtime(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,
                  DEV_TIME* pTime, COLOR Color);
// show
void GUI_WaveShare_Show(void);

/********************************************************************************
 *              Additional functionality added by Norbert Ligas
 * https://github.com/LigasN
 ********************************************************************************/

// Libraries
#include <stdarg.h>
#include <string.h>

/// Max number of textboxes in GUI
#define TEXTBOXES_NUMBER 3
/// Max number of consoles in GUI
#define CONSOLES_NUMBER 1
/// Max number of chars in GUI_Console's text
#define CONSOLE_TEXT_SIZE 52 * 7

/// Simple text object with possibilities to:
///  - setup start and end points of it,
///  - refresh once placed text with new one (GUI_RefreshTextBox()),
///  - use of different fonts,
///  - setup different fore and background colors
typedef struct
{
    POINT xPos;
    POINT yPos;
    POINT xEnd;
    POINT yEnd;
    COLOR backgroundColor;
    COLOR foregroundColor;
    sFONT* font;
    char* text;
} GUI_TextBox;

/// Text object with possibilities to:
///  - setup start and end points of it,
///  - use of different fonts,
///  - setup different fore and background colors
///  - print next line of text like in a console:
///     - printOnConsole()
///     - printfOnConsole()
typedef struct
{
    POINT xPos;
    POINT yPos;
    POINT xEnd;
    POINT yEnd;
    COLOR backgroundColor;
    COLOR foregroundColor;
    sFONT* font;
    char text[CONSOLE_TEXT_SIZE];
} GUI_Console;

/// Simple window object which holds all GUI data together. Usable with
/// GUI_DrawGUI() to draw whole GUI at once.
typedef struct
{
    COLOR background;
    GUI_TextBox textboxes[TEXTBOXES_NUMBER];
    GUI_Console consoles[CONSOLES_NUMBER];
} GUI_Window;

// Functions
// Descriptions for all functions are placed in LCD_GUI.c file, because Eclipse
// somehow prefers this way to enable showing hints during coding.
void GUI_DisStringInBox(POINT Xbegin, POINT Ybegin, POINT Xend, POINT Yend,
                        const char* pString, sFONT* Font,
                        COLOR Color_Background, COLOR Color_Foreground);
void GUI_DrawImage(POINT xPoint, POINT yPoint, const unsigned char* image_data);
void GUI_RefreshTextBox(const GUI_TextBox* t);
void printOnConsole(GUI_Console* c, char* text);
void printfOnConsole(GUI_Console* c, const char* text, ...);
void GUI_DrawGUI(const GUI_Window* w);

#ifdef __cplusplus
}
#endif

#endif
