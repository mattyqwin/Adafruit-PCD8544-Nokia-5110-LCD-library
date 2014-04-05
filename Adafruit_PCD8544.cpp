/*********************************************************************
This is a library for our Monochrome Nokia 5110 LCD Displays

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/338

These displays use SPI to communicate, 4 or 5 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

//#include <Wire.h>
#include <avr/pgmspace.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include <util/delay.h>
#include <stdlib.h>

#include <Adafruit_GFX.h>
#include "Adafruit_PCD8544.h"

// the memory buffer for the LCD
uint8_t pcd8544_buffer[LCDWIDTH * LCDHEIGHT / 8] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFC, 0xFE, 0xFF, 0xFC, 0xE0,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8,
0xF8, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80, 0xC0, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x7F,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC7, 0xC7, 0x87, 0x8F, 0x9F, 0x9F, 0xFF, 0xFF, 0xFF,
0xC1, 0xC0, 0xE0, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC, 0xFC, 0xFE, 0xFE, 0xFE,
0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xE0, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x80, 0xC0, 0xE0, 0xF1, 0xFB, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x1F, 0x0F, 0x0F, 0x87,
0xE7, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x3F, 0xF9, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xFD, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x07, 0x01, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xF0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
0x7E, 0x3F, 0x3F, 0x0F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFC, 0xF0, 0xE0, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFC, 0xF0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x01,
0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0F, 0x1F, 0x3F, 0x7F, 0x7F,
0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};


// reduces how much is refreshed, which speeds it up!
// originally derived from Steve Evans/JCW's mod but cleaned up and
// optimized
//#define enablePartialUpdate

#ifdef enablePartialUpdate
static uint8_t xUpdateMin, xUpdateMax, yUpdateMin, yUpdateMax;
#endif



static void updateBoundingBox(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax) {
#ifdef enablePartialUpdate
  if (xmin < xUpdateMin) xUpdateMin = xmin;
  if (xmax > xUpdateMax) xUpdateMax = xmax;
  if (ymin < yUpdateMin) yUpdateMin = ymin;
  if (ymax > yUpdateMax) yUpdateMax = ymax;
#endif
}

Adafruit_PCD8544::Adafruit_PCD8544(int8_t SCLK, int8_t DIN, int8_t DC,
    int8_t CS, int8_t RST) : Adafruit_GFX(LCDWIDTH, LCDHEIGHT) {
  _din = DIN;
  _sclk = SCLK;
  _dc = DC;
  _rst = RST;
  _cs = CS;
}

Adafruit_PCD8544::Adafruit_PCD8544(int8_t SCLK, int8_t DIN, int8_t DC,
    int8_t RST) : Adafruit_GFX(LCDWIDTH, LCDHEIGHT) {
  _din = DIN;
  _sclk = SCLK;
  _dc = DC;
  _rst = RST;
  _cs = -1;
}


// the most basic function, set a single pixel
void Adafruit_PCD8544::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= LCDWIDTH) || (y < 0) || (y >= LCDHEIGHT))
    return;

  // x is which column
  if (color) 
    pcd8544_buffer[x+ (y/8)*LCDWIDTH] |= _BV(y%8);  
  else
    pcd8544_buffer[x+ (y/8)*LCDWIDTH] &= ~_BV(y%8); 

  updateBoundingBox(x,y,x,y);
}

// the most basic function, get a single pixel
uint8_t Adafruit_PCD8544::getPixel(int8_t x, int8_t y) {
  if ((x < 0) || (x >= LCDWIDTH) || (y < 0) || (y >= LCDHEIGHT))
    return 0;

  return (pcd8544_buffer[x + (y/8) * LCDWIDTH] >> (y%8)) & 0x1;  
}

// capture a raw bitmap buffer into memory(SRAM)
void Adafruit_PCD8544::capture(uint8_t *bitmap) {
  memcpy(bitmap, pcd8544_buffer, LCDWIDTH * LCDHEIGHT/8);
}

// blast a native bitmap into the display buffer from program memory
/*HACK: Unkown why 256 must be subtracted from program-memory pointer*/
void Adafruit_PCD8544::replace_P(uint8_t *bitmap) {
  memcpy_P(pcd8544_buffer,
           (uint8_t*) pgm_read_byte(bitmap - 256),
           LCDWIDTH * LCDHEIGHT/8); 
  updateBoundingBox(0, 0, LCDWIDTH-1, LCDHEIGHT-1);
}

// blast a native bitmap into the display buffer from working memory(SRAM)
void Adafruit_PCD8544::replace_S(uint8_t *bitmap) {
  memcpy(pcd8544_buffer, bitmap, LCDWIDTH * LCDHEIGHT/8);
  updateBoundingBox(0, 0, LCDWIDTH-1, LCDHEIGHT-1);
}

// scroll screen 1 pixel to the left. 
void Adafruit_PCD8544::scrollLeft(boolean WRAP) {
  uint8_t row, /*col, */ tmp;
  /*uint16_t ix;*/
  for (row=0; row < LCDHEIGHT/8; row++) {
    tmp = pcd8544_buffer[row * LCDWIDTH];         // remember 1st byte
    memcpy(pcd8544_buffer + (row * LCDWIDTH),
           pcd8544_buffer + (row * LCDWIDTH) + 1, // 1 line fast but ugly
           LCDWIDTH-1);
//     for (col=0;col<LCDWIDTH-1;col++) { 
//       ix=row*LCDWIDTH+col;                     // 3 lines readable but slower
//       pcd8544_buffer[ix] = pcd8544_buffer[ix + 1]; 
//     }
    // place wrapped byte if required
    pcd8544_buffer[row * LCDWIDTH + LCDWIDTH-1] = WRAP ? tmp : 0x00;
  }
  updateBoundingBox(0, 0, LCDWIDTH-1, LCDHEIGHT-1);
}

// scroll screen 1 pixel to the right
void Adafruit_PCD8544::scrollRight(boolean WRAP) {
  uint8_t row, col, tmp;
  uint16_t ix;
  for (row=0; row < LCDHEIGHT/8 ; row++) {
    tmp = pcd8544_buffer[row * LCDWIDTH + LCDWIDTH-1];  // remember 1st byte
    for (col = (LCDWIDTH-1); col > 0; col--) {
      ix=row * LCDWIDTH + col;
      pcd8544_buffer[ix] = pcd8544_buffer[ix-1];
    }
    pcd8544_buffer[row * LCDWIDTH] = WRAP ? tmp : 0x00; // place wrapped byte
  }
  updateBoundingBox(0, 0, LCDWIDTH-1, LCDHEIGHT-1);
}

// scroll screen 1 pixel up
void Adafruit_PCD8544::scrollUp(boolean WRAP) {
  uint8_t row, col, carrybit, carryflag=0, carry[((LCDWIDTH-1) / 8) + 1];
  uint16_t ix;
  
  memset(&carry[0],0,((LCDWIDTH-1) / 8) + 1); // clear carry bits
  // move from the bottom row backwards to 0
  for (row= LCDHEIGHT/8 ; row > 0; row--) {
    for (col=0; col < LCDWIDTH; col++) {
      ix=(uint16_t)(((row-1) * LCDWIDTH) + col);
      carrybit = 0x01 << col%8;
      if (row < LCDHEIGHT/8 ) {        // 2nd-highest row and below
        // retrieve and place carry bit from last(higher) row
        if (carry[col/8] & carrybit) { // carry is set
            carryflag = 1;
            carry[col/8] &= ~carrybit; // reset this carry bit
        } else {
            carryflag = 0;
        }
      } 
      // remember soon-to-be shifted bit
      if (pcd8544_buffer[ix] & 0x01) { // carry LSb is set
        carry[col/8] |= carrybit;      //  remember LSb(top bit)
      }
      pcd8544_buffer[ix] >>= 1;        // shift right(MSb->LSb(up)) 1 bit
      if(carryflag) {           
        pcd8544_buffer[ix] |= 0x80;    // place carry in MSb
      }
    } // for col
  } // for row 
  // wrap around shifted-up bits
  if (WRAP) {
    for (col=0; col < LCDWIDTH; col++) {
      //ix = col+420;
      ix = col + ((LCDWIDTH * LCDHEIGHT/8) - LCDWIDTH);
      carrybit = 0x1 << col%8;
      if (carry[col/8] & carrybit) {   // carry, set
        pcd8544_buffer[ix] |= 0x80;
      }
    }  
  } //if WRAP
  updateBoundingBox(0, 0, LCDWIDTH-1, LCDHEIGHT-1);
}

// scroll screen 1 pixel down
void Adafruit_PCD8544::scrollDown(boolean WRAP) {
  uint8_t row, col, carrybit, carryflag, carry[((LCDWIDTH-1) / 8) + 1];
  uint16_t ix;
  
  memset(&carry[0],0,((LCDWIDTH-1) / 8) + 1);              // clear carry bits
  // move from the bottom row( LCDHEIGHT/8 ) up to row 0
  for (row=0; row < LCDHEIGHT/8; row++) {
    for (col=0; col < LCDWIDTH; col++) {
      ix=(uint16_t)((row*LCDWIDTH) + col);
      carrybit = 1 << (col%8); 
      if(row <  LCDHEIGHT/8  && (carry[col/8] & carrybit)) { 
        // retrieve  carry bit from last(lower) row
        carryflag = 1;
        carry[col/8] &= ~carrybit;     // and reset it(ready for another)
      } else {
        carryflag = 0;
      }                       
      // remember soon-to-be shifted carrybit
      if ((pcd8544_buffer[ix] & 0x80)) {
        carry[col/8] |= carrybit;
      }
      pcd8544_buffer[ix] <<= 1;        // shift left(MSb<-LSb(down)) 1 bit
      // place carry
      if(carryflag) {
        pcd8544_buffer[ix] |= 0x01;    // place carry in MSb
      }
    } // for col
  } // for row 
  if (WRAP) {
      for (col=0 ;col < LCDWIDTH; col++) {
      ix = col;
      carrybit = 0x01 << col%8;
      if (carry[col/8] & carrybit) {   // carry: set pixel
        pcd8544_buffer[ix] |= 0x01;
      }
    }
  }
  updateBoundingBox(0, 0, LCDWIDTH-1, LCDHEIGHT-1);
}

void Adafruit_PCD8544::begin(uint8_t contrast) {
  // set pin directions
  pinMode(_din, OUTPUT);
  pinMode(_sclk, OUTPUT);
  pinMode(_dc, OUTPUT);
  if (_rst > 0)
    pinMode(_rst, OUTPUT);
  if (_cs > 0)
    pinMode(_cs, OUTPUT);

  // toggle RST low to reset
  if (_rst > 0) {
    digitalWrite(_rst, LOW);
    _delay_ms(500);
    digitalWrite(_rst, HIGH);
  }

  clkport     = portOutputRegister(digitalPinToPort(_sclk));
  clkpinmask  = digitalPinToBitMask(_sclk);
  mosiport    = portOutputRegister(digitalPinToPort(_din));
  mosipinmask = digitalPinToBitMask(_din);
  csport    = portOutputRegister(digitalPinToPort(_cs));
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  dcpinmask = digitalPinToBitMask(_dc);

  // get into the EXTENDED mode!
  command(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION );

  // LCD bias select (4 is optimal?)
  command(PCD8544_SETBIAS | 0x4);

  // set VOP
  if (contrast > 0x7f)
    contrast = 0x7f;

  command( PCD8544_SETVOP | contrast); // Experimentally determined


  // normal mode
  command(PCD8544_FUNCTIONSET);

  // Set display to Normal
  command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);

  // initial display line
  // set page address
  // set column address
  // write display data

  // set up a bounding box for screen updates

  updateBoundingBox(0, 0, LCDWIDTH-1, LCDHEIGHT-1);
  // Push out pcd8544_buffer to the Display (will show the AFI logo)
  display();
}


inline void Adafruit_PCD8544::fastSPIwrite(uint8_t d) {
  
  for(uint8_t bit = 0x80; bit; bit >>= 1) {
    *clkport &= ~clkpinmask;
    if(d & bit) *mosiport |=  mosipinmask;
    else        *mosiport &= ~mosipinmask;
    *clkport |=  clkpinmask;
  }
}

inline void Adafruit_PCD8544::slowSPIwrite(uint8_t c) {
  shiftOut(_din, _sclk, MSBFIRST, c);
}

void Adafruit_PCD8544::command(uint8_t c) {
  digitalWrite(_dc, LOW);
  if (_cs > 0)
    digitalWrite(_cs, LOW);
  fastSPIwrite(c);
  if (_cs > 0)
    digitalWrite(_cs, HIGH);
}

void Adafruit_PCD8544::data(uint8_t c) {
  digitalWrite(_dc, HIGH);
  if (_cs > 0)
    digitalWrite(_cs, LOW);
  fastSPIwrite(c);
  if (_cs > 0)
    digitalWrite(_cs, HIGH);
}

void Adafruit_PCD8544::setContrast(uint8_t val) {
  if (val > 0x7f) {
    val = 0x7f;
  }
  command(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION );
  command( PCD8544_SETVOP | val); 
  command(PCD8544_FUNCTIONSET);
  
 }



void Adafruit_PCD8544::display(void) {
  uint8_t col, maxcol, p;
  
  for(p = 0; p < ((LCDHEIGHT-1) / 8) + 1; p++) {
#ifdef enablePartialUpdate
    // check if this page is part of update
    if ( yUpdateMin >= ((p+1)*8) ) {
      continue;   // nope, skip it!
    }
    if (yUpdateMax < p*8) {
      break;
    }
#endif

    command(PCD8544_SETYADDR | p);


#ifdef enablePartialUpdate
    col = xUpdateMin;
    maxcol = xUpdateMax;
#else
    // start at the beginning of the row
    col = 0;
    maxcol = LCDWIDTH-1;
#endif

    command(PCD8544_SETXADDR | col);

    digitalWrite(_dc, HIGH);
    if (_cs > 0)
      digitalWrite(_cs, LOW);
    for(; col <= maxcol; col++) {
      //uart_putw_dec(col);
      //uart_putchar(' ');
      fastSPIwrite(pcd8544_buffer[(LCDWIDTH*p)+col]);
    }
    if (_cs > 0)
      digitalWrite(_cs, HIGH);

  }

  command(PCD8544_SETYADDR );  // no idea why this is necessary but it is to finish the last byte?
#ifdef enablePartialUpdate
  xUpdateMin = LCDWIDTH - 1;
  xUpdateMax = 0;
  yUpdateMin = LCDHEIGHT-1;
  yUpdateMax = 0;
#endif

}

// clear everything
void Adafruit_PCD8544::clearDisplay(void) {
  memset(pcd8544_buffer, 0, LCDWIDTH*LCDHEIGHT/8);
  updateBoundingBox(0, 0, LCDWIDTH-1, LCDHEIGHT-1);
  cursor_y = cursor_x = 0;
}

/*
// this doesnt touch the buffer, just clears the display RAM - might be handy
void Adafruit_PCD8544::clearDisplay(void) {
  
  uint8_t p, c;
  
  for(p = 0; p < 8; p++) {

    st7565_command(CMD_SET_PAGE | p);
    for(c = 0; c < 129; c++) {
      //uart_putw_dec(c);
      //uart_putchar(' ');
      st7565_command(CMD_SET_COLUMN_LOWER | (c & 0xf));
      st7565_command(CMD_SET_COLUMN_UPPER | ((c >> 4) & 0xf));
      st7565_data(0x0);
    }     
    }

}

*/
