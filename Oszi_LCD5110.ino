/* Nokia 5100 LCD Example Code
   Graphics driver and PCD8544 interface code for SparkFun's
   84x48 Graphic LCD.
   https://www.sparkfun.com/products/10168

  by: Jim Lindblom
    adapted from code by Nathan Seidle and mish-mashed with
    code from the ColorLCDShield.
  date: October 10, 2013
  license: Beerware. Feel free to use, reuse, and modify this
  code as you see fit. If you find it useful, and we meet someday,
  you can buy me a beer.

  This all-inclusive sketch will show off a series of graphics
  functions, like drawing lines, circles, squares, and text. Then
  it'll go into serial monitor echo mode, where you can type
  text into the serial monitor, and it'll be displayed on the
  LCD.

  This stuff could all be put into a library, but we wanted to
  leave it all in one sketch to keep it as transparent as possible.

  Hardware: (Note most of these pins can be swapped)
    Graphic LCD Pin ---------- Arduino Pin
       1-VCC       ----------------  5V
       2-GND       ----------------  GND
       3-SCE       ----------------  7
       4-RST       ----------------  6
       5-D/C       ----------------  5
       6-DN(MOSI)  ----------------  11
       7-SCLK      ----------------  13
       8-LED       - 330 Ohm res --  9
   The SCLK, DN(MOSI), must remain where they are, but the other
   pins can be swapped. The LED pin should remain a PWM-capable
   pin. Don't forget to stick a current-limiting resistor in line
   between the LCD's LED pin and Arduino pin 9!

   Additional
       Arduino Pin
       0 (TRIGGER) ---------------- 0/5V
       ANALOG 5    ---------------- POTI
       ANALOG 0    ---------------- SIGNAL IN (Test: Pin 2)  
*/
#include <SPI.h> // We'll use SPI to transfer data. Faster!

/* PCD8544-specific defines: */
#define LCD_COMMAND  0
#define LCD_DATA     1

/* 84x48 LCD Defines: */
#define LCD_WIDTH   84 // Note: x-coordinates go wide
#define LCD_HEIGHT  48 // Note: y-coordinates go high
#define WHITE       0  // For drawing pixels. A 0 draws white.
#define BLACK       1  // A 1 draws black.

/* Pin definitions:
Most of these pins can be moved to any digital or analog pin.
DN(MOSI)and SCLK should be left where they are (SPI pins). The
LED (backlight) pin should remain on a PWM-capable pin. */
const unsigned int scePin = 7;    // SCE - Chip select, pin 3 on LCD.
const unsigned int rstPin = 6;    // RST - Reset, pin 4 on LCD.
const unsigned int dcPin = 5;     // DC - Data/Command, pin 5 on LCD.
const unsigned int sdinPin = 11;  // DN(MOSI) - Serial data, pin 6 on LCD.
const unsigned int sclkPin = 13;  // SCLK - Serial clock, pin 7 on LCD.
const unsigned int blPin = 9;     // LED - Backlight LED, pin 8 on LCD.
const unsigned int triggerPin = 0;

/* global variables */
uint32_t sensorValue;
uint32_t sensorValueMax = 0;
uint32_t sensorValueMin = 0;
uint32_t sensorIntervall;
// this is time sensitive! If the buffer gets to large, 
// it is possible to get stuck in an interrupt loop.
const unsigned int BUFFERSIZE = 120;
unsigned int ringBuffer[BUFFERSIZE];
unsigned int screenBuffer[BUFFERSIZE];
const uint32_t sensorScale = 100;
unsigned int triggerValue;

/* The displayMap variable stores a buffer representation of the
pixels on our display. There are 504 total bits in this array,
same as how many pixels there are on a 84 x 48 display.

Each byte in this array covers a 8-pixel vertical block on the
display. Each successive byte covers the next 8-pixel column over
until you reach the right-edge of the display and step down 8 rows.

To update the display, we first have to write to this array, then
call the updateDisplay() function, which sends this whole array
to the PCD8544.

Because the PCD8544 won't let us write individual pixels at a
time, this is how we can make targeted changes to the display. */
byte displayMap[LCD_WIDTH * LCD_HEIGHT / 8] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,0)->(11,7) ~ These 12 bytes cover an 8x12 block in the left corner of the display
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,0)->(23,7)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, // (24,0)->(35,7)
  0xF0, 0xF8, 0xFC, 0xFC, 0xFE, 0xFE, 0xFE, 0xFE, 0x1E, 0x0E, 0x02, 0x00, // (36,0)->(47,7)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (48,0)->(59,7)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,0)->(71,7)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,0)->(83,7)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,8)->(11,15)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,8)->(23,15)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, // (24,8)->(35,15)
  0x0F, 0x1F, 0x3F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8, // (36,8)->(47,15)
  0xF8, 0xF0, 0xF8, 0xFE, 0xFE, 0xFC, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00, // (48,8)->(59,15)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,8)->(71,15)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,8)->(83,15)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,16)->(11,23)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,16)->(23,23)
  0x00, 0x00, 0xF8, 0xFC, 0xFE, 0xFE, 0xFF, 0xFF, 0xF3, 0xE0, 0xE0, 0xC0, // (24,16)->(35,23)
  0xC0, 0xC0, 0xE0, 0xE0, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // (36,16)->(47,23)
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3E, 0x00, 0x00, 0x00, // (48,16)->(59,23)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,16)->(71,23)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,16)->(83,23)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,24)->(11,31)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,24)->(23,31)
  0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // (24,24)->(35,31)
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // (36,24)->(47,31)
  0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x1F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, // (48,24)->(59,31)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,24)->(71,31)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,24)->(83,31)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,32)->(11,39)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,32)->(23,39)
  0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x1F, // (24,32)->(35,39)
  0x0F, 0x0F, 0x0F, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x03, // (36,32)->(47,39)
  0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (48,32)->(59,39)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,32)->(71,39)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,32)->(83,39)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,40)->(11,47)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,40)->(23,47)
  0x00, 0x00, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, // (24,40)->(35,47)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (36,40)->(47,47)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (48,40)->(59,47)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,40)->(71,47)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,40)->(83,47) !!! The bottom right pixel!
};

void setup() {
  //Serial.begin(9600);

  lcdBegin(); // This will setup our pins, and initialize the LCD
  setContrast(55); // Pretty good value, play around with it

  updateDisplay(); // with displayMap untouched, SFE logo
  delay(500);

  clearDisplay(WHITE);
  updateDisplay();

  analogReference(EXTERNAL);

  pinMode(triggerPin, INPUT);
  
  //start preferences
  setTimer0Freq(2000); //set sampling rate to 2ksps
  triggerValue = LCD_HEIGHT / 2;

  //test
  pinMode(2,OUTPUT);
}

void loop() {
  //not time sensitive
  calculateMinMax();

  //toggle testpin
  digitalWrite(2, !digitalRead(2));

  unsigned int freq = 3*analogRead(A5) + 1;
  setTimer0Freq(freq);

  int itrigger = 0;
  if(digitalRead(triggerPin) == HIGH) {
    itrigger = getTriggerIndex(ringBuffer, triggerValue, BUFFERSIZE);
  }
  
  // fill the screenBuffer only if we can show an entire data record
  if (itrigger + LCD_WIDTH <= BUFFERSIZE) {
    cli(); //disable interrupts, so that ringBuffer does not get modified during copying
    copyShiftBuffer(ringBuffer, screenBuffer, BUFFERSIZE, -itrigger);
    sei();
  }

  // display the record
  clearDisplay(WHITE);
  setRect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, false, true);
  setLine(0, LCD_HEIGHT / 2, LCD_WIDTH, LCD_HEIGHT / 2, true);
  for (int i = 0; i < LCD_WIDTH - 1; i++) {
    setLine(i, screenBuffer[i], i + 1, screenBuffer[i + 1], true);
  }
  updateDisplay();
}

void setTimer0Freq(const unsigned int timer0Freq) {
  cli();

  //setup timer0
  TCCR0A = 0; // set TCCR0A register to 0
  TCCR0B = 0; // set TCCR0B register to 0
  TCNT0  = 0; // set counter value to 0
  unsigned int cr;

  if(timer0Freq <= 3100 && timer0Freq > 980) { //mac 3.1kHz
    TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00); // Set CS01 and CS00 bits for 1:64 prescaler -> 16MHz/64 -> 250kHz
    cr = 250000 / timer0Freq; // set compare match register
  } else if(timer0Freq <= 980 && timer0Freq > 245) {
    TCCR0B |= (1 << CS02) | (0 << CS01) | (0 << CS00); // Set CS01 and CS00 bits for 1:256 prescaler -> 16MHz/256 -> 62.5kHz
    cr = 62500 / timer0Freq; // set compare match register
  } else if(timer0Freq <= 245 && timer0Freq > 61) {
    TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00); // Set CS01 and CS00 bits for 1:1024 prescaler -> 16MHz/1024 -> 15.625
    cr = 15625 / timer0Freq; // set compare match register
  }
  
  if(cr >= 255) cr = 255;
  if(cr <= 1) cr = 1;
  OCR0A = cr;

  TCCR0A |= (1 << WGM01); // turn on CTC mode
  TIMSK0 |= (1 << OCIE0A); // enable timer compare interrupt

  sei(); // allow interrupts
}

// timer0 interrupt
ISR(TIMER0_COMPA_vect) {
  cli();
  // sensor value gets rescaled to use only integer calculations
  sensorValue = analogRead(A0) * sensorScale;
  for (int i = 1; i < BUFFERSIZE; i++) {
    ringBuffer[i - 1] = ringBuffer[i];
  }
  ringBuffer[BUFFERSIZE - 1] = ((sensorValue / sensorIntervall) * LCD_HEIGHT) / sensorScale;
  sei();
}

void copyShiftBuffer(const unsigned int inBuff[], unsigned int outBuff[], const unsigned int len, const int shift) {
  if (shift < 0) { //shift to the left
    for (int i = 0; i < len; i++) {
      if (i < abs(shift))
        outBuff[len + i + shift] = inBuff[i]; // shift is negative
      if (i >= abs(shift))
        outBuff[i + shift] = inBuff[i];
    }
  } else { //shift to the right
    for (int i = 0; i < len; i++) {
      if (i < shift)
        outBuff[i] = inBuff[len + i - shift];
      if (i >= shift)
        outBuff[i] = inBuff[i - shift];
    }
  }
}

int getTriggerIndex(const unsigned int inBuff[], const int triggerValue, const unsigned int len) {
  //determine trigger point
  int itrigger = 0;
  for (int i = 0; i < len - 2; i++) {
    //find the rising edge
    if (inBuff[i] > triggerValue && inBuff[i + 2] < triggerValue) {
      itrigger = i;
      break;
    }
  }
  return itrigger;
}

void calculateMinMax() {
  if (sensorValue > sensorValueMax) {
    sensorValueMax = sensorValue;
  }
  if (sensorValue < sensorValueMin) {
    sensorValueMin = sensorValue;
  }
  // slow decay, to reduce the max and min values after some time -> auto scale
  unsigned int gamma = 5;
  sensorValueMax = (sensorValueMax / sensorScale * (sensorScale - gamma));
  sensorValueMin = (sensorValueMin / sensorScale * (sensorScale - gamma));

  sensorIntervall = (sensorValueMax - sensorValueMin) / sensorScale;
  if (sensorIntervall < 2) sensorIntervall = 1;
}

// Because I keep forgetting to put bw variable in when setting...
void setPixel(int x, int y) {
  setPixel(x, y, BLACK); // Call setPixel with bw set to Black
}

void clearPixel(int x, int y) {
  setPixel(x, y, WHITE); // call setPixel with bw set to white
}

// This function sets a pixel on displayMap to your preferred
// color. 1=Black, 0= white.
void setPixel(int x, int y, boolean bw) {
  // First, double check that the coordinate is in range.
  if ((x >= 0) && (x < LCD_WIDTH) && (y >= 0) && (y < LCD_HEIGHT))
  {
    byte shift = y % 8;

    if (bw) // If black, set the bit.
      displayMap[x + (y / 8)*LCD_WIDTH] |= 1 << shift;
    else   // If white clear the bit.
      displayMap[x + (y / 8)*LCD_WIDTH] &= ~(1 << shift);
  }
}

// setLine draws a line from x0,y0 to x1,y1 with the set color.
// This function was grabbed from the SparkFun ColorLCDShield
// library.
void setLine(int x0, int y0, int x1, int y1, boolean bw) {
  int dy = y1 - y0; // Difference between y0 and y1
  int dx = x1 - x0; // Difference between x0 and x1
  int stepx, stepy;

  if (dy < 0) {
    dy = -dy;
    stepy = -1;
  } else
    stepy = 1;

  if (dx < 0) {
    dx = -dx;
    stepx = -1;
  } else
    stepx = 1;

  dy <<= 1; // dy is now 2*dy
  dx <<= 1; // dx is now 2*dx
  setPixel(x0, y0, bw); // Draw the first pixel.

  if (dx > dy) {
    int fraction = dy - (dx >> 1);
    while (x0 != x1) {
      if (fraction >= 0) {
        y0 += stepy;
        fraction -= dx;
      }
      x0 += stepx;
      fraction += dy;
      setPixel(x0, y0, bw);
    }
  } else {
    int fraction = dx - (dy >> 1);
    while (y0 != y1) {
      if (fraction >= 0) {
        x0 += stepx;
        fraction -= dy;
      }
      y0 += stepy;
      fraction += dx;
      setPixel(x0, y0, bw);
    }
  }
}

// setRect will draw a rectangle from x0,y0 top-left corner to
// a x1,y1 bottom-right corner. Can be filled with the fill
// parameter, and colored with bw.
// This function was grabbed from the SparkFun ColorLCDShield
// library.
void setRect(int x0, int y0, int x1, int y1, boolean fill, boolean bw) {
  // check if the rectangle is to be filled
  if (fill == 1) {
    int xDiff;

    if (x0 > x1)
      xDiff = x0 - x1; //Find the difference between the x vars
    else
      xDiff = x1 - x0;

    while (xDiff > 0) {
      setLine(x0, y0, x0, y1, bw);

      if (x0 > x1)
        x0--;
      else
        x0++;

      xDiff--;
    }
  } else {
    // best way to draw an unfilled rectangle is to draw four lines
    setLine(x0, y0, x1, y0, bw);
    setLine(x0, y1, x1, y1, bw);
    setLine(x0, y0, x0, y1, bw);
    setLine(x1, y0, x1, y1, bw);
  }
}

// setCircle draws a circle centered around x0,y0 with a defined
// radius. The circle can be black or white. And have a line
// thickness ranging from 1 to the radius of the circle.
// This function was grabbed from the SparkFun ColorLCDShield
// library.
void setCircle (int x0, int y0, int radius, boolean bw, int lineThickness) {
  for (int r = 0; r < lineThickness; r++) {
    int f = 1 - radius;
    int ddF_x = 0;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;

    setPixel(x0, y0 + radius, bw);
    setPixel(x0, y0 - radius, bw);
    setPixel(x0 + radius, y0, bw);
    setPixel(x0 - radius, y0, bw);

    while (x < y) {
      if (f >= 0) {
        y--;
        ddF_y += 2;
        f += ddF_y;
      }
      x++;
      ddF_x += 2;
      f += ddF_x + 1;

      setPixel(x0 + x, y0 + y, bw);
      setPixel(x0 - x, y0 + y, bw);
      setPixel(x0 + x, y0 - y, bw);
      setPixel(x0 - x, y0 - y, bw);
      setPixel(x0 + y, y0 + x, bw);
      setPixel(x0 - y, y0 + x, bw);
      setPixel(x0 + y, y0 - x, bw);
      setPixel(x0 - y, y0 - x, bw);
    }
    radius--;
  }
}

// This function clears the entire display either white (0) or
// black (1).
// The screen won't actually clear until you call updateDisplay()!
void clearDisplay(boolean bw) {
  for (int i = 0; i < (LCD_WIDTH * LCD_HEIGHT / 8); i++) {
    if (bw)
      displayMap[i] = 0xFF;
    else
      displayMap[i] = 0;
  }
}

// Helpful function to directly command the LCD to go to a
// specific x,y coordinate.
void gotoXY(int x, int y) {
  LCDWrite(0, 0x80 | x);  // Column.
  LCDWrite(0, 0x40 | y);  // Row.  ?
}

// This will actually draw on the display, whatever is currently
// in the displayMap array.
void updateDisplay() {
  gotoXY(0, 0);
  for (int i = 0; i < (LCD_WIDTH * LCD_HEIGHT / 8); i++) {
    LCDWrite(LCD_DATA, displayMap[i]);
  }
}

// Set contrast can set the LCD Vop to a value between 0 and 127.
// 40-60 is usually a pretty good range.
void setContrast(byte contrast) {
  LCDWrite(LCD_COMMAND, 0x21); //Tell LCD that extended commands follow
  LCDWrite(LCD_COMMAND, 0x80 | contrast); //Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
  LCDWrite(LCD_COMMAND, 0x20); //Set display mode
}

/* There are two ways to do this. Either through direct commands
to the display, or by swapping each bit in the displayMap array.
We'll leave both methods here, comment one or the other out if
you please. */
void invertDisplay() {
  /* Direct LCD Command option
  LCDWrite(LCD_COMMAND, 0x20); //Tell LCD that extended commands follow
  LCDWrite(LCD_COMMAND, 0x08 | 0x05); //Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
  LCDWrite(LCD_COMMAND, 0x20); //Set display mode  */

  /* Indirect, swap bits in displayMap option: */
  for (int i = 0; i < (LCD_WIDTH * LCD_HEIGHT / 8); i++) {
    displayMap[i] = ~displayMap[i] & 0xFF;
  }
  updateDisplay();
}

// There are two memory banks in the LCD, data/RAM and commands.
// This function sets the DC pin high or low depending, and then
// sends the data byte
void LCDWrite(byte data_or_command, byte data) {
  //Tell the LCD that we are writing either to data or a command
  digitalWrite(dcPin, data_or_command);

  //Send the data
  digitalWrite(scePin, LOW);
  SPI.transfer(data); //shiftOut(sdinPin, sclkPin, MSBFIRST, data);
  digitalWrite(scePin, HIGH);
}

//This sends the magical commands to the PCD8544
void lcdBegin(void) {
  //Configure control pins
  pinMode(scePin, OUTPUT);
  pinMode(rstPin, OUTPUT);
  pinMode(dcPin, OUTPUT);
  pinMode(sdinPin, OUTPUT);
  pinMode(sclkPin, OUTPUT);
  pinMode(blPin, OUTPUT);
  analogWrite(blPin, 255);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  //Reset the LCD to a known state
  digitalWrite(rstPin, LOW);
  digitalWrite(rstPin, HIGH);

  LCDWrite(LCD_COMMAND, 0x21); //Tell LCD extended commands follow
  LCDWrite(LCD_COMMAND, 0xB0); //Set LCD Vop (Contrast)
  LCDWrite(LCD_COMMAND, 0x04); //Set Temp coefficent
  LCDWrite(LCD_COMMAND, 0x14); //LCD bias mode 1:48 (try 0x13)
  //We must send 0x20 before modifying the display control mode
  LCDWrite(LCD_COMMAND, 0x20);
  LCDWrite(LCD_COMMAND, 0x0C); //Set display control, normal mode.
}
