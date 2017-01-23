
//
// ad9516uino - an Arduino sketch for controlling an AD9516 via SPI
//
// The MIT License (MIT)
//
// Copyright (c)  2017  Michael J. Wouters
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <SPI.h>

#define BUFLEN 64
#define ERRSTACKLEN 20 // this is standard for SCPI

// Default configuration is for 10 MHz input, 100 MHz output

const struct {int reg; uint8_t val; } ad9516_regs[] = {
{0x0000, 0x99}, // Serial port configuration: SDO active
{0x0001, 0x00}, // blank
{0x0002, 0x10}, // reserved
{0x0003, 0x01}, // Part ID: AD9516-0 (read only)
{0x0004, 0x00}, // Readback control: DEFAULT
{0x0010, 0x7C}, // PLL: normal operation
{0x0011, 0x05}, // R: lower 8 bits
{0x0012, 0x00}, // R: upper 6 bits
{0x0013, 0x0C}, // A counter
{0x0014, 0x12}, // B counter: lower 8 bits
{0x0015, 0x00}, // B counter: upper 5 bits
{0x0016, 0x05}, // PLL control 1:Prescaler: Divide by 32 (32/33) mode
{0x0017, 0xb4}, // PLL control 2: PLL_STATUS = Lock Detect
{0x0018, 0x07}, // PLL control 3:
{0x0019, 0x00}, // PLL control 4:
{0x001A, 0x00}, // PLL control 5:
{0x001B, 0xE0}, // PLL control 6: VCO+REF2+REF1 frequency monitor enabled
{0x001C, 0x02}, // PLL control 7: REF 1 on
{0x001D, 0x00},
{0x001E, 0x00},
{0x001F, 0x0E}, // PLL

{0x00A0, 0x01}, // OUT6 delay bypass: DEFAULT
{0x00A1, 0x00}, // OUT6 delay full-scale: DEFAULT
{0x00A2, 0x00}, // OUT6 delay fraction: DEFAULT

{0x00A3, 0x01}, // OUT7 delay bypass: DEFAULT
{0x00A4, 0x00}, // OUT7 delay full-scale: DEFAULT
{0x00A5, 0x00}, // OUT7 delay fraction: DEFAULT

{0x00A6, 0x01}, // OUT8 delay bypass: DEFAULT
{0x00A7, 0x00}, // OUT8 delay full-scale: DEFAULT
{0x00A8, 0x00}, // OUT8 delay fraction: DEFAULT

{0x00A9, 0x01}, // OUT9 delay bypass: DEFAULT
{0x00AA, 0x00}, // OUT9 delay full-scale: DEFAULT
{0x00AB, 0x00}, // OUT9 delay fraction: DEFAULT

{0x00F0, 0x08}, // OUT0: DEFAULT (non-inverting,on, 780 mV)
{0x00F1, 0x0A}, // OUT1: DEFAULT (non-inverting,off,780 mV)
{0x00F2, 0x08}, // OUT2: &c
{0x00F3, 0x0A}, // OUT3
{0x00F4, 0x08}, // OUT4
{0x00F5, 0x0A}, // OUT5

{0x0140, 0x42}, // OUT6: DEFAULT (CMOSB off,LVDS, 3.5 mA,on)
{0x0141, 0x43}, // OUT7: DEFAULT (off)
{0x0142, 0x42}, // OUT8: &c
{0x0143, 0x43}, // OUT9: &c

{0x0190, 0x00}, // DIVIDER 0 (PECL)
{0x0191, 0x80},
{0x0192, 0x00},

{0x0193, 0xBB}, // DIVIDER 1 (PECL)
{0x0194, 0x00},
{0x0195, 0x00},

{0x0196, 0x00}, // DIVIDER 2 (PECL)
{0x0197, 0x00},
{0x0198, 0x00},

{0x0199, 0x22}, // DIVIDER 3 (LVDS/CMOS)
{0x019A, 0x00},
{0x019B, 0x11},
{0x019C, 0x00}, // Divider bypass
{0x019D, 0x00}, // DIV3 DCCOFF

{0x019E, 0x22}, // DIVIDER 4 (LVDS/CMOS)
{0x019F, 0x00}, 
{0x01A0, 0x11}, 
{0x01A1, 0x00},
{0x01A2, 0x00}, // DIV4 DCCOF

{0x01A3, 0x00}, // Reserved

{0x01E0, 0x02}, // VCO divider:
{0x01E1, 0x00}, // CLK input:
{0x0230, 0x00}, // System: power down and sync
{0x0231, 0x00}, // System: blank/reserved
{0x0232, 0x00}, // Update all registers
{-1, 0}};

const int statusPin = 5;
const int chipSelectPin = 7;
const int chipResetPin = 6;

const int SPI_SPEED = 1000000;

byte cbuf = 0;
char sbuf[BUFLEN+1];
uint8_t  pos=0;
  
uint8_t ad9516_init();
void    push_error(int16_t);

// Error handling
int16_t errStack[ERRSTACKLEN]; // 
uint8_t nErr;//

uint8_t str2hex(char *s,int len,uint32_t *val){
  // note that len is an int because <0 is allowed (and trapped)
  *val=0;

  if (len-2 > 8 || len < 3 ) // overflow, too short
    return 0;
  
  if (!(s[0]=='#' && s[1]=='H'))
    return 0; 
  
  int i;
  
  for (i=2;i<len;i++){
    if (!isHexadecimalDigit(s[i]))
      return 0;
    uint8_t ch=0;
    if (s[i]<= '9') // already checked that it's a valid hex digit so no need to check lower bound
      ch = s[i] - 48;
    else // must be A-F
      ch = s[i] - 65 + 10;
    *val = (*val << 4) | ch;
  }
  return 1;
}

void setup() {

  nErr=0; 

  pinMode(statusPin, OUTPUT); // nb can't use the built-in led with SPI enabled
  digitalWrite(statusPin, HIGH); 
  
  // Set up CS for the ad9516
  pinMode(chipSelectPin, OUTPUT); 
  digitalWrite(chipSelectPin, HIGH); 

  // Set up RESET for the ad9516
  pinMode(chipResetPin, OUTPUT); 
  digitalWrite(chipResetPin, HIGH);
  
  SPI.begin();
  
  // Configure the AD9516
  uint8_t ok = ad9516_init();

  // If all is well, light the happy light
  digitalWrite(statusPin, (ok?HIGH:LOW)); 

  Serial.begin(9600);
 
}

void loop() {
  
  // Use a SCPI-like syntax for commands
  // *RST soft reset
  // *IDN?
  // :REG #Hxxxx? read a register
  // :REG #Hxxxx,#Hxxxx set a register
  // :REG? dump all registers
  // :SYST:ERR?
  
  uint32_t reg;
  uint32_t val;
  uint8_t ret;
   
  if (Serial.available() > 0){
    cbuf = Serial.read();
    cbuf=toupper(cbuf); // toupper() uses 100 bytes
    if (pos==BUFLEN+1){// input buffer overrun
      clear_sbuf();
      push_error(-363); // std_inputBufferOverrun
      return;
    }
    
    if (cbuf==10){ // terminate on newline
        sbuf[pos]=0; // terminate the string

        // In the following, string length is not tested because the buffer is zeroed after parsing
        // so that we never inadvertently use data from a previous command
        if (sbuf[0]=='*' && sbuf[1]=='I' && sbuf[2]== 'D' && sbuf[3]=='N' && sbuf[4]=='?' && pos==5){
          clear_sbuf();
          Serial.println("AD9516uino,v0.2");
          return;
        } 
        
        if (sbuf[0]=='*' && sbuf[1]=='R' && sbuf[2]== 'S' && sbuf[3]=='T' && pos==4){ // strstr uses 4K ..
          ad9516_init();
          clear_sbuf();
          return;
        }
        
        if (sbuf[0]==':' && sbuf[1]=='S' && sbuf[2]=='Y' && sbuf[3]== 'S' && sbuf[4]=='T' && sbuf[5]==':' &&
            sbuf[6]=='E' && sbuf[7]=='R' && sbuf[8]== 'R' && sbuf[9]=='?' && pos==10){ 
          if (nErr==0){
            Serial.println("+0,\"No error\"");
          }
          else{
            Serial.print(errStack[nErr-1],DEC);
            Serial.println(",\"RTFM\""); // yes, really
            pop_error();
          }
          clear_sbuf();
          return;
        }
        
        if (sbuf[0]==':' && sbuf[1]=='R' && sbuf[2]== 'E' && sbuf[3]=='G'){
          
          if (sbuf[4]=='?'){ // simple query
            clear_sbuf();
            ad9516_dump_regs();
            return;
          }
          
          if (sbuf[pos-1]=='?'){ // register query
          
            if ( str2hex(&(sbuf[4]),pos-5, &reg)){ // pos is length of string
               SPI.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
               digitalWrite(chipSelectPin, LOW); // select device
               ret = ad9516_read_reg((uint16_t) reg);
               digitalWrite(chipSelectPin, HIGH); // deselect device
               SPI.endTransaction();
               Serial.print(reg,HEX);
               Serial.print(",");
               Serial.println(ret,HEX);
            }
            else{
              push_error(-100);
            }
            clear_sbuf();
            return;
            
          } // if (sbuf[pos-1]=='?')

         // Could be a REG write so look for the delimiter
         int  ich=-1;
         for ( int i=4;i<=pos-1;++i){
          if (sbuf[i]==','){
            ich = i;
            break;
          }
         }
         
         if (ich==-1){
           clear_sbuf();
           push_error(-103); // std_invalidSeparator
           return;
         }
         
         // Can parse the two hex arguments now
         uint8_t gotReg = str2hex(&(sbuf[4]),ich-4, &reg);
         uint8_t gotVal = str2hex(&(sbuf[ich+1]),pos-ich-1, &val);
         
         if (gotReg && gotVal){
          
           SPI.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
           digitalWrite(chipSelectPin, LOW); // select device
           ad9516_write_reg(reg,val,1);
           ad9516_write_reg(0x232,0x01,1);
           digitalWrite(chipSelectPin, HIGH); // deselect device
           SPI.endTransaction();
        
         }
         else{
           push_error(-100);
         }
         clear_sbuf();
         return;
      
      } //if (sbuf[0]==':' && sbuf[1]=='R' && sbuf[2]== 'E' && sbuf[3]=='G')
      
      clear_sbuf();
      push_error(-100);
      return;  
    }//if (cbuf==10)
    else{
      if (!isspace(cbuf)){ // strip whitespace - it makes parsing simpler
        sbuf[pos]=cbuf;
        ++pos;
      }
    }//if (cbuf==10)
   
  } //if (Serial.available() > 0
}

void clear_sbuf()
{
  memset((void *) sbuf,0,BUFLEN+1);
  pos=0;
}

// Pushes an error onto the stack
void push_error(int16_t currErr){
  if (nErr<ERRSTACKLEN){
    errStack[nErr]=currErr;
    ++nErr;
  }
  else{
     errStack[nErr-1]=-350; // stack overflow
  }
  //Serial.print("Pushed ");
  //Serial.print(errStack[nErr-1],DEC);
  //Serial.print(" ");
  //Serial.println(nErr,DEC);
}

// Pops an error from the stack
void pop_error(){
  if (nErr > 0)
    --nErr;
}

uint8_t ad9516_init()
{
  // Maximum SCLK frequency is 25 MHz, default bit order is MSBFIRST
  // From the AD9516-0 datasheet 
  // "Write data bits are registered on the rising edge of this
  // clock, and read data bits are registered on the falling edge"
  // 

  uint8_t regVal;
  uint8_t ok=1;

  delay(1000);
   
  SPI.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(chipSelectPin, LOW); // select device

  ad9516_write_reg(0x00, 0x99,1); // long instructions, unidirectional
  ad9516_write_reg(0x232,0x01,1); // copy buffers to registers
  
  // Test for a functioning chip by reading the ID register
  if((regVal=ad9516_read_reg(0x03)) != 0x01){ // NB part-specific
    ok=0;
    goto CLEANUP;
  }
  
  CLEANUP:
  
  digitalWrite(chipSelectPin, HIGH); // deselect device
  SPI.endTransaction();

  delay(1000);
  
  if (ok==0){
    push_error(-240); // std_hardware
  }
 
  return ok;pos=0;
}

uint8_t ad9516_read_reg(uint16_t reg)
{
  // Reads one register
  uint8_t ret = 0;
   // For READ, MSB is 1
  uint16_t instructionWord = 0x8000 | (reg & 0x03ff) ;
  SPI.transfer16(instructionWord);
  ret = SPI.transfer(0x0); // extra bits seem to be ignored ... should only need 15 clocks (and this is what I see in AD's demo S/W)
  return ret;
}

void ad9516_write_reg(uint16_t reg,uint32_t val,uint8_t nbytes)
{
  // Writes of up to 3 bytes are supported
  // In MSB first mode, the address decrements from the initial address
  // Write order is 01, 0102, 010203
  // For WRITE, MSB is zero
  uint16_t instructionWord = 0x0000 | ((nbytes-1) <<13) | (reg & 0x03ff) ;
  SPI.transfer16(instructionWord);
  
  if (nbytes == 1)
    SPI.transfer(val);
  else if (nbytes==2){
    SPI.transfer16(val);
  }
  else if (nbytes==3){
    SPI.transfer16(val >> 8);
    SPI.transfer(val);
  }
}

void ad9516_dump_regs()
{
  int i=0;
  
  SPI.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(chipSelectPin, LOW); // select device
  
  while (ad9516_regs[i].reg != -1){
    Serial.print(ad9516_regs[i].reg,HEX);
    Serial.print(",");
    Serial.print(ad9516_regs[i].val,HEX);
    Serial.print(",");
    Serial.println(ad9516_read_reg(ad9516_regs[i].reg),HEX);
    i++;
  }

  digitalWrite(chipSelectPin, HIGH); // deselect device
  SPI.endTransaction();
  
}

