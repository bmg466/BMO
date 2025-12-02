//-----------------------------------------------------------------------------------------------------
//                                     BMO HUB FIRMWARE NICA MPD 
//                                          CREATED BY BMG
// 17.01.2024 BMG 115200 baud at 16 MHz
//
// 02.02.2024 BMG THR L/R P/N + DAC1 + DAC2 RI PB4
// 06.02.2024 BMG refine Tx / Rx, refine adc_single_measure()
// 02.10.2024 BMG rev 3.1 update, more address up16 +D, ADC remap, PTC refine
// 19.09.2025 BMG added daclsb&=0xfc; in w case
//-----------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------
//Includes
//-----------------------------------------------------------------------------------------------------

#include <iostm8l152c8.h>                    // c8
#include <stm8l.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>
//#include <stm8l15x_flash.h>
//#include <i2c.h>


//#include <i2c_master_poll.h>

//-----------------------------------------------------------------------------------------------------
//Global constant deinitions
//-----------------------------------------------------------------------------------------------------

//#define dselfaddr 11;
#define revision 312190925; // rev PCB, rev FW
#define revisionsector "x" //revsector 

#define RBSIZE	16
#define cmdbufsz 16 
#define sbufsz 80 

#define I2C_ERROR_NO_ACK 0
#define I2C_ERROR_TX_BUF_EMPTY 1
#define I2C_ERROR_ 2

#define I2C_TIMEOUT 1000  // ???????????? ????? ???????? ? ??????


//-----------------------------------------------------------------------------------------------------
//Global variables
//-----------------------------------------------------------------------------------------------------

char cmdl[cmdbufsz];                         // input buffer used to get full command line from PC until '\n'
// sprintf buffer size 
char sbuf[sbufsz];                           // sprintf buffer (to usart1)

float vtpmv, vtnmv, vtamv, tptn;

const float VCC_ref = 2.997;
const float R_ref = 992.0;  // 
const float alpha = 0.003850;  //0.0039083;
float dDac = 0.732421875;

/*const float A = 0.0039083;  
const float B = -0.0000005775;
const float C = -0.000000000004183;
*/

float Vout, Rth, temperature, adcValue, vcc_adc, VCC,vtLp, vtLn, vtRp, vtRn, vdThrR, vdThrL;

unsigned int  vta, PT, GT;
uint32_t rev,timeout;
volatile char address;
volatile char inbuf[RBSIZE];		     // circular reception buffer
volatile char *rdptr;			     // read pointer for inbuf (used in interrup and main)
volatile char *wrptr;			     // write pointer for inbuf (used in interrup and main)
// command line buffer size - used to get full command line from PC/ctl until '\n' 
volatile unsigned char cmd, star, side,dacid;            // cmd - command, one of "r w < >" , side = Right/Left/All
volatile unsigned char sl, s;                // lenghth to print and cycle var for sprintf buffer 
volatile unsigned int idelay;

volatile int vdac1 = 100;
volatile int vdac2 = 100;
volatile int vdac1calib, vdac1last;
volatile int argsread;                       // number of arguments (values) got after sscanf // receiver buffer size


 

volatile uint8_t registerAddress;
char ca, oa;                // vars coming from external command - ca - Cell Address, sa - SubAddress inside the cell
volatile unsigned int cd1, cd2;              // vars coming from external command - cd - data to be written to cell
int sad;               // Our Address, Self Address Delay
volatile unsigned int daclsb,dacmsb,daccmd,dacreg;
volatile unsigned char* revs;
volatile uint8_t msdb, lsdb, msb, lsb, value, dacchannel;
volatile uint8_t result, dataR,temp;


int __eeprom_wait_for_last_operation(void)
{
 if(FLASH_IAPSR_bit.WR_PG_DIS) return 0;
 while(!FLASH_IAPSR_bit.HVOFF);
 return 1;
}

void __eeprom_program_byte(uint8_t __near * dst, uint8_t v)
{
 *dst = v;
}

void __eeprom_program_long(uint8_t __near * dst, uint32_t v)
{
 FLASH_CR2_bit.WPRG = 1;
 *(dst++) = *((uint8_t*)(&v));   
 *(dst++) = *((uint8_t*)(&v) + 1);
 *(dst++) = *((uint8_t*)(&v) + 2);
 *dst = *((uint8_t*)(&v) + 3);  
}

__eeprom uint16_t eeV[16] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 
                             4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095}; // store ch voltage on any change;
typedef struct {
    uint8_t slave_addr;    // address DAC (right 0x48, left 0x49)
    uint8_t register_addr; // channel (1 0x08 - 8 0x0F)
} DAC_Channel;
const DAC_Channel channel_config[16] = {
    {0x48, 0x08}, {0x48, 0x09}, {0x48, 0x0A}, {0x48, 0x0B},
    {0x48, 0x0C}, {0x48, 0x0D}, {0x48, 0x0E}, {0x48, 0x0F},
    {0x49, 0x08}, {0x49, 0x09}, {0x49, 0x0A}, {0x49, 0x0B},
    {0x49, 0x0C}, {0x49, 0x0D}, {0x49, 0x0E}, {0x49, 0x0F}
};
//-----------------------------------------------------------------------------------------------------
//Function prototypes
//-----------------------------------------------------------------------------------------------------
//void WDT_Init(void);
void initextdac(void);
void SetDacAfterRestart(void);
void EEPROM_Write(uint16_t __eeprom *address, uint16_t value);
void setextdac(uint8_t dacchannel, uint8_t msb, uint8_t lsb);
void sendMessage(const char *format, ...);    
char determine_address(void);

void I2C_Init(void);
void delay(unsigned int delay);
void sadelay(void);
void Tx(void);
void Rx(void);
void init_adc(void);
void initextdac(void);
void dac_calibrator(void);


char rch(void);

char SendData(char address);
//char getData(char address);
void I2C_WriteData(uint8_t address, uint8_t *data, uint8_t length);
void resetI2C(void);
uint16_t I2C_ReadRegister(uint8_t deviceAddress, uint8_t registerAddress);

int MyLowLevelPutchar(int sendchar);

unsigned int adc_single_measure(uint8_t channel);
unsigned int adc_multiple_measure(uint8_t channel);

unsigned int palka_set(uint8_t palka);




__interrupt void interrupt(void);



//-----------------------------------------------------------------------------------------------------
// MAIN ROUTINE
//-----------------------------------------------------------------------------------------------------

int counter=0;                             // general purpose cycle/index var

char inch; 



int main() 
{
  // Clock Configuration
  
  CLK_SWCR_SWEN = 1;                  //  Enable switching.
  CLK_SWR = 0x01;                     //  Use HSI as the clock source.
  while (CLK_SWCR_SWBSY != 0);        //  Pause while the clock switch is busy. 
  //Set Clock to full speed
  CLK_CKDIVR_CKM = 0; // Set to 0x00 => /1 ; Reset is 0x03 => /8
  
      CLK_CKDIVR = 0x00;  // System clock source /16 == 16MHz
  //    // Enable clock switch, but ensure proper configuration to select the desired clock source
      CLK_SWCR_bit.SWEN = 1;
      CLK_ICKCR_bit.HSION = 1;  // Enable HSI
      while (CLK_ICKCR_bit.HSIRDY != 1) {}  // Wait for HSI to stabilize
  
  // I2C Initialization
  I2C_Init();

  
  //--------------------  USART1 INIT------   
  
  
  // Remap USART pin
  SYSCFG_RMPCR1_bit.USART1TR_REMAP = 1; // USART1_TX on PA2 and USART1_RX on PA3
  //usart1 io      
  PA_DDR_bit.DDR2 = 1;                 // PA2 OutPut TX line on
  PA_CR1_bit.C12 = 1;                  // Push-pull
  PA_CR2_bit.C22 = 0;                  // Output speed up to 10 MHz
  
  PA_DDR_bit.DDR3 = 0;                 //  PA3 InPut RX line on
  PA_CR1_bit.C13 = 1;                  // Input with pull-up
  PA_CR2_bit.C23 = 0;                  // 2Mhz
  
  PA_DDR_bit.DDR4 = 1;
  PA_CR1_bit.C14 = 1;
  PA_CR2_bit.C24 = 1;
  PA_ODR_bit.ODR4 = 0;                 // 0 - low is enable
  
  
  CLK_PCKENR1_bit.PCKEN15 = 1;              // USART1 clock enable 
  
  // USART Speed 115200 bps for 16MHz clock on chip
  USART1_BRR2 = 0x0A;                       // 0Ah
  USART1_BRR1 = 0x08;                       // 08h 115200 baud
  
  // USART pin
  USART1_CR2_bit.REN = 1;                   // rx en
  USART1_CR2_bit.TEN = 1;                   // tx en
  USART1_CR2_bit.RIEN = 1;                  // interrupt r i e n
  
  asm("RIM");
  //-------------------- END USART1 INIT------  
  CLK_PCKENR2_bit.PCKEN20 = 1;               // ADC clock enable
  init_adc();
  
  CLK_PCKENR1_bit.PCKEN17 = 1; // give clocking in DAC
  CLK_PCKENR2_bit.PCKEN25 = 1; // for acces to RI registers (clocking comparator)
  // RI_ASCR1_bit.AS4 = 0;
  RI_IOSR3_bit.CH15E = 1; // turn on I/O switch for PB4
  
  
  
  DAC_CH1CR1_bit.BOFF = 0;                  // set internal buffer for reduce output impedance DAC1
  DAC_CH2CR1_bit.BOFF = 0;                  // DAC2
  
  DAC_CH1CR1_bit.EN = 1;                    // powered DAC1
  DAC_CH2CR1_bit.EN = 1;                    // DAC2
  
  
  DAC_CH1DHR8 = vdac1;
  DAC_CH2DHR8 = vdac2;
  
  // PE0..PE7, PD0, PD1, PC2..PC7 OUTPUT, OFF for palka_set
  // PE0..PE7 
  PE_DDR_bit.DDR0 = 1;
  PE_CR1_bit.C10 = 1;  
  PE_ODR_bit.ODR0 = 0;  
  
  PE_DDR_bit.DDR1 = 1;
  PE_CR1_bit.C11 = 1;
  PE_ODR_bit.ODR1 = 0;
  
  PE_DDR_bit.DDR2 = 1;
  PE_CR1_bit.C12 = 1;
  PE_ODR_bit.ODR2 = 0;
  
  PE_DDR_bit.DDR3 = 1;
  PE_CR1_bit.C13 = 1;
  PE_ODR_bit.ODR3 = 0;
  
  PE_DDR_bit.DDR4 = 1;
  PE_CR1_bit.C14 = 1;
  PE_ODR_bit.ODR4 = 0;
  
  PE_DDR_bit.DDR5 = 1;
  PE_CR1_bit.C15 = 1;
  PE_ODR_bit.ODR5 = 0;
  
  PE_DDR_bit.DDR6 = 1;
  PE_CR1_bit.C16 = 1;
  PE_ODR_bit.ODR6 = 0;
  
  PE_DDR_bit.DDR7 = 1;
  PE_CR1_bit.C17 = 1;
  PE_ODR_bit.ODR7 = 0;
  
  // PD0, PD1, PD6 (CLR_R), PD7 (CLR_L) 
  PD_DDR_bit.DDR0 = 1;
  PD_CR1_bit.C10 = 1;
  PD_ODR_bit.ODR0 = 0;
  
  PD_DDR_bit.DDR1 = 1;
  PD_CR1_bit.C11 = 1;
  PD_ODR_bit.ODR1 = 0;
  
  PD_DDR_bit.DDR6 = 1;
  PD_CR1_bit.C16 = 1;
  PD_ODR_bit.ODR6 = 1; //set on
  
  PD_DDR_bit.DDR7 = 1;
  PD_CR1_bit.C17 = 1;
  PD_ODR_bit.ODR7 = 1; //set on
  
  
  
  
  //  PC2..PC7 
  PC_DDR_bit.DDR2 = 1;
  PC_CR1_bit.C12 = 1;
  PC_ODR_bit.ODR2 = 0;
  
  PC_DDR_bit.DDR3 = 1;
  PC_CR1_bit.C13 = 1;
  PC_ODR_bit.ODR3 = 0;
  
  PC_DDR_bit.DDR4 = 1;
  PC_CR1_bit.C14 = 1;
  PC_ODR_bit.ODR4 = 0;
  
  PC_DDR_bit.DDR5 = 1;
  PC_CR1_bit.C15 = 1;
  PC_ODR_bit.ODR5 = 0;
  
  PC_DDR_bit.DDR6 = 1;
  PC_CR1_bit.C16 = 1;
  PC_ODR_bit.ODR6 = 0;
  
  PC_DDR_bit.DDR7 = 1;
  PC_CR1_bit.C17 = 1;
  PC_ODR_bit.ODR7 = 0;  
  
  //Port init   
  
  char phaddress = determine_address();
  oa = phaddress; 
  rev = revision;
  revs = revisionsector;
  dDac = VCC_ref / 4095;
  initextdac(); //DAC init
  //  init_adc();
  delay(1000);
  SetDacAfterRestart();
  //  dac_calibrator();
  //  DAC_CH1DHR8 = vdac1calib;
  wrptr = rdptr = inbuf;
  
  
  sadelay(); //delay by adress
  sendMessage(" %c ready \r\n",oa); 
  cmdl[0] = 0; 
  
  
  //sendMessage(" %u ready \r\n",eeV[1]); 
  
  
  
  while(1)			             // loop forever waiting for a command and servicing interrupts
  { 
    msb = lsb = value = dacchannel = 0;
    counter = 0; // point to beginning of cmdl
    argsread = 0;
    //for(int i=0; i < cmdbufsz;i++){cmdl[i]=0;}
    do 
    {inch = rch(); cmdl[counter++]=inch;} 
    while(inch!='\r' && inch!='\n' && counter!=cmdbufsz);        // read cmd line from PC/ctl to buffer while not CR or LF or buffer overflow 
    
    
    
    cmdl[--counter] = 0;                                                     // terminate buffer by 0
    
    if(cmdl[0]!='*' && cmdl[0]!='@' && cmdl[0]!='&' && cmdl[0]!='#' && cmdl[0]!='w' && cmdl[0]!='r') continue;
    
    
if(cmdl[0]=='w') {  // write dac data w 3 S ff ff.
    argsread = sscanf(cmdl, "%c%c%c%2x%2x", &cmd, &ca, &dacid, &dacmsb, &daclsb); daclsb&=0xfc;
    if(counter != 8 || argsread != 5 || ca != oa) {
        continue;
    }

    // Convert char to uint8_t (0-15)
    uint8_t dac_channel;
    if (dacid >= '0' && dacid <= '9') {
        dac_channel = dacid - '0';
    } 
    else if (dacid >= 'A' && dacid <= 'F') {
        dac_channel = dacid - 'A' + 10;
    } 
    else if (dacid == 'S') {
        uint8_t data2[] = {0x03, dacmsb, daclsb}; // Set Data for all DACs
        I2C_WriteData(0x48, data2, sizeof(data2));
        I2C_WriteData(0x49, data2, sizeof(data2));
        
        uint16_t value = ((uint16_t)dacmsb << 8) | daclsb;
    for (uint8_t i = 0; i < 16; i++) {
        EEPROM_Write(&eeV[i], value); //update all eeV
    }

        sendMessage("%c all ch set %02x %02x \r\n", dacid, dacmsb, daclsb); 
        continue;  
    } 
    else {
        sendMessage("Invalid channel\r\n");
        continue;
    }

    // ?????? dac_channel ???????? ?????????? ???????? ?? 0 ?? 15
    setextdac(dac_channel, dacmsb, daclsb);
    sendMessage("Channel %c (0x%X) set to 0x%02X 0x%02X\r\n", dacid, dac_channel, dacmsb, daclsb);
    continue;  // *** ????????? ?????????, ?? ?????? ? switch ***

     
    }//end of 'w' cmd
    
    if(cmdl[0]=='r'){//read data from DAC  r308.
      argsread = sscanf(cmdl, "%c%c%x",&cmd,&ca,&dacreg);
      if(counter!=4 || argsread!=3 || ca!=oa) {continue;}
      
      registerAddress = dacreg;  // Example register address
      //temp = registerAddress;
      uint16_t dataR;
      // Read register from device with A0 on GND
      dataR = I2C_ReadRegister(0x48,registerAddress);
      sendMessage("read %02x ext.DAC 0x48  %04x \r\n",registerAddress,dataR);
      
      dataR = I2C_ReadRegister(0x49,registerAddress);
      sendMessage("read %02x ext.DAC 0x49  %04x \r\n",registerAddress,dataR);
      continue;
    }//end of 'r' cmd
    
    if(cmdl[0]=='@'){//set DAC by addres and side~  @3R044.
      argsread = sscanf(cmdl, "%c%c%c%3d",&cmd,&ca,&side,&cd1);  // get command, addr., side and 8bit data from cmdline
      if(counter!=7 || argsread!=4 || ca!=oa) continue;}
    
    if(cmdl[0]=='&'){//set palka all~ &38. mean brd 3 case 8 set all palka off
      argsread = sscanf(cmdl, "%c%c%d",&star,&ca,&cd1);                  // get command, addr. and 0-9 case
      if(counter!=4 || argsread!=3 || ca!=oa) continue;}
    
    if(cmdl[0]=='*'){//to all~ *R.
      argsread = sscanf(cmdl, "%c%c", &star, &cmd);
      if(counter!=3 || argsread!=2) {continue;} sadelay();}
    
    if(cmdl[0]=='#'){//send CMD by ADDR~  #3R.
      argsread = sscanf(cmdl,"%c%c%c",&star,&ca,&cmd);                  // get command, addr. and A-Z case
      if(counter!=4 || argsread!=3 || ca!=oa) continue;}
    
    
    //////////////////////////////////////////////////   //////////////////////////////////////////////////              
    
    switch (cmd) 
    {
      
    case '@': //set DAC by addres and side~  @3R044.
      
      if(side == 'L' ){
        vdac1 = cd1;
        DAC_CH1DHR8 = vdac1;
        sendMessage("%c %c %d\r\n", ca, side, vdac1); 
        
      }
      else if(side == 'R'){
        vdac2 = cd1;
        DAC_CH2DHR8 = vdac2;
        sendMessage("%c %c %d\r\n", ca, side, vdac2);
        
      }
      else if(side == 'A'){
        vdac1 = vdac2 = cd1;
        DAC_CH1DHR8 = vdac1;
        DAC_CH2DHR8 = vdac2;
        sendMessage("%c %c %d\r\n", ca, side, vdac1); 
        
      }
      else {
        sendMessage("index error\r\n"); 
        
      }
      
      break;
      
    case '&': //set palka all~ &38. mean brd 3 case 8 set all palka off
      
      palka_set(cd1);
      
      break;
      
    case 'f': 
      
      uint8_t data[] = {0x01, 0x00, 0x00}; //config DAC
      I2C_WriteData(0x48, data, sizeof(data)); //DAC RIGHT
      sendMessage("ext.DAC 0x48 set ON \r\n");
      I2C_WriteData(0x49, data, sizeof(data)); // DAC LEFT
      sendMessage("ext.DAC 0x49 set ON \r\n");
      delay(200);
       
      
      //0x03 brdcast command byte , ff00 2.8V   ffff  2.994    00ff 0.186  0100-0.002v  01ff 0.373V
      //uint8_t data2[] = {0x03, 0x55, 0x80}; //set Data
     // I2C_WriteData(0x48, data2, sizeof(data2));
     //sendMessage("ext.DAC 0x48 set 0x%02x 0x%2x 0x%2x \r\n",data2[0],data2[1],data2[2]);
     // I2C_WriteData(0x49, data2, sizeof(data2));
     // sendMessage("ext.DAC 0x49 set 0x%02x 0x%2x 0x%2x \r\n",data2[0],data2[1],data2[2]);
            
      break;
      
    case 'D': 
      sendMessage("ext.DAC id:%c channel values:\r\n", oa);
      for (uint8_t i = 0; i < 16; i++) {
        sendMessage("Ch %X: 0x%04X\r\n", i, eeV[i]);
      }
      break;
      
    case 'R': 
      
      sendMessage("HUB ID %c rev: %lu \r\n",oa,rev);
      
      break;
      
    case 'i': //read PTC
      
      for(idelay=0;idelay<300;idelay++)
      {
        delay(20000);                       
        vta = adc_multiple_measure(0);
        delay(20000);
        adcValue = vta;
        vcc_adc = adc_multiple_measure(1);
        //VCC = (vcc_adc * VCC_ref) / 4095;
        VCC = vcc_adc * dDac;
        delay(20000);
        delay(20000);
        //Vout = (adcValue * VCC) / 4095;
        Vout = adcValue * dDac;
        delay(20000);
        
        Rth = R_ref*(Vout / (VCC - Vout));
        delay(20000);
        //temperature = 1.0 / (A + B * log(Rth) + C * pow(log(Rth), 3));
        temperature = (log(Rth / 1078.0) / alpha) + 20.0;
        
        
        delay(20000);
        sendMessage(" %c\r\n", oa);
        sendMessage("VCC %f Vout %f Rth %f  T %f\r\n",VCC, Vout, Rth, temperature);
        
        delay(20000);
        delay(20000);
        delay(20000);
        delay(20000);
        delay(20000);
        delay(20000);
        delay(20000);
      }     
      break;          
      
    case '=': 
      
      vdac1 = vdac1calib;
      DAC_CH1DHR8 = vdac1;           
      sendMessage("%d", vdac1);
      
      
      break;
    case 'g': 
      
      vtLp = adc_single_measure(9);
      vtLp = vtLp*dDac;
      delay(20000);
      vtLn = adc_single_measure(10);
      vtLn = vtLn*dDac;
      delay(20000);
      vtRp = adc_single_measure(16);
      vtRp = vtRp*dDac;
      vdThrL = vtLp - vtLn;
      delay(20000);
      vtRn = adc_single_measure(15);
      vtRn = vtRn*dDac;
      delay(20000);
      vdThrR = vtRp - vtRn;
      
      sendMessage(" %c\r\n", oa);
      sendMessage("LP = %f LN = %f vdThrL = %f DAC1 = %d \r\n RP = %f RN = %f vdThrR = %f DAC2 = %d \r\n", vtLp, vtLn, vdThrL, vdac1, vtRp, vtRn, vdThrR, vdac2);
      
      break;           
    case 'G': 
      
      vtLp = adc_multiple_measure(9);
      vtLp = vtLp*dDac;
      delay(10000);
      vtLn = adc_multiple_measure(10);
      vtLn = vtLn*dDac;
      delay(10000);
      vdThrL = vtLp - vtLn; 
      delay(10000);
      
      vtRp = adc_multiple_measure(16);
      vtRp = vtRp*dDac;
      //vtRp = 1599;
      delay(10000);
      vtRn = adc_multiple_measure(15);
      vtRn = vtRn*dDac;
      delay(10000);
      vdThrR = vtRp - vtRn;
      
      sendMessage(" %c\r\n", oa);
      sendMessage("LP = %f LN = %f vdThrL = %f DAC1 = %d \r\n RP = %f RN = %f vdThrR = %f DAC2 = %d \r\n", vtLp, vtLn, vdThrL, vdac1, vtRp, vtRn, vdThrR, vdac2);
      
      break;         
      
    case '?': 
      
      sendMessage("HUB address is %c\r\n", oa);
      
      break;
      
    default:
      
      sendMessage("Unknown case: %c\r\n", cmd);
      
      break;
      
    } //end of switch
    
    
    
  } //end of while
  
} //end of MAIN

//-----------------------------------------------------------------------------------------------------
// END OF MAIN
//-----------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------------------------------

void initextdac(void) {
  uint8_t data[] = {0x01, 0x00, 0x00};              // 0x01 - enable DAC output
  I2C_WriteData(0x48, data, sizeof(data));          // U7 - see .sch
  I2C_WriteData(0x49, data, sizeof(data));          // U6
}

void EEPROM_Write(uint16_t __eeprom *address, uint16_t value) {
    // ?????????????? EEPROM ??? ??????
     FLASH_DUKR = 0xAE;
     FLASH_DUKR = 0x56;
    
    *address = value; // ?????? 16-??????? ???????? ? EEPROM
    while (!(FLASH_IAPSR & MASK_FLASH_IAPSR_EOP));

    // ????????????? EEPROM ????? ??????
    FLASH_IAPSR_bit.DUL=0;
}
void setextdac(uint8_t dacchannel, uint8_t msb, uint8_t lsb) { 
  
    //sendMessage("dacchannel = %u \r\n", dacchannel);
    if (dacchannel >= 16) return; // Check if the channel number is valid (should be 0-15)

    // Get the DAC address and register from the table
    uint8_t slave_addr = channel_config[dacchannel].slave_addr;
    uint8_t register_addr = channel_config[dacchannel].register_addr;

    // Prepare the data array for I2C transmission
    uint8_t initdata[3] = { register_addr, msb, lsb };

    //sendMessage("slave_addr = %02x \r\n", slave_addr);
    //sendMessage("register_addr = %02x \r\n", register_addr);
    //sendMessage("msb = %02x \r\n", msb);
    //sendMessage("lsb = %02x \r\n", lsb);

    // Send data via I2C to the corresponding DAC
    I2C_WriteData(slave_addr, initdata, sizeof(initdata));

    // Combine MSB and LSB into a 16-bit value and store it in eeV[]
    uint16_t value = ((uint16_t)msb << 8) | lsb;
    EEPROM_Write(&eeV[dacchannel], value);

    //sendMessage("DAC[%u] = 0x%04X ???????? ? EEPROM\r\n", dacchannel, value);
}

void SetDacAfterRestart() {
    uint8_t msb, lsb;

    for (uint8_t dac_channel = 0; dac_channel < 16; dac_channel++) {
        if (eeV[dac_channel] != 0xFFFF) {  // ?????????, ??? ???????? ?? "??????"
            msb = (uint8_t)(eeV[dac_channel] >> 8);  // ??????? ????
            lsb = (uint8_t)(eeV[dac_channel] & 0xFF); // ??????? ????
            
            // ??????????????? ???????? ????? ???? ???????
            setextdac(dac_channel, msb, lsb);

            //sendMessage("Restart DAC ch=%u set %04x\r\n", dac_channel, eeV[dac_channel]);
        }
    }
}

void sendMessage(const char *format, ...) {
  Tx();
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
  Rx();
}

char determine_address() {
  // set port as IN
  PD_DDR_DDR2 = 0; // A
  PD_DDR_DDR3 = 0; // B
  PB_DDR_DDR0 = 0; // C
  PB_DDR_DDR1 = 0; // D
  
  // read port state
  char phaddress;
  if (PD_IDR_IDR2 == 0 && PD_IDR_IDR3 == 0 && PB_IDR_IDR0 == 0 && PB_IDR_IDR1 == 0) {
    phaddress = '0'; // 0
  } else if (PD_IDR_IDR2 == 0 && PD_IDR_IDR3 == 0 && PB_IDR_IDR0 == 0 && PB_IDR_IDR1 == 1) {
    phaddress = '1'; // 1
  } else if (PD_IDR_IDR2 == 0 && PD_IDR_IDR3 == 0 && PB_IDR_IDR0 == 1 && PB_IDR_IDR1 == 0) {
    phaddress = '2'; // 2
  } else if (PD_IDR_IDR2 == 0 && PD_IDR_IDR3 == 0 && PB_IDR_IDR0 == 1 && PB_IDR_IDR1 == 1) {
    phaddress = '3'; // 3
  } else if (PD_IDR_IDR2 == 0 && PD_IDR_IDR3 == 1 && PB_IDR_IDR0 == 0 && PB_IDR_IDR1 == 0) {
    phaddress = '4'; // 4
  } else if (PD_IDR_IDR2 == 0 && PD_IDR_IDR3 == 1 && PB_IDR_IDR0 == 0 && PB_IDR_IDR1 == 1) {
    phaddress = '5'; // 5
  } else if (PD_IDR_IDR2 == 0 && PD_IDR_IDR3 == 1 && PB_IDR_IDR0 == 1 && PB_IDR_IDR1 == 0) {
    phaddress = '6'; // 6
  } else if (PD_IDR_IDR2 == 0 && PD_IDR_IDR3 == 1 && PB_IDR_IDR0 == 1 && PB_IDR_IDR1 == 1) {
    phaddress = '7'; // 7
  } else if (PD_IDR_IDR2 == 1 && PD_IDR_IDR3 == 0 && PB_IDR_IDR0 == 0 && PB_IDR_IDR1 == 0) {
    phaddress = '8'; // 8
  } else if (PD_IDR_IDR2 == 1 && PD_IDR_IDR3 == 0 && PB_IDR_IDR0 == 0 && PB_IDR_IDR1 == 1) {
    phaddress = '9'; // 9
  } else if (PD_IDR_IDR2 == 1 && PD_IDR_IDR3 == 0 && PB_IDR_IDR0 == 1 && PB_IDR_IDR1 == 0) {
    phaddress = 'A'; // A
  } else if (PD_IDR_IDR2 == 1 && PD_IDR_IDR3 == 0 && PB_IDR_IDR0 == 1 && PB_IDR_IDR1 == 1) {
    phaddress = 'B'; // B
  } else if (PD_IDR_IDR2 == 1 && PD_IDR_IDR3 == 1 && PB_IDR_IDR0 == 0 && PB_IDR_IDR1 == 0) {
    phaddress = 'C'; // C
  } else if (PD_IDR_IDR2 == 1 && PD_IDR_IDR3 == 1 && PB_IDR_IDR0 == 0 && PB_IDR_IDR1 == 1) {
    phaddress = 'D'; // D
  } else if (PD_IDR_IDR2 == 1 && PD_IDR_IDR3 == 1 && PB_IDR_IDR0 == 1 && PB_IDR_IDR1 == 0) {
    phaddress = 'E'; // E
  } else if (PD_IDR_IDR2 == 1 && PD_IDR_IDR3 == 1 && PB_IDR_IDR0 == 1 && PB_IDR_IDR1 == 1) {
    phaddress = 'F'; // F
  } else {
    phaddress = 'Z'; // address unknown
  }
  
  return phaddress;
}

void I2C_Init()
{
  //Enable I2C1 Peripheral Clock
  CLK_PCKENR1_PCKEN13 = 1;
  
  //I2C_Init_Pins();//B5 SDA, B4 SCL
  
  I2C1_CR1 = 0;				// clearing (PE) if this is a re - init, this does not stop the ongoing communication
  // CR1_NOSTRETCH	Clock Strtching enabled
  // CR1_ENGC 		General call disabled
  // CR2_POS 			ACK controls the current byte
  I2C1_FREQR = 16;             // clk at least 1 MHz for Standard and 4MHz for Fast
  I2C1_CCRH_F_S = 0;           // I2C running is standard mode.
  I2C1_CCRL = 0x50;            // I2C period = 2 * CCR * tMASTER 100KHz : tabe 50 RM0016 P 315
  I2C1_CCRH = 0x00;			// CCR[11:8] = 0
  
  I2C1_OARH_ADDMODE = 0;               // 7-bit slave address
  I2C1_OARH_ADDCONF = 1;               // This bit must be set by software
  // ADD[9:8] unused
  
  I2C1_TRISER = 9;			//Maximum time used by the feedback loop to keep SCL Freq stable whatever SCL rising time is
  //Standard mode max rise time is 1000ns
  //example for 8MHz : (1000ns / 125 ns = 8 ) + 1 = 9
  //for 16 MHz : (1000 ns / 62.5 ns = 16 ) + 1 = 17
  
  I2C1_CR1_bit.PE = 1;						//Enable the I2C Peripheral
  I2C1_CR2_bit.ACK = 1;
  I2C1_CR2_bit.POS = 0;
  
  // GPIO Configuration for I2C Pins (PC0: SDA, PC1: SCL)
  PC_DDR_bit.DDR0 = 1;  // PC0 (SDA) as input
  PC_DDR_bit.DDR1 = 1;  // PC1 (SCL) as input
  PC_CR1_bit.C10 = 1;   // PC0 (SDA) as open-drain output
  PC_CR1_bit.C11 = 1;   // PC1 (SCL) as open-drain output
  PC_CR2_bit.C20 = 0;   // PC0 (SDA) as external interrupt enabled
  PC_CR2_bit.C21 = 0;   // PC1 (SCL) as external interrupt enabled
}  

void delay(unsigned int delay)
{
  
  unsigned int idelay;
  
  for(idelay=0;idelay<delay;idelay++)
    
    asm("nop"); 
  
}

//-----------------------------------------------------------------------------------------------------

void sadelay()                               // Self Address Delay
{   
  
  sad = strtol(&oa,NULL,16); //convert char oa into int
  while (sad-- > 0)
  {
    delay(20000);delay(20000);delay(20000);delay(20000);delay(20000);delay(20000);delay(20000);delay(20000);delay(20000);delay(20000);
  }                                        // delay
}
//-----------------------------------------------------------------------------------------------------
// TxRx function for USART
//-----------------------------------------------------------------------------------------------------
void Tx()
{
  PA_ODR_bit.ODR4 = 1; //for V3+ only this line 
  delay(20000);
}
void Rx()
{
  delay(20000);
  PA_ODR_bit.ODR4 = 0; //for V3+ only this line 
}
//-----------------------------------------------------------------------------------------------------

void init_adc()
{
  //PD5 THR_LN
  PD_DDR_bit.DDR5 = 0;  //input  
  PD_CR1_bit.C15 = 0;   //ppl  
  PD_CR2_bit.C25 = 0;   //off 
  //PD4 THR_LP
  PD_DDR_bit.DDR4 = 0;  //input  
  PD_CR1_bit.C14 = 0;   //ppl  
  PD_CR2_bit.C24 = 0;   //off
  
  //PB3 THR_RN
  PB_DDR_bit.DDR3 = 0;  //input  
  PB_CR1_bit.C13 = 0;   //ppl  
  PB_CR2_bit.C23 = 0;   //off
  //PB2 THR_RP
  PB_DDR_bit.DDR6 = 0;  //input  
  PB_CR1_bit.C16 = 0;   //ppl  
  PB_CR2_bit.C26 = 0;   //off
  
  //PA6 Tsensor
  PA_DDR_bit.DDR6 = 0;  //input  
  PA_CR1_bit.C16 = 0;   //ppl  
  PA_CR2_bit.C26 = 0;   //off  
  //PA5 Tsensor REFVCC
  PA_DDR_bit.DDR5 = 0;  //input  
  PA_CR1_bit.C15 = 0;   //ppl  
  PA_CR2_bit.C25 = 0;   //off  
  
  
  ADC1_CR1_bit.ADON = 0;                     // adc off
  ADC1_CR1_bit.RES = 0x00;                   // 12 bit
  ADC1_CR2 = 0x87;                           // 384 ADC clock cycles
  ADC1_SQR1_bit.DMAOFF;                      // DMA OFF
  ADC1_TRIGR2_bit.TRIG23 = 1;                // trigger off
  ADC1_TRIGR1_bit.VREFINTON = 1;              // Enable internal reference voltage
  ADC1_TRIGR3_bit.TRIG9 = 1;                 // trigger off
  ADC1_CR1_bit.ADON = 1;                     // adc on
  
}

//-----------------------------------------------------------------------------------------------------

/*void dac_colibrator()
{
while (tptn<=162 || tptn>=170){
vtp = adc_single_measureP(); 
delay(3000);
vtn = adc_single_measureN();
delay(3000);
tptn=(vtp-vtn)*0.805664;
delay(50);
if (tptn<=162){
vdac=vdac-1;DAC_CH1DHR8 = vdac;
vdaccolib=vdac;}
if (tptn>=170){
vdac=vdac+1;DAC_CH1DHR8 = vdac;
vdaccolib=vdac;}}
}
*/
//-----------------------------------------------------------------------------------------------------

char rch(void)                               // Get a char 
{ 
  char c;				     // character to be returned 
  
  while (rdptr == wrptr)               
  {
  };  
  
  c = *rdptr++;
  
  if (rdptr >= &inbuf[RBSIZE]) rdptr = inbuf;
  
  return (c);
}

//-----------------------------------------------------------------------------------------------------

void I2C_WriteData(uint8_t address, uint8_t *data, uint8_t length) {
  // Wait until the bus is free
  timeout = I2C_TIMEOUT;
  while (I2C1_SR3_BUSY && timeout--) {
    if (!timeout) {
      resetI2C();
      return; // Error: bus is busy
    }
  }
  // Generate START condition
  I2C1_CR2_bit.START = 1;
  timeout = I2C_TIMEOUT;
  while (!(I2C1_SR1_bit.SB) && timeout--) {
    if (!timeout) {
      resetI2C();
      return; // Error: START condition not sent
    }
  }
  
  // Send the address with the write bit (0)
  I2C1_DR = address << 1; // R/W = 0
  
  //sendMessage("W address is %02x \r\n", address);
  
  timeout = I2C_TIMEOUT;
  while (!(I2C1_SR1_bit.ADDR) && timeout--) {
    if (!timeout) {
      resetI2C();
      return; // Error: address not acknowledged
    }
  }
  
  // Clear ADDR flag by reading SR1 and SR3
  (void)I2C1_SR1;
  (void)I2C1_SR3;
  
  // Send the data bytes
  for (uint8_t i = 0; i < length; i++) {
    I2C1_DR = data[i];
    timeout = I2C_TIMEOUT;
    while (!(I2C1_SR1_bit.TXE) && timeout--) {
      if (!timeout) {
        resetI2C();
        return; // Error: TXE (Transmit buffer empty) not set
      }
    }
  }
  
  // Wait for the byte transfer to finish
  timeout = I2C_TIMEOUT;
  while (!(I2C1_SR1_bit.BTF) && timeout--) {
    if (!timeout) {
      resetI2C();
      return; // Error: BTF (Byte transfer finished) not set
    }
  }
  
  // Generate STOP condition
  I2C1_CR2_bit.STOP = 1;
}

void resetI2C() {
  I2C1_CR2_bit.STOP = 1;            // ???????????? STOP
  I2C1_CR1_bit.PE = 0;             // ????????? ????????? I2C
  for (volatile int i = 0; i < 1000; i++); // ???????? ????????
  I2C1_CR1_bit.PE = 1;             // ???????? ????????? I2C
}


uint16_t I2C_ReadRegister(uint8_t deviceAddress, uint8_t registerAddress) {
  // read I2C 7-bit
  
  /*
  dac53608
  Address:
  0100 1000  GND 0x48
  0100 1001  VDD 0x49
  0100 1010  SDA 0x4A
  0100 1011  SCL 0x4B
  */
  
  // Wait until the bus is free
  timeout = I2C_TIMEOUT;
  while (I2C1_SR3_BUSY && timeout--) {
    if (!timeout) {
      resetI2C();
      return 0x1FFF; // Error: bus is busy
    }
  }
  
  // Enable acknowledgment (ACK)
  I2C1_CR2_bit.ACK = 1;
  
  // Generate START condition
  I2C1_CR2_bit.START = 1;
  timeout = I2C_TIMEOUT;
  while (!(I2C1_SR1_bit.SB) && timeout--) {
    if (!timeout) {
      resetI2C();
      return 0x2FFF; // Error: failed to generate START
    }
  }
  
  // Send the device address with a write bit (R/W = 0)
  I2C1_DR = (deviceAddress << 1 | 0); // R/W = 0
  timeout = I2C_TIMEOUT;
  while (!(I2C1_SR1_bit.ADDR) && timeout--) {
    if (!timeout) {
      resetI2C();
      return 0x3FFF; // Error: no response from the device
    }
  }
  
  // Clear the ADDR flag
  (void)I2C1_SR1; 
  (void)I2C1_SR3;
  
  // Send the register address
  I2C1_DR = registerAddress;
  timeout = I2C_TIMEOUT;
  while (!(I2C1_SR1_bit.TXE) && timeout--) {
    if (!timeout) {
      resetI2C();
      return 0x4FFF; // Error: data not transmitted
    }
  }
  
  // Generate repeated START condition
  I2C1_CR2_bit.START = 1;
  timeout = I2C_TIMEOUT;
  while (!(I2C1_SR1_bit.SB) && timeout--) {
    if (!timeout) {
      resetI2C();
      return 0x5FFF; // Error: repeated START not generated
    }
  }
  
  // Send the device address with a read bit (R/W = 1)
  I2C1_DR = (deviceAddress << 1) | 1; // R/W = 1
  timeout = I2C_TIMEOUT;
  while (!(I2C1_SR1_bit.ADDR) && timeout--) {
    if (!timeout) {
      resetI2C();
      return 0x6FFF; // Error: no response from the device
    }
  }
  
  // Clear the ADDR flag
  (void)I2C1_SR1; 
  (void)I2C1_SR3;
  
  // Read the MSDB (Most Significant Data Byte)
  timeout = I2C_TIMEOUT;
  while (!(I2C1_SR1_bit.RXNE) && timeout--) {
    if (!timeout) {
      resetI2C();
      return 0x7FFF; // Error: MSDB not received
    }
  }
  msdb = I2C1_DR;
  
  // Disable acknowledgment (ACK) before reading LSDB
  I2C1_CR2_bit.ACK = 0;
  
  // Read the LSDB (Least Significant Data Byte)
  timeout = I2C_TIMEOUT;
  while (!(I2C1_SR1_bit.RXNE) && timeout--) {
    if (!timeout) {
      resetI2C();
      return 0x8FFF; // Error: LSDB not received
    }
  }
  lsdb = I2C1_DR;
  
  // Generate STOP condition
  I2C1_CR2_bit.STOP = 1;
  
  // Combine MSDB and LSDB into a 16-bit result
  result = lsdb | msdb;
  
  return result;
  
  
}

//-----------------------------------------------------------------------------------------------------

int MyLowLevelPutchar(int sendchar)
{
  USART1_DR = sendchar;                      // Put data in send register
  while(!USART1_SR_TC && !USART1_SR_TXE);  // Wait until sent (TXE or TC ??)
  
  return sendchar;
  
}

//-----------------------------------------------------------------------------------------------------
/*
thresholds:
PD4 - THR_LN ADC1_SQR3_bit.CHSEL_S10
PD5 - THR_LP ADC1_SQR3_bit.CHSEL_S9 
PF0 DAC - THR_L

PB3 - THR_RN ADC1_SQR3_bit.CHSEL_S15
PB2 - THR_RP ADC1_SQR2_bit.CHSEL_S16
PB4 DAC - THR_R 

ADC read PTC  
PA6 ADC1_SQR4_bit.CHSEL_S0
PA5 REFTEMPVCC ADC1_SQR4_bit.CHSEL_S1
*/

unsigned int adc_single_measure(uint8_t channel)
{ 
  // 
  switch (channel) {
  case 9:
    ADC1_SQR3_bit.CHSEL_S9 = 1; 
    break;
  case 10:
    ADC1_SQR3_bit.CHSEL_S10 = 1; 
    break;
  case 15:
    ADC1_SQR3_bit.CHSEL_S15 = 1; 
    break;
  case 16:
    ADC1_SQR2_bit.CHSEL_S16 = 1; 
    break;
  case 0:
    ADC1_SQR4_bit.CHSEL_S0 = 1; 
    break;
  case 1:
    ADC1_SQR4_bit.CHSEL_S1 = 1; 
    break;      
  default:
    
    return 0; 
  }
  
  unsigned int adc_res;
  unsigned int value = 0;
  
  ADC1_CR1_bit.START = 1;                    
  
  while (!(ADC1_SR & MASK_ADC1_SR_EOC))   
  {
    //
  };                                      
  
  adc_res = ADC1_DRH << 8;   
  adc_res |= ADC1_DRL;       
  value = adc_res;           
  
  ADC1_SQR3_bit.CHSEL_S9 = 0; 
  ADC1_SQR3_bit.CHSEL_S10 = 0; 
  ADC1_SQR3_bit.CHSEL_S15 = 0; 
  ADC1_SQR2_bit.CHSEL_S16 = 0; 
  ADC1_SQR4_bit.CHSEL_S0 = 0; 
  ADC1_SQR4_bit.CHSEL_S1 = 0;
  
  return value;
}

unsigned int adc_multiple_measure(uint8_t channel) {
  unsigned int sum = 0; // ?????????? ??? ???????? ????? ????????
  unsigned int average = 0; // ?????????? ??? ???????? ???????? ????????
  
  for (int i = 0; i < 16; i++) { // ???? ??? ?????????? 16 ?????????
    switch (channel) {
    case 9:
      ADC1_SQR3_bit.CHSEL_S9 = 1; 
      break;
    case 10:
      ADC1_SQR3_bit.CHSEL_S10 = 1; 
      break;
    case 15:
      ADC1_SQR3_bit.CHSEL_S15 = 1; 
      break;
    case 16:
      ADC1_SQR2_bit.CHSEL_S16 = 1; 
      break;
    case 0:
      ADC1_SQR4_bit.CHSEL_S0 = 1; 
      break;
    case 1:
      ADC1_SQR4_bit.CHSEL_S1 = 1; 
      break; 
    default:
      return 0; // ?????????? 0 ? ?????? ????????? ?????? ??????
    }
    
    ADC1_CR1_bit.START = 1; // ????????? ??????????????
    
    while (!(ADC1_SR & MASK_ADC1_SR_EOC)) {
      // ??????? ????????? ??????????????
    }
    
    unsigned int adc_res = (ADC1_DRH << 8) | ADC1_DRL; // ???????? ????????? ?????????
    sum += adc_res; // ????????? ????????? ????????? ? ?????
  }
  
  average = sum / 16; // ????????? ??????? ????????
  
  // ?????????? ???? ?????? ??????
  ADC1_SQR3_bit.CHSEL_S9 = 0; 
  ADC1_SQR3_bit.CHSEL_S10 = 0; 
  ADC1_SQR3_bit.CHSEL_S15 = 0; 
  ADC1_SQR2_bit.CHSEL_S16 = 0; 
  ADC1_SQR4_bit.CHSEL_S0 = 0; 
  ADC1_SQR4_bit.CHSEL_S1 = 0;
  
  return average; // ?????????? ??????? ????????
}


unsigned int palka_set(uint8_t palka)
{ 
  // 
  switch (palka) {
  case 0:
    PE_ODR_bit.ODR0 = 0; 
    PE_ODR_bit.ODR7 = 0;
    break;
  case 1:
    PE_ODR_bit.ODR1 = 0; 
    PE_ODR_bit.ODR6 = 0; 
    break;
  case 2:
    PE_ODR_bit.ODR2 = 0; 
    PC_ODR_bit.ODR7 = 0; 
    break;
  case 3:
    PE_ODR_bit.ODR3 = 0; 
    PC_ODR_bit.ODR6 = 0;
    break;
  case 4:
    PE_ODR_bit.ODR4 = 0; 
    PC_ODR_bit.ODR5 = 0; 
    break;
  case 5:
    PE_ODR_bit.ODR5 = 0; 
    PC_ODR_bit.ODR4 = 0; 
    break;
  case 6:
    PD_ODR_bit.ODR0 = 0; 
    PC_ODR_bit.ODR3 = 0; 
    break;
  case 7:
    PD_ODR_bit.ODR1 = 0; 
    PC_ODR_bit.ODR2 = 0;  
    break;
  case 8: //set all OFF
    
    PE_ODR_bit.ODR0 = 1; 
    PE_ODR_bit.ODR7 = 1;
    PE_ODR_bit.ODR1 = 1; 
    PE_ODR_bit.ODR6 = 1; 
    PE_ODR_bit.ODR2 = 1; 
    PC_ODR_bit.ODR7 = 1; 
    PE_ODR_bit.ODR3 = 1; 
    PC_ODR_bit.ODR6 = 1;
    PE_ODR_bit.ODR4 = 1; 
    PC_ODR_bit.ODR5 = 1; 
    PE_ODR_bit.ODR5 = 1; 
    PC_ODR_bit.ODR4 = 1; 
    PD_ODR_bit.ODR0 = 1; 
    PC_ODR_bit.ODR3 = 1; 
    PD_ODR_bit.ODR1 = 1; 
    PC_ODR_bit.ODR2 = 1; 
    
    sendMessage("%c case set %d ALL OFF\r\n", ca, cd1);
    
    break;      
  case 9: //set all ON
    
    PE_ODR_bit.ODR0 = 0; 
    PE_ODR_bit.ODR7 = 0;
    PE_ODR_bit.ODR1 = 0; 
    PE_ODR_bit.ODR6 = 0; 
    PE_ODR_bit.ODR2 = 0; 
    PC_ODR_bit.ODR7 = 0; 
    PE_ODR_bit.ODR3 = 0; 
    PC_ODR_bit.ODR6 = 0;
    PE_ODR_bit.ODR4 = 0; 
    PC_ODR_bit.ODR5 = 0; 
    PE_ODR_bit.ODR5 = 0; 
    PC_ODR_bit.ODR4 = 0; 
    PD_ODR_bit.ODR0 = 0; 
    PC_ODR_bit.ODR3 = 0; 
    PD_ODR_bit.ODR1 = 0; 
    PC_ODR_bit.ODR2 = 0; 
    
    sendMessage("%c case set %d ALL ON\r\n", ca, cd1);
    
    break; 
  default:
    return 0; 
  }
  
  return 0;
}

//-----------------------------------------------------------------------------------------------------

// interrupt handler on RX USART1

#pragma vector = USART1_R_OR_vector          // (OR or RXNE???)

__interrupt void interrupt(void)
{
  
  *wrptr++ = USART1_DR;
  
  if (wrptr >= &inbuf[RBSIZE]) wrptr = inbuf;
  
}