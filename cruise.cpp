#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"

#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001

/* LED Patterns */

#define LED_RED_0 0x01 // Engine
#define LED_RED_1 0x02 // Top Gear

#define LED_GREEN_0 0x01 // Cruise Control activated
#define LED_GREEN_2 0x04 // Cruise Control Button
#define LED_GREEN_4 0x10 // Brake Pedal
#define LED_GREEN_6 0x40 // Gas Pedal

#define LED_RED_17 0x20000 // 0010 0000 0000 0000 0000
#define LED_RED_16 0x10000 // 0001 0000 0000 0000 0000
#define LED_RED_15 0x08000 // 0000 1000 0000 0000 0000
#define LED_RED_14 0x04000 // 0000 0100 0000 0000 0000
#define LED_RED_13 0x02000 // 0000 0010 0000 0000 0000
#define LED_RED_12 0x01000 // 0000 0001 0000 0000 0000

#define LED_RED_9 0x200 // 0000 0000 0010 0000 0000
#define LED_RED_8 0x100 // 0000 0000 0001 0000 0000
#define LED_RED_7 0x80 // 0000 0000 0000 1000 0000
#define LED_RED_6 0x40 // 0000 0000 0000 0100 0000
#define LED_RED_5 0x20 // 0000 0000 0000 0010 0000
#define LED_RED_4 0x10 // 0000 0000 0000 0001 0000


#define ON  1
#define OFF 0

#define ENGINE      0         
#define GEAR        1
#define CRUISE      2  
#define CR_BUTTON   3
#define BRAKE       4
#define GAS         5 

#define DIST_BTW_0000_TO_0400         6 
#define DIST_BTW_0400_TO_0800         7 
#define DIST_BTW_0800_TO_1200         8 
#define DIST_BTW_1200_TO_1600         9 
#define DIST_BTW_1600_TO_2000         10 
#define DIST_BTW_2000_TO_2400         11

#define GLOW_RED_LED4   12
#define GLOW_RED_LED5   13
#define GLOW_RED_LED6   14
#define GLOW_RED_LED7   15
#define GLOW_RED_LED8   16
#define GLOW_RED_LED9   17


#define YES  1
#define NO   0

#define IS_ENGINE_ON    0         
#define IS_IT_TOP_GEAR  1
#define IS_CRUISE_ON    2  
#define IS_BRAKE_ON     3
#define IS_GAS_ON       4
#define IS_SW4_ON       5
#define IS_SW5_ON       6
#define IS_SW6_ON       7
#define IS_SW7_ON       8
#define IS_SW8_ON       9
#define IS_SW9_ON       10


/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonTask_Stack[TASK_STACKSIZE];
OS_STK SwitchTask_Stack[TASK_STACKSIZE];
OS_STK WatchdogTask_Stack[TASK_STACKSIZE];
OS_STK OverloadTask_Stack[TASK_STACKSIZE];
OS_STK IWillOverloadTask_Stack[TASK_STACKSIZE];

// Task Priorities

#define WATCHDOGTASK_PRIO       4 
#define STARTTASK_PRIO          5
#define BUTTONTASK_PRIO         6
#define SWITCHTASK_PRIO         8
#define VEHICLETASK_PRIO        10
#define CONTROLTASK_PRIO        12
#define IWILLOVERLOADTASK_PRIO  14
#define OVERLOADTASK_PRIO       16

// Task Periods

#define CONTROL_PERIOD          300
#define VEHICLE_PERIOD          300
#define BUTTON_PERIOD           300
#define SWITCH_PERIOD           300
#define WATCHDOG_PERIOD         300
#define OVERLOAD_PERIOD         300
#define IWILLOVERLOAD_PERIOD    300

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;

// Semaphores
OS_EVENT *Sem_CruiseControl;
OS_EVENT *Sem_GasPedal;
OS_EVENT *Sem_BrakePedal;
OS_EVENT *Sem_Engine;
OS_EVENT *Sem_Gear;
OS_EVENT *Sem_OK;

// SW-Timer

/*
 * Types
 */
enum active {on, off};
enum active gas_pedal = off;
enum active brake_pedal = off;
enum active top_gear = off;
enum active engine = off;
enum active cruise_control = off; 



void LED_Ctrl(int,int);
int READ_Ctrl(int);
void Display_Dist_By_Red_Leds(unsigned short);
short mod_val(short);
void start_measurement(void);
void stop_measurement(void);
float microseconds(int);


/*
 * Global variables
 */
int delay; // Delay of HW-timer 
volatile unsigned long long g_led_green = 0; // Green LEDs
volatile unsigned long long g_led_red = 0;   // Red LEDs

volatile unsigned char g_Is_Engine_On = 0;
volatile unsigned char g_Is_It_Top_Gear = 0;
volatile unsigned char g_Is_Cruising_On = 0;
volatile unsigned char g_Is_Brake_On = 0;
volatile unsigned char g_Is_Gas_On = 0;

volatile long g_Curr_Velocity = 0;
volatile long g_Curr_Throttle = 0;

volatile unsigned long ticks = 0;
volatile unsigned long time_1 = 0;
volatile unsigned long time_2 = 0;
volatile unsigned long timer_overhead = 0;


volatile unsigned char g_Should_I_Reset_Watchdog = 0;


int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

int READ_Ctrl(int READ_Type)
{
    int return_val = 0;
    long temp = 0;
    
    switch(READ_Type)
    {
        case IS_ENGINE_ON :
        {
            temp = switches_pressed();
            
            if((temp & 0x01) == 0x01)
            return_val = YES;
            else
            return_val = NO;
        }  
        break;     
        case IS_IT_TOP_GEAR :
        {
            temp = switches_pressed();
            
            if((temp & 0x02) == 0x02)
            return_val = YES;
            else
            return_val = NO;
        }  
        break;      
        case IS_CRUISE_ON :
        {
            temp = buttons_pressed();
            
            if((temp & 0x02) == 0x02)
            return_val = YES;
            else
            return_val = NO;
        }  
        break;      
        case IS_BRAKE_ON :
        {
            temp = buttons_pressed();
            
            if((temp & 0x04) == 0x04)
            return_val = YES;
            else
            return_val = NO;
        }  
        break;      
        case IS_GAS_ON :
        {
            temp = buttons_pressed();
            
            if((temp & 0x08) == 0x08)
            return_val = YES;
            else
            return_val = NO;
        }  
        break; 
        case IS_SW4_ON :
        {
            temp = buttons_pressed();
            
            if((temp & 0x10) == 0x10)
            return_val = YES;
            else
            return_val = NO;
        }  
        break;
        case IS_SW5_ON :
        {
            temp = buttons_pressed();
            
            if((temp & 0x20) == 0x20)
            return_val = YES;
            else
            return_val = NO;
        }  
        break;
        case IS_SW6_ON :
        {
            temp = buttons_pressed();
            
            if((temp & 0x40) == 0x40)
            return_val = YES;
            else
            return_val = NO;
        }  
        break;
        case IS_SW7_ON :
        {
            temp = buttons_pressed();
            
            if((temp & 0x80) == 0x80)
            return_val = YES;
            else
            return_val = NO;
        }  
        break;
        case IS_SW8_ON :
        {
            temp = buttons_pressed();
            
            if((temp & 0x100) == 0x100)
            return_val = YES;
            else
            return_val = NO;
        }  
        break;
        case IS_SW9_ON :
        {
            temp = buttons_pressed();
            
            if((temp & 0x200) == 0x200)
            return_val = YES;
            else
            return_val = NO;
        }  
        break;      
    }   
    return(return_val);
}


void LED_Ctrl(int LED_Type, int ON_OFF)
{
    switch(LED_Type)
    {
        case ENGINE      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_0;
            else
                g_led_red &= ~LED_RED_0;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case GEAR        :
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_1;
            else
                g_led_red &= ~LED_RED_1;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        }       
        break;
        case CRUISE      :  
        {
            if(ON_OFF == ON)
                g_led_green |= LED_GREEN_0;
            else
                g_led_green &= ~LED_GREEN_0;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE,g_led_green);
        }       
        break;
        case CR_BUTTON   :
        {
            if(ON_OFF == ON)
                g_led_green |= LED_GREEN_2;
            else
                g_led_green &= ~LED_GREEN_2;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE,g_led_green);
        }       
        break;
        case BRAKE       :
        {
            if(ON_OFF == ON)
                g_led_green |= LED_GREEN_4;
            else
                g_led_green &= ~LED_GREEN_4;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE,g_led_green);
        }       
        break; 
        case GAS         :
        {
            if(ON_OFF == ON)
                g_led_green |= LED_GREEN_6;
            else
                g_led_green &= ~LED_GREEN_6;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE,g_led_green);
        }
        break;
        case DIST_BTW_0000_TO_0400      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_17;
            else
                g_led_red &= ~LED_RED_17;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case DIST_BTW_0400_TO_0800      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_16;
            else
                g_led_red &= ~LED_RED_16;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case DIST_BTW_0800_TO_1200      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_15;
            else
                g_led_red &= ~LED_RED_15;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case DIST_BTW_1200_TO_1600      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_14;
            else
                g_led_red &= ~LED_RED_14;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case DIST_BTW_1600_TO_2000      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_13;
            else
                g_led_red &= ~LED_RED_13;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case DIST_BTW_2000_TO_2400      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_12;
            else
                g_led_red &= ~LED_RED_12;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;    
        case GLOW_RED_LED4      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_4;
            else
                g_led_red &= ~LED_RED_4;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case GLOW_RED_LED5      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_5;
            else
                g_led_red &= ~LED_RED_5;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case GLOW_RED_LED6      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_6;
            else
                g_led_red &= ~LED_RED_6;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case GLOW_RED_LED7      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_7;
            else
                g_led_red &= ~LED_RED_7;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case GLOW_RED_LED8      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_8;
            else
                g_led_red &= ~LED_RED_8;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
        case GLOW_RED_LED9      :  
        {
            if(ON_OFF == ON)
                g_led_red |= LED_RED_9;
            else
                g_led_red &= ~LED_RED_9;
            
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,g_led_red);
        } 
        break;      
               
    }
 }


/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  //OSTmrSignal(); /* Signals a 'tick' to the SW timers */
  
  return delay;
}

static int b2sLUT[] = {0x40, //0
                 0x79, //1
                 0x24, //2
                 0x30, //3
                 0x19, //4
                 0x12, //5
                 0x02, //6
                 0x78, //7
                 0x00, //8
                 0x18, //9
                 0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
    return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
     out_sign = int2seven(10);
     tmp *= -1;
  }else{
     out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);
  
  out = int2seven(0) << 21 |
            out_sign << 14 |
            out_high << 7  |
            out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
}

/*
 * The function 'adjust_position()' adjusts the position depending on the
 * acceleration and velocity.
 */
 INT16U adjust_position(INT16U position, INT16S velocity,
                        INT8S acceleration, INT16U time_interval)
{
  INT16S new_position = position + velocity * time_interval / 1000
    + acceleration / 2  * (time_interval / 1000) * (time_interval / 1000);

  if (new_position > 24000) {
    new_position -= 24000;
  } else if (new_position < 0){
    new_position += 24000;
  }
  
  show_position(new_position);
  return new_position;
}
 
/*
 * The function 'adjust_velocity()' adjusts the velocity depending on the
 * acceleration.
 */
INT16S adjust_velocity(INT16S velocity, INT8S acceleration,  
               enum active brake_pedal, INT16U time_interval)
{
  INT16S new_velocity;
  INT8U brake_retardation = 200;

  if (brake_pedal == off)
    new_velocity = velocity  + (float) (acceleration * time_interval) / 1000.0;
  else {
    if (brake_retardation * time_interval / 1000 > velocity)
      new_velocity = 0;
    else
      new_velocity = velocity - brake_retardation * time_interval / 1000;
  }
  
  return new_velocity;
}

short mod_val(short input)
{
    short output = 0;
    
    if(input<0)
        output = input * -1;
    else
        output = input;
        
    return(output);  
}

void WatchdogTask(void* pdata)
{
    INT8U err;
    printf("\n\rWatchdog Task created ");
    while (1)
    { 
        //WATCHDOG
        OSSemPend(Sem_OK,0,&err);        
        if(g_Should_I_Reset_Watchdog == YES)
        {
            g_Should_I_Reset_Watchdog = 0;
        }
        else
        {   
            printf("******** Warning Over Load Detected************ \n");
        }
          
        OSSemPost(Sem_OK);          
        OSTimeDlyHMSM(0, 0, 0, 1);        
    }
}

void OverloadTask(void* pdata)
{
    INT8U err;
    printf("\n\rOverload Task created ");
    while (1)
    { 
        //OVERLOAD
        OSSemPend(Sem_OK,0,&err);        
        g_Should_I_Reset_Watchdog = YES;
        OSSemPost(Sem_OK);          
        OSTimeDlyHMSM(0, 0, 0, 1);        
    }
}


void start_measurement(void)
{
      /* Flush caches */
      alt_dcache_flush_all();
      alt_icache_flush_all();   
      /* Measure */
      alt_timestamp_start();
      time_1 = alt_timestamp();
}

void stop_measurement(void)
{
      time_2 = alt_timestamp();
      ticks = time_2 - time_1;
}

float microseconds(int ticks)
{
  return (float) 1000000 * (float) ticks / (float) alt_timestamp_freq();
}


void IWillOverloadTask(void* pdata)
{
    INT8U err;
    int Switch1 = 0, Switch2 = 0, Switch3 = 0, Switch4 = 0, Switch5 = 0, Switch6 = 0;
    double i = 0,Total = 0;
    printf("\n\rI Will Overload Task created ");
    
    while(1)
    {
 //        start_measurement();
         OSSemPend(Sem_OK,0,&err);        
         
         if(READ_Ctrl(IS_SW4_ON) == YES){LED_Ctrl(GLOW_RED_LED4,ON);Switch1 = 2;}
         else{LED_Ctrl(GLOW_RED_LED4,OFF);}
         if(READ_Ctrl(IS_SW5_ON) == YES){LED_Ctrl(GLOW_RED_LED5,ON);Switch2 = 4;}
         else{LED_Ctrl(GLOW_RED_LED5,OFF);}
         if(READ_Ctrl(IS_SW6_ON) == YES){LED_Ctrl(GLOW_RED_LED6,ON);Switch3 = 8;}
         else{LED_Ctrl(GLOW_RED_LED6,OFF);}
         if(READ_Ctrl(IS_SW7_ON) == YES){LED_Ctrl(GLOW_RED_LED7,ON);Switch4 = 16;}
         else{LED_Ctrl(GLOW_RED_LED7,OFF);}
         if(READ_Ctrl(IS_SW8_ON) == YES){LED_Ctrl(GLOW_RED_LED8,ON);Switch5 = 32;}
         else{LED_Ctrl(GLOW_RED_LED8,OFF);}
         if(READ_Ctrl(IS_SW9_ON) == YES){LED_Ctrl(GLOW_RED_LED9,ON);Switch6 = 64;}
         else{LED_Ctrl(GLOW_RED_LED9,OFF);}
         
        Total = Switch1 + Switch2 + Switch3 + Switch4 + Switch5 + Switch6;
        
        if(Total>50)
            {Total = 50;}
        
        Total = Total * microseconds(6);
        for(t:0; t<usage ; t++);
//        for(i=0;i<Total;i++);   // this takes 6 ms
{6millieconds delay}
        
        OSSemPost(Sem_OK); 
//        stop_measurement();         
        OSTimeDlyHMSM(0, 0, 0, 6); 
    }       
}

  


void ButtonTask(void* pdata)
{
    INT8U err;
    printf("\n\rButton Task created ");
    while (1)
    { 
        //CRUISING
        OSSemPend(Sem_CruiseControl,0,&err);        
        if(READ_Ctrl(IS_CRUISE_ON) == YES)
        {
           LED_Ctrl(CR_BUTTON,ON);
           
           if((g_Is_Cruising_On == NO)
           &&(g_Is_It_Top_Gear == YES)
           &&(g_Is_Engine_On == YES)
           &&(g_Curr_Velocity > mod_val(50)))
           {           
                cruise_control = on;
                g_Is_Cruising_On = YES;
                g_Curr_Throttle = 40;
           }
           else if (g_Is_It_Top_Gear == NO)
           {
                cruise_control = off;
                g_Is_Cruising_On = NO;
                
                OSSemPend(Sem_BrakePedal,0,&err);
                g_Is_Brake_On = YES;
                brake_pedal = on;
                OSSemPost(Sem_BrakePedal);
           }
           else if(g_Curr_Velocity < mod_val(50))
           {
               cruise_control = off;
               g_Is_Cruising_On = NO;
                
                OSSemPend(Sem_BrakePedal,0,&err);
                g_Is_Brake_On = YES;
                brake_pedal = on;
                OSSemPost(Sem_BrakePedal);
           }
           else if(g_Is_Cruising_On == YES)
           {           
                cruise_control = off;
                g_Is_Cruising_On = NO;
                
                OSSemPend(Sem_BrakePedal,0,&err);
                g_Is_Brake_On = YES;
                brake_pedal = on;
                OSSemPost(Sem_BrakePedal);
           }
        }
        else
        {
           LED_Ctrl(CR_BUTTON,OFF);
           if( (g_Is_It_Top_Gear == NO)&&(g_Is_Cruising_On == YES))
           {
                cruise_control = off;
                g_Is_Cruising_On = NO;
                
                OSSemPend(Sem_BrakePedal,0,&err);
                g_Is_Brake_On = YES;
                brake_pedal = on;
                OSSemPost(Sem_BrakePedal);
           }
           else if((g_Curr_Velocity < mod_val(50))&&(g_Is_Cruising_On == YES))
           {
               cruise_control = off;
               g_Is_Cruising_On = NO;
                
                OSSemPend(Sem_BrakePedal,0,&err);
                g_Is_Brake_On = YES;
                brake_pedal = on;
                OSSemPost(Sem_BrakePedal);
           }
           
        }
        OSSemPost(Sem_CruiseControl);
        
         if(g_Is_Cruising_On == YES)
            LED_Ctrl(CRUISE,ON);
         else
            LED_Ctrl(CRUISE,OFF);
            
        
        //ACCELERATOR OR GAS
        OSSemPend(Sem_GasPedal,0,&err);
        if(READ_Ctrl(IS_GAS_ON) == YES)
        {
           LED_Ctrl(GAS,ON);
           g_Is_Gas_On = YES;
           gas_pedal = on;
           
           if((g_Is_Cruising_On == NO)&&(g_Curr_Throttle <= 75))
            g_Curr_Throttle += 5;
        }
        else
        {
           LED_Ctrl(GAS,OFF);
           g_Is_Gas_On = NO;
           gas_pedal = off;          
        }
        OSSemPost(Sem_GasPedal);
         
         if(g_Is_Gas_On == YES)
            LED_Ctrl(GAS,ON);
         else
            LED_Ctrl(GAS,OFF);
        
        
        
        //BRAKE
        OSSemPend(Sem_BrakePedal,0,&err);
        if(READ_Ctrl(IS_BRAKE_ON) == YES)
        {
           LED_Ctrl(BRAKE,ON);
           g_Is_Brake_On = YES;
           brake_pedal = on;
           
          if((g_Is_Cruising_On == NO)&&(g_Curr_Throttle > 5))
            g_Curr_Throttle -= 5;
        }
        else
        {
           LED_Ctrl(BRAKE,OFF);
           g_Is_Brake_On = NO;
           brake_pedal = off;          
        }
        
        OSSemPost(Sem_BrakePedal);
        
        if(g_Is_Brake_On == YES)
            LED_Ctrl(BRAKE,ON);
         else
            LED_Ctrl(BRAKE,OFF);
  
        OSTimeDlyHMSM(0, 0, 0, 11);
    }
}

void SwitchTask(void* pdata)
{
    INT8U err;
    printf("\n\rSwitch Task created ");
    while (1)
    { 
        //ENGINE
        OSSemPend(Sem_Engine,0,&err);        
        if(READ_Ctrl(IS_ENGINE_ON) == YES)
        {
           if((g_Curr_Velocity == 0)&&(g_Is_Engine_On == YES))
           {
                g_Is_Engine_On = NO;
                
                OSSemPend(Sem_CruiseControl,0,&err);
                g_Is_Cruising_On = NO;
                OSSemPost(Sem_CruiseControl);
                
                OSSemPend(Sem_GasPedal,0,&err);
                g_Is_Gas_On = NO;
                OSSemPost(Sem_GasPedal);
                
                engine = off;
                cruise_control = off;
                gas_pedal = off;
           }
           else if((g_Curr_Velocity == 0)&&(g_Is_Engine_On == NO))
           {
                g_Is_Engine_On = YES;
                engine = on;
           }
        }
        OSSemPost(Sem_Engine);
        
        if(g_Is_Engine_On == YES)
            LED_Ctrl(ENGINE,ON);
        else
            LED_Ctrl(ENGINE,OFF);
        
        
        
        //GEAR
        OSSemPend(Sem_Gear,0,&err);
        if(READ_Ctrl(IS_IT_TOP_GEAR) == YES)
        {
           g_Is_It_Top_Gear = YES;
           top_gear = on;
        }
        else
        {
           g_Is_It_Top_Gear = NO;
           top_gear = off;          
        }
        OSSemPost(Sem_Gear);
        
         if(g_Is_It_Top_Gear == YES)
            LED_Ctrl(GEAR,ON);
         else
            LED_Ctrl(GEAR,OFF);
        
        
        OSTimeDlyHMSM(0, 0, 0, 12);
    }
}


/*
 * The task 'VehicleTask' updates the current velocity of the vehicle
 */
void VehicleTask(void* pdata)
{ 
    INT8U err;  
    void* msg;
    INT8U* throttle; 
    INT8S acceleration;  /* Value between 40 and -20 (4.0 m/s^2 and -2.0 m/s^2) */
    INT8S retardation;   /* Value between 20 and -10 (2.0 m/s^2 and -1.0 m/s^2) */
    INT16U position = 0; /* Value between 0 and 20000 (0.0 m and 2000.0 m)  */
    INT16S velocity = 0; /* Value between -200 and 700 (-20.0 m/s amd 70.0 m/s) */
    INT16S wind_factor;   /* Value between -10 and 20 (2.0 m/s^2 and -1.0 m/s^2) */

    unsigned short old_position = 0;
    short old_velocity = 0;
    int old_throttle = 0;

    printf("\n\rVehicle task created");

    while(1)
    {
        err = OSMboxPost(Mbox_Velocity, (void *) &velocity);

        OSTimeDlyHMSM(0,0,0,VEHICLE_PERIOD); 

        if(g_Is_Engine_On == YES)
        {    
            if(g_Is_Cruising_On == YES)
            {
                /* Non-blocking read of mailbox: 
                - message in mailbox: update throttle
                - no message:         use old throttle
                */
                msg = OSMboxPend(Mbox_Throttle, 1, &err); 
                if (err == OS_NO_ERR) 
                throttle = (INT8U*) msg;

                /* Retardation : Factor of Terrain and Wind Resistance */
                if (velocity > 0)
                    wind_factor = velocity * velocity / 10000 + 1;
                else 
                    wind_factor = (-1) * velocity * velocity / 10000 + 1;

                if (position < 4000) 
                    retardation = wind_factor; // even ground
                else if (position < 8000)
                    retardation = wind_factor + 15; // traveling uphill
                else if (position < 12000)
                    retardation = wind_factor + 25; // traveling steep uphill
                else if (position < 16000)
                    retardation = wind_factor; // even ground
                else if (position < 20000)
                    retardation = wind_factor - 10; //traveling downhill
                else
                    retardation = wind_factor - 5 ; // traveling steep downhill

                acceleration = *throttle / 2 - retardation;   
            }
            else
                acceleration = g_Curr_Throttle / 2 - retardation;   
                        
            position = adjust_position(position, velocity, acceleration, 300); 
            velocity = adjust_velocity(velocity, acceleration, brake_pedal, 300); 

            Display_Dist_By_Red_Leds(position/10);
            g_Curr_Velocity = velocity;

            if((old_position != position)||(old_velocity != velocity)||(old_throttle != *throttle))
            {
                printf("Position: %dm\n", position / 10);
                printf("Velocity: %4.1fm/s\n", velocity /10.0);
                printf("Throttle: %dV\n", *throttle / 10);
                show_velocity_on_sevenseg((INT8S) (velocity / 10));                
            }
        }
    }
} 

void Display_Dist_By_Red_Leds(unsigned short curr_position)
{
    if(curr_position > 000 ){LED_Ctrl(DIST_BTW_0000_TO_0400,ON);}else{LED_Ctrl(DIST_BTW_0000_TO_0400,OFF);}
    if(curr_position > 400 ){LED_Ctrl(DIST_BTW_0400_TO_0800,ON);}else{LED_Ctrl(DIST_BTW_0400_TO_0800,OFF);}
    if(curr_position > 800 ){LED_Ctrl(DIST_BTW_0800_TO_1200,ON);}else{LED_Ctrl(DIST_BTW_0800_TO_1200,OFF);}
    if(curr_position > 1200){LED_Ctrl(DIST_BTW_1200_TO_1600,ON);}else{LED_Ctrl(DIST_BTW_1200_TO_1600,OFF);}
    if(curr_position > 1600){LED_Ctrl(DIST_BTW_1600_TO_2000,ON);}else{LED_Ctrl(DIST_BTW_1600_TO_2000,OFF);}
    if(curr_position > 2000){LED_Ctrl(DIST_BTW_2000_TO_2400,ON);}else{LED_Ctrl(DIST_BTW_2000_TO_2400,OFF);}
        
}
/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 0; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S* current_velocity;

  printf("\n\rControl Task created ");
  printf("\n\rPlease start the engine \n\r");

    while(1)
    {
        if(g_Is_Engine_On == YES)
        {
          throttle = g_Curr_Throttle;
            
          msg = OSMboxPend(Mbox_Velocity, 0, &err);
          current_velocity = (INT16S*) msg;
          
          err = OSMboxPost(Mbox_Throttle, (void *) &throttle);
        
          OSTimeDlyHMSM(0,0,0, CONTROL_PERIOD);
        }
     }
}

/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */
  
  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
      delay,
      alarm_handler,
      context) < 0)
      {
          printf("No system clock available!n");
      }

  /* 
   * Create and start Software Timer 
   */

  /*
   * Creation of Kernel Objects
   */
  
  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
   
  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
       ControlTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       CONTROLTASK_PRIO,
       CONTROLTASK_PRIO,
       (void *)&ControlTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
       VehicleTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       VEHICLETASK_PRIO,
       VEHICLETASK_PRIO,
       (void *)&VehicleTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
  

  err = OSTaskCreateExt(
       ButtonTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &ButtonTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       BUTTONTASK_PRIO,
       BUTTONTASK_PRIO,
       (void *)&ButtonTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
  

  err = OSTaskCreateExt(
       SwitchTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &SwitchTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       SWITCHTASK_PRIO,
       SWITCHTASK_PRIO,
       (void *)&SwitchTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
  

  err = OSTaskCreateExt(
       WatchdogTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &WatchdogTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       WATCHDOGTASK_PRIO,
       WATCHDOGTASK_PRIO,
       (void *)&WatchdogTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
  

  err = OSTaskCreateExt(
       OverloadTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &OverloadTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       OVERLOADTASK_PRIO,
       OVERLOADTASK_PRIO,
       (void *)&OverloadTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
       IWillOverloadTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &IWillOverloadTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       IWILLOVERLOADTASK_PRIO,
       IWILLOVERLOADTASK_PRIO,
       (void *)&IWillOverloadTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);      
  
  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");
    
  LED_Ctrl(ENGINE,OFF);
  LED_Ctrl(GEAR,OFF);
  LED_Ctrl(CRUISE,OFF);
  LED_Ctrl(CR_BUTTON,OFF);
  LED_Ctrl(BRAKE,OFF);
  LED_Ctrl(GAS,OFF);
  
  
    
  OSTaskCreateExt(
     StartTask, // Pointer to task code
         NULL,      // Pointer to argument that is
                    // passed to task
         (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack 
         STARTTASK_PRIO,
         STARTTASK_PRIO,
         (void *)&StartTask_Stack[0],
         TASK_STACKSIZE,
         (void *) 0,  
         OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
         
  OSStart();
  
  return 0;
}
