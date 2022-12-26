//  Author: Vatsal Asitkumar Joshi
//  Date: May 23rd, 2022
//  This code is a set of functions to work with USB serial communication.
//
//  "If you are done writing the code, now is a good time to debug it."
//

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include <hardware/i2c.h>


#define TOUCH_ADD 0b1001000
    #define i2cSCLpin 1
    #define i2cSDApin 0

uint8_t sndData1[1] = {0b11000000};
    uint8_t recData1[2];
     float x;
   float y;





      #define IN1 16
#define IN2 15
  uint16_t tau = 500;
  uint16_t tau2 = 500;

// Define alarm numbers and IRQ vector numbers for touch panel reading timing
#define TOUCH_ALARM_NUM 0
#define TOUCH_IRQ TIMER_IRQ_0
#define TOUCH_CALLBACK_TIME 5000

// Define alarm numbers and IRQ vector numbers for the control loop
#define CONTROLLER_ALARM_NUM 1
#define CONTROLLER_IRQ TIMER_IRQ_1
#define CONTROLLER_CALLBACK_TIME 20000

// ISR to read touch screen values
void readTouch()
{
    // Acknowledge the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << TOUCH_ALARM_NUM);

    // Call readYDir ISR after 5 milli seconds
    timer_hw->alarm[TOUCH_ALARM_NUM] += TOUCH_CALLBACK_TIME;

    // Put your code to read the touchscreen below

   i2c_write_blocking(i2c0, TOUCH_ADD, sndData1, 1, true);
    i2c_read_blocking(i2c0, TOUCH_ADD, recData1, 2, false);
     x = ((((recData1[0] << 4)|(recData1[1] >>4))/4095.0)*165-82.5);
    

    uint8_t sndData2[1] = {0b11010000};
    uint8_t recData2[2];
    i2c_write_blocking(i2c0, TOUCH_ADD, sndData2, 1, true);
    i2c_read_blocking(i2c0, TOUCH_ADD, recData2, 2, false);

    y = ((((recData2[0] << 4) | (recData2[1] >>4))/4095.0)*105-52.5);
    //////////////////////////////////////////////
  
    
}

// ISR to controll the servo motors
void controllerLoop()
{
    // Acknowledge the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << CONTROLLER_ALARM_NUM);

    // Call this ISR after 20 milli seconds
    timer_hw->alarm[CONTROLLER_ALARM_NUM] += CONTROLLER_CALLBACK_TIME;

    // Put your code to calculate the servo output and setting PWM CC register below
pwm_set_gpio_level(IN1, tau);
 pwm_set_gpio_level(IN2, tau2);

  ++tau; 
   
    
    if (tau > 1000)
    {
        tau = 500;
    }
    

    
     ++tau2; 
   
   
    if (tau2 > 1000)
    {
        tau2 = 500;
    }
   

    ////////////////////////////////////////////////////////////////////////////////
}

// Function that initializes all the pins and hardware
void setup()
{
    stdio_init_all();

    // Set up ISRs and enable IRQ for touch panel readings of x and y direction
    irq_set_exclusive_handler(TOUCH_IRQ, readTouch);     // Define the callback function for reading touchscreen alarm
    hw_set_bits(&timer_hw->inte, 1u << TOUCH_ALARM_NUM); // Enable the interrupt for reading alarm
    irq_set_enabled(TOUCH_IRQ, true);                    // Enable the reading specific alarm irq

    // Set up ISR and enable IRQ for sending out servo signals
    irq_set_exclusive_handler(CONTROLLER_IRQ, controllerLoop); // Define the callback function for updating CC registers alarm
    hw_set_bits(&timer_hw->inte, 1u << CONTROLLER_ALARM_NUM);  // Enable the interrupt for updating CC registers alarm
    irq_set_enabled(CONTROLLER_IRQ, true);                     // Enable the CC registers update specific alarm irq

    // Call readTouch after 5 milliseconds
    uint t = time_us_32();                       // Get current time
    timer_hw->alarm[TOUCH_ALARM_NUM] = t + 5000; // Set the reading alarm to fire after 5 milli seconds

    // Call controllerLoop after 22.5 ms
    timer_hw->alarm[CONTROLLER_ALARM_NUM] = time_us_32() + 2500 + CONTROLLER_CALLBACK_TIME;

    // Put the setup code for I2C communication and PWM below



    gpio_set_function(i2cSCLpin, GPIO_FUNC_I2C);
    gpio_set_function(i2cSDApin, GPIO_FUNC_I2C);
    gpio_pull_up(i2cSCLpin);
    gpio_pull_up(i2cSDApin);
    i2c_init(i2c0, 100000);
     


    gpio_set_function(IN1, GPIO_FUNC_PWM);
    gpio_set_function(IN2, GPIO_FUNC_PWM);

    uint slice_num1 = pwm_gpio_to_slice_num(IN1);
    uint chan_num1 = pwm_gpio_to_channel(IN1);

    uint slice_num2 = pwm_gpio_to_slice_num(IN2);
    uint chan_num2 = pwm_gpio_to_channel(IN2);

    pwm_set_clkdiv_int_frac(slice_num1, 249,13);
    pwm_set_wrap(slice_num1, 1250);
    pwm_set_gpio_level(IN1, 0);
    pwm_set_gpio_level(IN2, 0);
    pwm_set_enabled(slice_num1, true);

     pwm_set_clkdiv_int_frac(slice_num2, 249,13);
    pwm_set_wrap(slice_num2, 1250);
    pwm_set_enabled(slice_num2, true);


    /////////////////////////////////////////////////////////
}

void loop()
{

   printf("%f\r\n ,%f\r\n",x, y);
   sleep_ms(1000);

 printf("%u\r\n",tau);
 sleep_ms(1000);
  
    
 printf("%u\r\n",tau2);
 sleep_ms(1000);

    
    

    
}

int main()
{
    setup();

    while (true)
    {
        loop();
    }
}