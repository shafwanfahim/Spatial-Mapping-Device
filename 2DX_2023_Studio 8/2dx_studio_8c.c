/*  Time of Flight for 2DX2 -- Studio W8-0 
// 2DX3 Final Deliverable 
// Written By: Shafwan Fahim - 400388847 - fahims5 
// Date: April 11th 2023
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE
THIS IS AN EDITED VERSION FOR THE DELIVERABLE 2
*/

#include <stdint.h> 
#include <stdio.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

//port M is for the push button
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTM_DIR_R = 0b00000000;  //0b00000000    								      // Make PM0 and PM1 inputs 
  GPIO_PORTM_DEN_R = 0b00000001;  //0b00000011
	return;
	}

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
        
}

//stepper motor initialization 
void PortH_Init(void){
  //Use PortM pins for output
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                // activate clock for Port H
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};    // allow time for clock to stabilize
  GPIO_PORTH_DIR_R |= 0xFF;                                        // make PH0 out (PN0 built-in LED1)
  GPIO_PORTH_AFSEL_R &= ~0xFF;                                     // disable alt funct on PH0
  GPIO_PORTH_DEN_R |= 0xFF;                                        // enable digital I/O on PH0
                                                                                                    // configure PN1 as GPIO
  GPIO_PORTH_AMSEL_R &= ~0xFF;                                     // disable analog functionality on PH0        
  return;
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}


//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void spin(){ //spins the motor clockwise
	
    for(int i=0; i<32; i++){
			
        GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait10ms(1);
			
        GPIO_PORTH_DATA_R = 0b00000011; 
        SysTick_Wait10ms(1);
			
        GPIO_PORTH_DATA_R = 0b00000110; 
        SysTick_Wait10ms(1);
			
        GPIO_PORTH_DATA_R = 0b00001100; 
        SysTick_Wait10ms(1);
    }
} 
void spin_ccw(){ //spins the motor counterclockwise
	
    for(int i=0; i<32; i++){
			
        GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait10ms(1);
			
        GPIO_PORTH_DATA_R = 0b00001100; 
        SysTick_Wait10ms(1);
			
        GPIO_PORTH_DATA_R = 0b00000110; 
        SysTick_Wait10ms(1);
			
        GPIO_PORTH_DATA_R = 0b00000011; 
        SysTick_Wait10ms(1);
    }
}

//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	PortM_Init();

	int mynumber = 1;
/////////////////////////////////////////////////////
/* Those basic I2C read functions can be used to check your own I2C functions */

	status = VL53L1X_GetSensorId(dev, &wordData); //gets sensor id and stores in wordData
	
	status = VL53L1_RdByte(dev, 0x010F, &byteData); //for model ID (0xEA) reads byte data
	myByteArray[i++] = byteData;

	status = VL53L1_RdByte(dev, 0x0110, &byteData); //for module type (0xCC)
	myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData); //for both model ID and type

	
	
////////////////////////////////////////////////////

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState); //1
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev); //3 initializes the sensor
	Status_Check("SensorInit", status);  //checks if the sensor has been enabled
	
	
  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

	//STARTING HERE ARE THE EDITS 
	
	//END OF EDITS

	// Get the Distance Measures 8 times // 360/45 = 8 
	float degree = 22.5; 
	int count = 0 ;
	
	while(1){
	if((GPIO_PORTM_DATA_R & 0b00000001) == 0b00000001){
	for(int i = 0; i < 360/degree; i++) {
		
		
				
		//wait until the ToF sensor's data is ready
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady); //5
          FlashLED3(1);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		//read the data values from ToF sensor

	  status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value

    
		FlashLED4(1);

	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		// print the resulted readings to UART
		sprintf(printf_buffer,"%u\n", Distance);
		UART_printf(printf_buffer);
		spin();
	  SysTick_Wait10ms(50); 

  } 
//added for spin twice 
	for(int i = 0; i < 360/degree; i++) {
		
		
				
		//wait until the ToF sensor's data is ready
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady); //5
          FlashLED3(1);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		//read the data values from ToF sensor
	  status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value

    
		FlashLED4(1);

	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		// print the resulted readings to UART
		sprintf(printf_buffer,"%u\n", Distance);
		UART_printf(printf_buffer);
		spin_ccw();
	  SysTick_Wait10ms(50); 

  }  
	
//spin a third time
for(int i = 0; i < 360/degree; i++) {
		
		
				
		//wait until the ToF sensor's data is ready
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady); //5
          FlashLED3(1);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		//read the data values from ToF sensor
	  status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value

		FlashLED4(1);

	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		// print the resulted readings to UART
		sprintf(printf_buffer,"%u\n", Distance);
		UART_printf(printf_buffer);
		spin();
	  SysTick_Wait10ms(50); 

  } 	
  
	VL53L1X_StopRanging(dev);
  while(1) {}

}
}
}