/*
	MCP3002 is Arduino Library for communicating with MCP3002 Analog to digital converter.
	Based on the MCP3008 Library created by Uros Petrevski, Nodesign.net 2013
	Released into the public domain.


	ported from Python code originaly written by Adafruit learning system for rPI :
	http://learn.adafruit.com/send-raspberry-pi-data-to-cosm/python-script
*/

#include <MCP3002.h>
#define INVERTED_HIGH LOW
#define INVERTED_LOW HIGH
MCP3002::MCP3002(GPIO_TypeDef* a_miso_gpio_bank,
		const uint16_t a_miso_gpio,
		GPIO_TypeDef* a_mosi_gpio_bank,
		const uint16_t a_mosi_gpio,
		GPIO_TypeDef* a_clk_bank,
		const uint16_t a_clk_gpio,
		GPIO_TypeDef* a_cs_gpio_bank,
		const uint16_t a_cs_gpio) {
	m_miso_gpio_bank = a_miso_gpio_bank;
	m_miso_gpio = a_miso_gpio;
	m_mosi_gpio_bank =  a_mosi_gpio_bank;
	m_mosi_gpio =  a_mosi_gpio;
	m_clk_bank = a_clk_bank;
	m_clk_gpio = a_clk_gpio;
	m_cs_gpio_bank = a_cs_gpio_bank;
	m_cs_gpio = a_cs_gpio;
}

int MCP3002::readCurrent(int adcnum) {
	return 2048 - readADC(adcnum);
}


// read SPI data from MCP3002 chip, 8 possible adc's (0 thru 7)
int MCP3002::readADC(int adcnum) {

  if ((adcnum > 7) || (adcnum < 0)) return -1; // Wrong adc address return -1

  // algo
  //digitalWrite(_cspin, INVERTED_HIGH);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  //HAL_Delay(1);
  //digitalWrite(_clockpin, INVERTED_LOW); //  # start clock low
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);//  # start clock low
  //HAL_Delay(1);
  //digitalWrite(_cspin, INVERTED_LOW); //     # bring CS low
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //     # bring CS low
  //HAL_Delay(1);

  int commandout = adcnum*4; // 4 added so that 1 is converted to 4, this makes both channels (0 and 1) usable
  commandout |= 0x18; //  # start bit + single-ended bit
  commandout <<= 3; //    # we only need to send 5 bits here

  for (int i=0; i<5; i++) {
	if (commandout & 0x80){
		//digitalWrite(_mosipin, INVERTED_HIGH);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	else {
	    //digitalWrite(_mosipin, INVERTED_LOW);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	}
	//HAL_Delay(1);
	commandout <<= 1;
	//digitalWrite(_clockpin, INVERTED_HIGH);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	//HAL_Delay(1);
	//digitalWrite(_clockpin, INVERTED_LOW);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	//HAL_Delay(1);

  }

  int adcout = 0;
  // read in one empty bit, one null bit and 10 ADC bits
  for (int i=0; i<12; i++) {
	  //digitalWrite(_clockpin, INVERTED_HIGH);
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  	//HAL_Delay(1);
	  	//digitalWrite(_clockpin, INVERTED_LOW);
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	  	//HAL_Delay(1);
	adcout <<= 1;
	//if (!digitalRead(_misopin)) {
	if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)) {
	  adcout |= 0x1;
	}
  }
  //digitalWrite(_cspin, INVERTED_HIGH);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  adcout >>= 1; //      # first bit is 'null' so drop it
  return adcout;
}

