/*
	MCP3002 is Arduino Library for communicating with MCP3002 Analog to digital converter.
	Based on the MCP3008 Library created by Uros Petrevski, Nodesign.net 2013
	Released into the public domain.


	ported from Python code originaly written by Adafruit learning system for rPI :
	http://learn.adafruit.com/send-raspberry-pi-data-to-cosm/python-script
*/

// Mesures :
// 105 <=> 360mA
// 69 <=> 240mA
// 53 <=> 186mA
// 77 <=> 267mA
// 89 <=> 301mA
// 137 <=> 480mA
// "ONE_VOLT" factor = measure/("ONE_AMP" factor * real current)
#define ONE_VOLT        774//2048/5 // TODO find the actual value
#define ONE_AMP         0.377*ONE_VOLT

#ifndef MCP3002_H_
#define MCP3002_H_
#include "stm32f3xx_hal.h"
#define CURRENT_READER_OFFLINE 2048

class MCP3002
{
  public:
	MCP3002(){};
    MCP3002(GPIO_TypeDef* a_miso_gpio_bank,
    		const uint16_t a_miso_gpio,
    		GPIO_TypeDef* a_mosi_gpio_bank,
    		const uint16_t a_mosi_gpio,
			GPIO_TypeDef* a_clk_bank,
			const uint16_t a_clk_gpio,
			GPIO_TypeDef* a_cs_gpio_bank,
			const uint16_t a_cs_gpio);
    int readADC(int adcnum);
    int readCurrent(int adcnum);
  private:
    GPIO_TypeDef* m_miso_gpio_bank;
	uint16_t m_miso_gpio;
	GPIO_TypeDef* m_mosi_gpio_bank;
	uint16_t m_mosi_gpio;
	GPIO_TypeDef* m_clk_bank;
	uint16_t m_clk_gpio;
	GPIO_TypeDef* m_cs_gpio_bank;
	uint16_t m_cs_gpio;
};

#endif /* MCP3002_H_ */
