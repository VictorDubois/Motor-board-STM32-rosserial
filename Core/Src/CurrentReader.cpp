/*
	MCP3002 is Arduino Library for communicating with MCP3002 Analog to digital converter.
	Based on the MCP3008 Library created by Uros Petrevski, Nodesign.net 2013
	Released into the public domain.


	ported from Python code originaly written by Adafruit learning system for rPI :
	http://learn.adafruit.com/send-raspberry-pi-data-to-cosm/python-script
*/

#include <CurrentReader.h>


extern "C" {
	#include "main.h" // Include pin definitions
}

#define INVERTED_HIGH LOW
#define INVERTED_LOW HIGH

CurrentReaderAdc::CurrentReaderAdc(ADC_HandleTypeDef* a_ADC_Handle): m_ADC_Handle(a_ADC_Handle)
{}


int CurrentReaderMCP3002::readCurrent(int adcnum) {
	return CURRENT_READER_OFFLINE - this->readADC(adcnum);
}

int CurrentReaderAdc::readCurrent(int adcnum) {
	return readADC(adcnum);
}

int CurrentReaderCan::readCurrent(int adcnum)
{
	if (adcnum)
	{
		return m_current_right;
	}
	return m_current_left;
}


// read SPI data from MCP3002 chip, 8 possible adc's (0 thru 7)
int CurrentReaderAdc::readADC(int adcnum)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;

	if(adcnum == 1)
	{
		sConfig.Channel = ADC_CHANNEL_4;
	}

	if (HAL_ADC_ConfigChannel(m_ADC_Handle, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
	HAL_ADC_Start(m_ADC_Handle);

	int32_t ret_code = HAL_ADC_PollForConversion(m_ADC_Handle, 100);

	if (ret_code == HAL_TIMEOUT)
	{
	  HAL_ADC_Stop(m_ADC_Handle);
	  return -1;
	}

	uint32_t adc_val = HAL_ADC_GetValue(m_ADC_Handle);

	HAL_ADC_Stop(m_ADC_Handle);

	return adc_val;
}

int CurrentReaderMCP3002::readADC(int adcnum)
{
#ifdef USE_MCP3002
  if ((adcnum > 7) || (adcnum < 0)) return -1; // Wrong adc address return -1

  // algo
  //digitalWrite(_cspin, INVERTED_HIGH);
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
  //HAL_Delay(1);
  //digitalWrite(_clockpin, INVERTED_LOW); //  # start clock low
  HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_SET);//  # start clock low
  //HAL_Delay(1);
  //digitalWrite(_cspin, INVERTED_LOW); //     # bring CS low
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); //     # bring CS low
  //HAL_Delay(1);

  int commandout = adcnum*4; // 4 added so that 1 is converted to 4, this makes both channels (0 and 1) usable
  commandout |= 0x18; //  # start bit + single-ended bit
  commandout <<= 3; //    # we only need to send 5 bits here

  for (int i=0; i<5; i++) {
	if (commandout & 0x80){
		//digitalWrite(_mosipin, INVERTED_HIGH);
		HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, GPIO_PIN_RESET);
	}
	else {
	    //digitalWrite(_mosipin, INVERTED_LOW);
		HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, GPIO_PIN_SET);
	}
	//HAL_Delay(1);
	commandout <<= 1;
	//digitalWrite(_clockpin, INVERTED_HIGH);
	HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_RESET);
	//HAL_Delay(1);
	//digitalWrite(_clockpin, INVERTED_LOW);
	HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);

  }

  int adcout = 0;
  // read in one empty bit, one null bit and 10 ADC bits
  for (int i=0; i<12; i++) {
	  //digitalWrite(_clockpin, INVERTED_HIGH);
	  	HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_RESET);
	  	//HAL_Delay(1);
	  	//digitalWrite(_clockpin, INVERTED_LOW);
	  	HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_SET);
	  	//HAL_Delay(1);
	adcout <<= 1;
	//if (!digitalRead(_misopin)) {
	if (!HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin)) {
	  adcout |= 0x1;
	}
  }
  //digitalWrite(_cspin, INVERTED_HIGH);
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  adcout >>= 1; //      # first bit is 'null' so drop it
  return adcout;
#endif
  return 10;
}

