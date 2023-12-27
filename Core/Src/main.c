/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t adraw[2];
uint8_t uart2_raw[10];
volatile int rx_flagA = 0;
volatile int rx_flagB = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[100];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}

char writeBuf[100];

char strA1[50];
volatile uint16_t ad1_raw[5];
const int adcChannelCount = 2;
volatile int adcConversionComplete = 0;
volatile int lock = 0;
volatile uint32_t millis = 0;
volatile uint32_t conv_rate = 0;

uint32_t ad1 = 0;
uint32_t ad2 = 0;

int32_t sawtooth_buf1[200];
int32_t sawtooth_buf2[200];
int32_t signal_buf[200];
int32_t signal_buf1[200];
int32_t signal_buf2[200];
int32_t kalman_buf1[200];
int32_t kalman_buf2[200];
int32_t peaks_buff1[200];
int32_t peaks_buff2[200];
volatile int signal_buffer_in_queue = 1;
volatile int gidxB = 0;
volatile int gidxA = 0;

volatile uint32_t relative_sawtooth_voltage = 0;

int FindPeak(uint32_t *sig)
{
	int fidxA = 0;

	if((sig[0] < sig[1]) && (sig[2] < sig[1]))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void insert_new_value(int32_t *buf, int32_t new_value)
{
	for(gidxA=0;gidxA<199;gidxA++)
	{
		buf[gidxA] = buf[gidxA+1];
	}

	buf[199] = new_value;
}

int flag_FallingEdge = 0;
int flag_saving = 0;

int file_name_index = 0;

FIL log_file;
int log_file_opened = 0;

static float ADC_OLD_Value;
static float P_k1_k1;

static float Q = 0.0001;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
//static float Q = 0.0005;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
static float R = 0.005; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better
//static float R = 0.2;
static float Kg = 0;
static float P_k_k1 = 0.5;
static float kalman_adc_old=0;
static int kalman_adc_int = 0;

unsigned long kalman_filter(unsigned long ADC_Value)
{
    float x_k1_k1,x_k_k1;
    //static float ADC_OLD_Value;
    float Z_k;


    float kalman_adc;

    Z_k = ADC_Value;
    x_k1_k1 = kalman_adc_old;

    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1/(P_k_k1 + R);

    kalman_adc = x_k_k1 + Kg * (Z_k - kalman_adc_old);
    P_k1_k1 = (1 - Kg)*P_k_k1;
    P_k_k1 = P_k1_k1;

    ADC_OLD_Value = ADC_Value;
    kalman_adc_old = kalman_adc;
    kalman_adc_int = (int)kalman_adc;
    return kalman_adc;
}

int startWriting(FIL *fil, char *fname)
{
	FRESULT fres; //Result after operations
	fres = f_open(fil, fname, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	if(fres == FR_OK)
	{
		myprintf("Writing '%s' \r\n", fname);
		log_file_opened = 1;
		return 1;
	}
	else
	{
		myprintf("f_open error %s (%i)\r\n", fname, fres);
		return -1;
	}
}

int keepWriting(FIL *fil, int data)
{
	FRESULT fres;
	char ch_data[6];
	UINT bytesWrote;
	if(log_file_opened == 0)
	{
		return -1;
	}
	sprintf(ch_data, "%04d\n", data);
	fres = f_write(fil, ch_data, 5, &bytesWrote);
	if(fres == FR_OK)
	{
		return 1;
	}
	else
	{
		myprintf("f_write error (%i)\r\n");
		return -1;
	}
}

int endWriting(FIL *fil)
{
	if(log_file_opened == 0)
	{
		return -1;
	}

	f_close(fil);

	log_file_opened = 0;

	return 1;
}

int getSimplifiedSlope(int32_t *buf)
{
	int32_t avg1 = (buf[0] + buf[1] + buf[2] + buf[3] + buf[4]) / 5;
	int32_t avg2 = (buf[5] + buf[6] + buf[7] + buf[8] + buf[9]) / 5;

	if((avg1 - avg2) > 1000)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void DMA_ADC_Complete(DMA_HandleTypeDef *_hdma)
{

}

int max = 0;
int min = 5000;

int gmaxA = 0;
int gminA = 0;
int midlineA = 0;
int dripOff = 0;

int32_t GetMidLine(int32_t *gbuff, uint32_t sz)
{
	int lidxA = 0;

	for(lidxA = 1; lidxA<(sz - 1 ); lidxA++)
	{
		if(gbuff[lidxA] > max)
		{
			max = gbuff[lidxA];
		}
	}

	for(lidxA = 1; lidxA<(sz - 1 ); lidxA++)
	{
		if(gbuff[lidxA] < min)
		{
			min = gbuff[lidxA];
		}
	}

	return (((max - min)/2) + min);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adcConversionComplete = 1;
	ad1 = adraw[0]; // HAL_ADC_GetValue(&hadc1);
	ad2 = adraw[1]; // HAL_ADC_GetValue(&hadc1);
	conv_rate++;

	//insert_new_value(sawtooth_buf, (int32_t)ad1);
	//insert_new_value(signal_buf, (int32_t)ad2);

	if(rx_flagA == 0)
	{
		if(gidxB == 200)
		{
			if(signal_buffer_in_queue == 1)
			{
				signal_buffer_in_queue = 2;
			}
			else
			{
				signal_buffer_in_queue = 1;
			}
			gidxB = 0;
		}

		if(gidxB > 5)
		{
			if(gmaxA < kalman_buf1[gidxB])
			{
				gmaxA = kalman_buf1[gidxB];
			}

			if(gminA > kalman_buf1[gidxB])
			{
				gminA = kalman_buf1[gidxB];
			}
		}

		if(gidxB == 195)
		{
			midlineA = (((gmaxA - gminA)/2) + gminA);
		}

		if(signal_buffer_in_queue == 1)
		{
			signal_buf1[gidxB] = ad1;
			sawtooth_buf1[gidxB] = ad2;
			kalman_buf1[gidxB] = kalman_filter(signal_buf1[gidxB]);

			if(gidxB==0)
			{
				gminA = kalman_buf1[0];
				gmaxA = kalman_buf1[0];
			}

			if((gidxB >= 5) && (gidxB < 190))
			{

				if(FindPeak(&kalman_buf1[gidxB-3]) && (kalman_buf1[gidxB-3] > midlineA))
				{
					if(dripOff == 0)
					{
						peaks_buff1[gidxB] = 2000;
						dripOff = 20;
						relative_sawtooth_voltage = (3300000 / 4096) * sawtooth_buf1[gidxB-3]; // sawtooth_buf1[gidxB-3]; //
					}
					else
					{
						peaks_buff1[gidxB] = 500;
					}
				}
				else
				{
					peaks_buff1[gidxB] = 500;
				}
			}
			else
			{
				peaks_buff1[gidxB] = 500;
			}
		}
		else
		{
			signal_buf2[gidxB] = ad1;
			sawtooth_buf2[gidxB] = ad2;
			kalman_buf2[gidxB] = kalman_filter(signal_buf2[gidxB]);

			if(gidxB==0)
			{
				gminA = kalman_buf2[0];
				gmaxA = kalman_buf2[0];
			}

			if((gidxB >= 5) && (gidxB < 190))
			{
				if(FindPeak(&kalman_buf2[gidxB-3]) && (kalman_buf2[gidxB-3] > midlineA))
				{
					if(dripOff == 0)
					{
						peaks_buff2[gidxB] = 2000;
						relative_sawtooth_voltage = (3300000 / 4096) * sawtooth_buf2[gidxB-3]; // sawtooth_buf2[gidxB-3]; //
						dripOff = 20;
					}
					else
					{
						peaks_buff2[gidxB] = 500;
					}
				}
				else
				{
					peaks_buff2[gidxB] = 500;
				}
			}
			else
			{
				peaks_buff2[gidxB] = 500;
			}
		}
		gidxB++;
		if(dripOff > 0)
		{
			dripOff--;
		}
	}

	// HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adraw, adcChannelCount);
}

uint32_t crate = 0;

uint32_t __t2_cntr = 0;
uint32_t __dripA = 0;
uint32_t flag_saving_interval = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		if(__t2_cntr < 3)
		{
			__t2_cntr++;
		}
		else
		{
			__t2_cntr = 0;
		}

		if(__dripA > 0)
		{
			__dripA++;

			if(__dripA > 1500) // 500uS
			{
				flag_FallingEdge = 0;
				__dripA = 0;
			}
		}

		if(flag_saving_interval > 0)
		{
			if(flag_saving_interval == 1)
			{
				endWriting(&log_file);
			}
			flag_saving_interval--;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		if(uart2_raw[0] == 'a')
		{
			rx_flagA = 1;
			//HAL_UART_Receive_IT(&huart2, uart2_raw, 1);
		}

		if(uart2_raw[0] == 'b')
		{
			rx_flagB = 1;
			//HAL_UART_Receive_IT(&huart2, uart2_raw, 1);
		}
	}
	HAL_UART_Receive_IT(&huart2, uart2_raw, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint32_t a_shot = 0;
	uint32_t b_shot = 0;

	int lidxA = 0;

	char log_file_name[10];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  	//HAL_ADC_Start_IT(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adraw, adcChannelCount);
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start(&htim2);

    HAL_UART_Receive_IT(&huart2, uart2_raw, 1);

    //myprintf("\r\n~ ADC Peak Detector ~\r\n\r\n");

    HAL_Delay(500); //a short delay is important to let the SD card settle

    //some variables for FatFs
    FATFS FatFs; 	//Fatfs handle
    FIL fil; 		//File handle
    FRESULT fres; //Result after operations
    UINT bytesWrote;
/*********************************************************/
    //Open the file system
//    fres = f_mount(&FatFs, "", 1); //1=mount now
//    if (fres != FR_OK) {
//  	myprintf("f_mount error (%i)\r\n", fres);
//  	while(1);
//    }
/*********************************************************/

/*********************************************************/
    //Let's get some statistics from the SD card
//    DWORD free_clusters, free_sectors, total_sectors;
//    FATFS* getFreeFs;
//    fres = f_getfree("", &free_clusters, &getFreeFs);
//    if (fres != FR_OK)
//    {
//    	myprintf("f_getfree error (%i)\r\n", fres);
//    	while(1);
//    }
/*********************************************************/

    //Formula comes from ChaN's documentation
    ////total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    ////free_sectors = free_clusters * getFreeFs->csize;

    ////myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

    //Now let's try to open file "test.txt"
  /*  fres = f_open(&fil, "test.txt", FA_READ);
    if (fres != FR_OK)
    {
    	myprintf("f_open error (%i)\r\n");
    	while(1);
    }
    myprintf("I was able to open 'test.txt' for reading!\r\n");
	*/
    //Read 30 bytes from "test.txt" on the SD card
    ////BYTE readBuf[30];

    //We can either use f_read OR f_gets to get data out of files
    //f_gets is a wrapper on f_read that does some string formatting for us
  /*  TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
    if(rres != 0)
    {
    	myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
    }
    else
    {
    	myprintf("f_gets error (%i)\r\n", fres);
    }
	*/

    //Be a tidy kiwi - don't forget to close your file!
    //f_close(&fil);

    //Now let's try and write a file "write.txt"
    /*
    fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
    if(fres == FR_OK)
    {
    	myprintf("I was able to open 'write.txt' for writing\r\n");
    }
    else
    {
    	myprintf("f_open error (%i)\r\n", fres);
    }
    */

    //Copy in a string
    //strncpy((char*)readBuf, "a new file is made!", 19);
    //readBuf[19] = '\0'; // Add the null terminator
    /*
    fres = f_write(&fil, readBuf, 19, &bytesWrote);
    if(fres == FR_OK)
    {
    	myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
    }
    else
    {
    	myprintf("f_write error (%i)\r\n");
    }
    */

    //Be a tidy kiwi - don't forget to close your file!
    //f_close(&fil);

    //We're done, so de-mount the drive
    //f_mount(NULL, "", 0);

    // Mount the file system
    /*    if(f_mount(&FatFs, "", 0) != FR_OK) {
          myprintf("ERROR : Cannot mount SD CARD.\r\n");
          while(1);
        }
     */

        // Open or create the file to write PA0 and PA1 data
        /*
    	fres = f_open(&fil, "Data.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
        if(fres != FR_OK) {
          myprintf("Failed to open 'Data.txt' for writing\r\n");
          while(1);
        }
        // Move the file pointer to the end, so new data won't overwrite existing data
        f_lseek(&fil, f_size(&fil));
        */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t millis = 0;

    while(1)
    {
    	millis = HAL_GetTick();

		if(HAL_GetTick() > (a_shot + 500))
		{
		  a_shot = HAL_GetTick();

//		  if(adcConversionComplete == 1)
//		  {
//			  adcConversionComplete = 0;
//			  sprintf(strA1, "A0 - %d A1 - %d CONV RATE: %d\r\n", ad1, ad2, conv_rate);
//			  myprintf("%s", strA1);
//			  ////HAL_UART_Transmit(&huart2, strA1, strlen(strA1), 0xFFFF);
//			  crate = conv_rate;
//			  conv_rate = 0;
//
//		  }

		  float t = millis/1000.0;
		  if(rx_flagA == 1)
		  {
			  if(adcConversionComplete == 1)
			  {
				  adcConversionComplete = 0;
				  for(lidxA=0;lidxA<200;lidxA++)
				  {
					  //myprintf("A0:%d, A1:%d\n", signal_buf[lidxA], sawtooth_buf[lidxA]);
					  if(signal_buffer_in_queue == 2)
					  {
						  //myprintf("A0:%d\n", signal_buf1[lidxA]);
						  myprintf("%d,%d,%d,%d,%d\r\n", signal_buf1[lidxA], sawtooth_buf1[lidxA], kalman_buf1[lidxA], peaks_buff1[lidxA], midlineA); //GetMidLine(kalman_buf1, 200));
					  }
					  else
					  {
						  //myprintf("A0:%d\n", signal_buf2[lidxA]);
						  myprintf("%d,%d,%d,%d,%d\r\n", signal_buf2[lidxA], sawtooth_buf2[lidxA], kalman_buf2[lidxA], peaks_buff2[lidxA], midlineA); // GetMidLine(kalman_buf2, 200));
					  }
				  }
			  }
			  HAL_Delay(2500);
			  gidxB = 0; // Fresh Copy of ADC
			  rx_flagA = 0;
			  rx_flagB = 0;
		  }

		  if(rx_flagB == 1)
		  {
			  myprintf("Sawtooth Voltage : %d\r\n", relative_sawtooth_voltage);
			  HAL_Delay(100);
			  rx_flagB = 0;
		  }
		}


//		if(flag_FallingEdge == 0)
//		{
//			if(getSimplifiedSlope(sawtooth_buf) == 1)
//			{
//				flag_FallingEdge = 1;
//				__dripA = 1;
//
//				if(flag_saving_interval == 0)
//				{
//					file_name_index++;
//					sprintf(log_file_name, "%04d.txt", file_name_index);
//					startWriting(&log_file, log_file_name);
//					flag_saving = 1;
//					flag_saving_interval = 1000000;
//				}
//			}
//		}
    }
//  while (1)
//  {
//	  //Read PA0 and PA1 data
//	    HAL_ADC_Start(&hadc1);
//	    if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
//	      adcValuePA0 = HAL_ADC_GetValue(&hadc1);
//	    }
//	    HAL_ADC_PollForConversion(&hadc2, 5);
//	    adcValuePA1 = HAL_ADC_GetValue(&hadc2);
//	    HAL_ADC_Stop(&hadc1);
//
//	    //Write PA0 and PA1 data to file
//	    char writeBuf[100];
//	    sprintf(writeBuf, "%lu,%lu\r\n", adcValuePA0, adcValuePA1);
//
//	    UINT bytesWrote;
//	    f_write(&fil, writeBuf, strlen(writeBuf), &bytesWrote);
//
//	    // Force the data to be written to the disk
//	    f_sync(&fil);
//
//	    // Blink the LED every second
//	   // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//	    HAL_Delay(5);
//	  }
        /*
        while (1)
        {



        	uint32_t startTimestamp = HAL_GetTick(); // Record start timestamp

            // Sample 4096 times per period
            for (int i = 0; i < 4096; ++i) {
              // Read ADC data from A0 and A1
              HAL_ADC_Start(&hadc1);
              HAL_ADC_Start(&hadc2);

              if (HAL_ADC_PollForConversion(&hadc1, 1000) != HAL_OK ||
                  HAL_ADC_PollForConversion(&hadc2, 1000) != HAL_OK) {
                Error_Handler(); // Handle ADC conversion error
              }

              uint32_t adcValuePA0 = HAL_ADC_GetValue(&hadc1);
              uint32_t adcValuePA1 = HAL_ADC_GetValue(&hadc2);

              // Get current timestamp
              RTC_TimeTypeDef currentTime;
              HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);

              // Format the data with timestamp and save to the buffer
              snprintf(writeBuf, sizeof(writeBuf), "%02d:%02d:%02d.%03d,%lu,%lu\r\n",
                       currentTime.Hours, currentTime.Minutes, currentTime.Seconds,
                       HAL_GetTick() % 1000, adcValuePA0, adcValuePA1);

              // Write data to the file
              UINT bytesWrote;
              if (f_write(&fil, writeBuf, strlen(writeBuf), &bytesWrote) != FR_OK) {
                Error_Handler(); // Handle file writing error
              }
            }

            // Force the data to be written to the disk
            if (f_sync(&fil) != FR_OK) {
              Error_Handler(); // Handle file sync error
            }

            uint32_t endTimestamp = HAL_GetTick(); // Record end timestamp
           //uint32_t elapsedMilliseconds = endTimestamp - startTimestamp;

            // Delay to maintain the desired sampling frequency

              HAL_Delay(5);
            }
        */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x38;
  sTime.Seconds = 0x0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_AUGUST;
  sDate.Date = 0x8;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 6;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
