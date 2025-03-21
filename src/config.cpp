/*  This file redefines the clock configuration function that is called by the HAL on startup.
    The function differs from the default for the NUCLEO-L432KC module to support V2.x StackSynth modules */

#include "stm32_def.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_flash.h"
#include "stm32l4xx_hal_pwr_ex.h"

#ifdef __cplusplus
extern "C" {
#endif
    void SystemClock_Config(void)
        {
        RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
        RCC_OscInitTypeDef RCC_OscInitStruct = {};
        RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

        #if 0
        /* MSI is enabled after System reset, activate PLL with MSI as source */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
        RCC_OscInitStruct.LSEState = RCC_LSE_ON;
        RCC_OscInitStruct.MSIState = RCC_MSI_ON;
        RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
        RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
        RCC_OscInitStruct.PLL.PLLM = 1;
        RCC_OscInitStruct.PLL.PLLN = 40;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
        RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
        RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
        #else
        /* MSI is enabled after System reset, activate PLL with MSI as source */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
        RCC_OscInitStruct.MSIState = RCC_MSI_ON;
        RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
        RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
        RCC_OscInitStruct.PLL.PLLM = 1;
        RCC_OscInitStruct.PLL.PLLN = 40;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
        RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
        RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
        #endif
        
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /* Initializes the CPU, AHB and APB buses clocks */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
            Error_Handler();
        }

        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
        PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
        PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
        PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
        PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
        PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
        PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
        PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
        PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }
        /* Configure the main internal regulator output voltage */
        if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
            Error_Handler();
        }
        /* Enable MSI Auto calibration */
        HAL_RCCEx_EnableMSIPLLMode();
        }

    // void SystemClock_Config(void)
    // {
    //   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    //   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    //   /** Configure the main internal regulator output voltage
    //   */
    //   if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    //   {
    //     Error_Handler();
    //   }
    
    //   /** Initializes the RCC Oscillators according to the specified parameters
    //   * in the RCC_OscInitTypeDef structure.
    //   */
    //   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    //   RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    //   RCC_OscInitStruct.MSICalibrationValue = 0;
    //   RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    //   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    //   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    //   {
    //     Error_Handler();
    //   }
    
    //   /** Initializes the CPU, AHB and APB buses clocks
    //   */
    //   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    //                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    //   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    //   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    //   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    //   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    //   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    //   {
    //     Error_Handler();
    //   }
    // }
}
