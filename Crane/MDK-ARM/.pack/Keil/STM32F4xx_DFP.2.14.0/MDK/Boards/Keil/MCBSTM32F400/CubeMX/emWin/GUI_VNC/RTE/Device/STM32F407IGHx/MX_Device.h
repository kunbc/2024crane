/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 18/07/2019 11:56:39
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated by STM32CubeMX (DO NOT EDIT!)
 ******************************************************************************/

#ifndef __MX_DEVICE_H
#define __MX_DEVICE_H

/*---------------------------- Clock Configuration ---------------------------*/

#define MX_LSI_VALUE                            32000
#define MX_LSE_VALUE                            32768
#define MX_HSI_VALUE                            16000000
#define MX_HSE_VALUE                            25000000
#define MX_EXTERNAL_CLOCK_VALUE                 12288000
#define MX_PLLCLKFreq_Value                     168000000
#define MX_SYSCLKFreq_VALUE                     168000000
#define MX_HCLKFreq_Value                       168000000
#define MX_FCLKCortexFreq_Value                 168000000
#define MX_CortexFreq_Value                     168000000
#define MX_AHBFreq_Value                        168000000
#define MX_APB1Freq_Value                       42000000
#define MX_APB2Freq_Value                       84000000
#define MX_APB1TimFreq_Value                    84000000
#define MX_APB2TimFreq_Value                    168000000
#define MX_48MHZClocksFreq_Value                42000000
#define MX_EthernetFreq_Value                   168000000
#define MX_I2SClocksFreq_Value                  96000000
#define MX_RTCFreq_Value                        32000
#define MX_WatchDogFreq_Value                   32000
#define MX_MCO1PinFreq_Value                    16000000
#define MX_MCO2PinFreq_Value                    168000000

/*-------------------------------- ETH        --------------------------------*/

#define MX_ETH                                  1

/* GPIO Configuration */

/* Pin PA1 */
#define MX_ETH_REF_CLK_GPIO_Speed               GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_REF_CLK_Pin                      PA1
#define MX_ETH_REF_CLK_GPIOx                    GPIOA
#define MX_ETH_REF_CLK_GPIO_PuPd                GPIO_NOPULL
#define MX_ETH_REF_CLK_GPIO_Pin                 GPIO_PIN_1
#define MX_ETH_REF_CLK_GPIO_AF                  GPIO_AF11_ETH
#define MX_ETH_REF_CLK_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PA7 */
#define MX_ETH_CRS_DV_GPIO_Speed                GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_CRS_DV_Pin                       PA7
#define MX_ETH_CRS_DV_GPIOx                     GPIOA
#define MX_ETH_CRS_DV_GPIO_PuPd                 GPIO_NOPULL
#define MX_ETH_CRS_DV_GPIO_Pin                  GPIO_PIN_7
#define MX_ETH_CRS_DV_GPIO_AF                   GPIO_AF11_ETH
#define MX_ETH_CRS_DV_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PC4 */
#define MX_ETH_RXD0_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_RXD0_Pin                         PC4
#define MX_ETH_RXD0_GPIOx                       GPIOC
#define MX_ETH_RXD0_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_RXD0_GPIO_Pin                    GPIO_PIN_4
#define MX_ETH_RXD0_GPIO_AF                     GPIO_AF11_ETH
#define MX_ETH_RXD0_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PC5 */
#define MX_ETH_RXD1_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_RXD1_Pin                         PC5
#define MX_ETH_RXD1_GPIOx                       GPIOC
#define MX_ETH_RXD1_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_RXD1_GPIO_Pin                    GPIO_PIN_5
#define MX_ETH_RXD1_GPIO_AF                     GPIO_AF11_ETH
#define MX_ETH_RXD1_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PG11 */
#define MX_ETH_TX_EN_GPIO_Speed                 GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_TX_EN_Pin                        PG11
#define MX_ETH_TX_EN_GPIOx                      GPIOG
#define MX_ETH_TX_EN_GPIO_PuPd                  GPIO_NOPULL
#define MX_ETH_TX_EN_GPIO_Pin                   GPIO_PIN_11
#define MX_ETH_TX_EN_GPIO_AF                    GPIO_AF11_ETH
#define MX_ETH_TX_EN_GPIO_Mode                  GPIO_MODE_AF_PP

/* Pin PA2 */
#define MX_ETH_MDIO_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_MDIO_Pin                         PA2
#define MX_ETH_MDIO_GPIOx                       GPIOA
#define MX_ETH_MDIO_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_MDIO_GPIO_Pin                    GPIO_PIN_2
#define MX_ETH_MDIO_GPIO_AF                     GPIO_AF11_ETH
#define MX_ETH_MDIO_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PG14 */
#define MX_ETH_TXD1_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_TXD1_Pin                         PG14
#define MX_ETH_TXD1_GPIOx                       GPIOG
#define MX_ETH_TXD1_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_TXD1_GPIO_Pin                    GPIO_PIN_14
#define MX_ETH_TXD1_GPIO_AF                     GPIO_AF11_ETH
#define MX_ETH_TXD1_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PG13 */
#define MX_ETH_TXD0_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_TXD0_Pin                         PG13
#define MX_ETH_TXD0_GPIOx                       GPIOG
#define MX_ETH_TXD0_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_TXD0_GPIO_Pin                    GPIO_PIN_13
#define MX_ETH_TXD0_GPIO_AF                     GPIO_AF11_ETH
#define MX_ETH_TXD0_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PC1 */
#define MX_ETH_MDC_GPIO_Speed                   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_MDC_Pin                          PC1
#define MX_ETH_MDC_GPIOx                        GPIOC
#define MX_ETH_MDC_GPIO_PuPd                    GPIO_NOPULL
#define MX_ETH_MDC_GPIO_Pin                     GPIO_PIN_1
#define MX_ETH_MDC_GPIO_AF                      GPIO_AF11_ETH
#define MX_ETH_MDC_GPIO_Mode                    GPIO_MODE_AF_PP

/* NVIC Configuration */

/* NVIC ETH_IRQn */
#define MX_ETH_IRQn_interruptPremptionPriority  0
#define MX_ETH_IRQn_PriorityGroup               NVIC_PRIORITYGROUP_4
#define MX_ETH_IRQn_Subriority                  0

/*-------------------------------- FSMC       --------------------------------*/

#define MX_FSMC                                 1

/* GPIO Configuration */

/* Pin PD8 */
#define MX_FSMC_D13_DA13_Pin                    PD8
#define MX_FSMC_D13_DA13_GPIOx                  GPIOD
#define MX_FSMC_D13_DA13_GPIO_PuPd              GPIO_NOPULL
#define MX_FSMC_D13_DA13_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D13_DA13_GPIO_Pin               GPIO_PIN_8
#define MX_FSMC_D13_DA13_GPIO_AF                GPIO_AF12_FSMC
#define MX_FSMC_D13_DA13_GPIO_Mode              GPIO_MODE_AF_PP

/* Pin PD0 */
#define MX_FSMC_D2_DA2_Pin                      PD0
#define MX_FSMC_D2_DA2_GPIOx                    GPIOD
#define MX_FSMC_D2_DA2_GPIO_PuPd                GPIO_NOPULL
#define MX_FSMC_D2_DA2_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D2_DA2_GPIO_Pin                 GPIO_PIN_0
#define MX_FSMC_D2_DA2_GPIO_AF                  GPIO_AF12_FSMC
#define MX_FSMC_D2_DA2_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PE10 */
#define MX_FSMC_D7_DA7_Pin                      PE10
#define MX_FSMC_D7_DA7_GPIOx                    GPIOE
#define MX_FSMC_D7_DA7_GPIO_PuPd                GPIO_NOPULL
#define MX_FSMC_D7_DA7_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D7_DA7_GPIO_Pin                 GPIO_PIN_10
#define MX_FSMC_D7_DA7_GPIO_AF                  GPIO_AF12_FSMC
#define MX_FSMC_D7_DA7_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PD15 */
#define MX_FSMC_D1_DA1_Pin                      PD15
#define MX_FSMC_D1_DA1_GPIOx                    GPIOD
#define MX_FSMC_D1_DA1_GPIO_PuPd                GPIO_NOPULL
#define MX_FSMC_D1_DA1_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D1_DA1_GPIO_Pin                 GPIO_PIN_15
#define MX_FSMC_D1_DA1_GPIO_AF                  GPIO_AF12_FSMC
#define MX_FSMC_D1_DA1_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PE8 */
#define MX_FSMC_D5_DA5_Pin                      PE8
#define MX_FSMC_D5_DA5_GPIOx                    GPIOE
#define MX_FSMC_D5_DA5_GPIO_PuPd                GPIO_NOPULL
#define MX_FSMC_D5_DA5_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D5_DA5_GPIO_Pin                 GPIO_PIN_8
#define MX_FSMC_D5_DA5_GPIO_AF                  GPIO_AF12_FSMC
#define MX_FSMC_D5_DA5_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PE12 */
#define MX_FSMC_D9_DA9_Pin                      PE12
#define MX_FSMC_D9_DA9_GPIOx                    GPIOE
#define MX_FSMC_D9_DA9_GPIO_PuPd                GPIO_NOPULL
#define MX_FSMC_D9_DA9_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D9_DA9_GPIO_Pin                 GPIO_PIN_12
#define MX_FSMC_D9_DA9_GPIO_AF                  GPIO_AF12_FSMC
#define MX_FSMC_D9_DA9_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PE13 */
#define MX_FSMC_D10_DA10_Pin                    PE13
#define MX_FSMC_D10_DA10_GPIOx                  GPIOE
#define MX_FSMC_D10_DA10_GPIO_PuPd              GPIO_NOPULL
#define MX_FSMC_D10_DA10_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D10_DA10_GPIO_Pin               GPIO_PIN_13
#define MX_FSMC_D10_DA10_GPIO_AF                GPIO_AF12_FSMC
#define MX_FSMC_D10_DA10_GPIO_Mode              GPIO_MODE_AF_PP

/* Pin PD4 */
#define MX_FSMC_NOE_Pin                         PD4
#define MX_FSMC_NOE_GPIOx                       GPIOD
#define MX_FSMC_NOE_GPIO_PuPd                   GPIO_NOPULL
#define MX_FSMC_NOE_GPIO_Speed_High_Default     GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_NOE_GPIO_Pin                    GPIO_PIN_4
#define MX_FSMC_NOE_GPIO_AF                     GPIO_AF12_FSMC
#define MX_FSMC_NOE_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PD1 */
#define MX_FSMC_D3_DA3_Pin                      PD1
#define MX_FSMC_D3_DA3_GPIOx                    GPIOD
#define MX_FSMC_D3_DA3_GPIO_PuPd                GPIO_NOPULL
#define MX_FSMC_D3_DA3_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D3_DA3_GPIO_Pin                 GPIO_PIN_1
#define MX_FSMC_D3_DA3_GPIO_AF                  GPIO_AF12_FSMC
#define MX_FSMC_D3_DA3_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PE9 */
#define MX_FSMC_D6_DA6_Pin                      PE9
#define MX_FSMC_D6_DA6_GPIOx                    GPIOE
#define MX_FSMC_D6_DA6_GPIO_PuPd                GPIO_NOPULL
#define MX_FSMC_D6_DA6_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D6_DA6_GPIO_Pin                 GPIO_PIN_9
#define MX_FSMC_D6_DA6_GPIO_AF                  GPIO_AF12_FSMC
#define MX_FSMC_D6_DA6_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PD10 */
#define MX_FSMC_D15_DA15_Pin                    PD10
#define MX_FSMC_D15_DA15_GPIOx                  GPIOD
#define MX_FSMC_D15_DA15_GPIO_PuPd              GPIO_NOPULL
#define MX_FSMC_D15_DA15_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D15_DA15_GPIO_Pin               GPIO_PIN_10
#define MX_FSMC_D15_DA15_GPIO_AF                GPIO_AF12_FSMC
#define MX_FSMC_D15_DA15_GPIO_Mode              GPIO_MODE_AF_PP

/* Pin PE15 */
#define MX_FSMC_D12_DA12_Pin                    PE15
#define MX_FSMC_D12_DA12_GPIOx                  GPIOE
#define MX_FSMC_D12_DA12_GPIO_PuPd              GPIO_NOPULL
#define MX_FSMC_D12_DA12_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D12_DA12_GPIO_Pin               GPIO_PIN_15
#define MX_FSMC_D12_DA12_GPIO_AF                GPIO_AF12_FSMC
#define MX_FSMC_D12_DA12_GPIO_Mode              GPIO_MODE_AF_PP

/* Pin PG12 */
#define MX_FSMC_NE4_Pin                         PG12
#define MX_FSMC_NE4_GPIOx                       GPIOG
#define MX_FSMC_NE4_GPIO_PuPd                   GPIO_NOPULL
#define MX_FSMC_NE4_GPIO_Speed_High_Default     GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_NE4_GPIO_Pin                    GPIO_PIN_12
#define MX_FSMC_NE4_GPIO_AF                     GPIO_AF12_FSMC
#define MX_FSMC_NE4_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PF0 */
#define MX_FSMC_A0_Pin                          PF0
#define MX_FSMC_A0_GPIOx                        GPIOF
#define MX_FSMC_A0_GPIO_PuPd                    GPIO_NOPULL
#define MX_FSMC_A0_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_A0_GPIO_Pin                     GPIO_PIN_0
#define MX_FSMC_A0_GPIO_AF                      GPIO_AF12_FSMC
#define MX_FSMC_A0_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PD5 */
#define MX_FSMC_NWE_Pin                         PD5
#define MX_FSMC_NWE_GPIOx                       GPIOD
#define MX_FSMC_NWE_GPIO_PuPd                   GPIO_NOPULL
#define MX_FSMC_NWE_GPIO_Speed_High_Default     GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_NWE_GPIO_Pin                    GPIO_PIN_5
#define MX_FSMC_NWE_GPIO_AF                     GPIO_AF12_FSMC
#define MX_FSMC_NWE_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PE14 */
#define MX_FSMC_D11_DA11_Pin                    PE14
#define MX_FSMC_D11_DA11_GPIOx                  GPIOE
#define MX_FSMC_D11_DA11_GPIO_PuPd              GPIO_NOPULL
#define MX_FSMC_D11_DA11_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D11_DA11_GPIO_Pin               GPIO_PIN_14
#define MX_FSMC_D11_DA11_GPIO_AF                GPIO_AF12_FSMC
#define MX_FSMC_D11_DA11_GPIO_Mode              GPIO_MODE_AF_PP

/* Pin PD14 */
#define MX_FSMC_D0_DA0_Pin                      PD14
#define MX_FSMC_D0_DA0_GPIOx                    GPIOD
#define MX_FSMC_D0_DA0_GPIO_PuPd                GPIO_NOPULL
#define MX_FSMC_D0_DA0_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D0_DA0_GPIO_Pin                 GPIO_PIN_14
#define MX_FSMC_D0_DA0_GPIO_AF                  GPIO_AF12_FSMC
#define MX_FSMC_D0_DA0_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PE11 */
#define MX_FSMC_D8_DA8_Pin                      PE11
#define MX_FSMC_D8_DA8_GPIOx                    GPIOE
#define MX_FSMC_D8_DA8_GPIO_PuPd                GPIO_NOPULL
#define MX_FSMC_D8_DA8_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D8_DA8_GPIO_Pin                 GPIO_PIN_11
#define MX_FSMC_D8_DA8_GPIO_AF                  GPIO_AF12_FSMC
#define MX_FSMC_D8_DA8_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PE7 */
#define MX_FSMC_D4_DA4_Pin                      PE7
#define MX_FSMC_D4_DA4_GPIOx                    GPIOE
#define MX_FSMC_D4_DA4_GPIO_PuPd                GPIO_NOPULL
#define MX_FSMC_D4_DA4_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D4_DA4_GPIO_Pin                 GPIO_PIN_7
#define MX_FSMC_D4_DA4_GPIO_AF                  GPIO_AF12_FSMC
#define MX_FSMC_D4_DA4_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PD9 */
#define MX_FSMC_D14_DA14_Pin                    PD9
#define MX_FSMC_D14_DA14_GPIOx                  GPIOD
#define MX_FSMC_D14_DA14_GPIO_PuPd              GPIO_NOPULL
#define MX_FSMC_D14_DA14_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FSMC_D14_DA14_GPIO_Pin               GPIO_PIN_9
#define MX_FSMC_D14_DA14_GPIO_AF                GPIO_AF12_FSMC
#define MX_FSMC_D14_DA14_GPIO_Mode              GPIO_MODE_AF_PP

/*-------------------------------- I2C1       --------------------------------*/

#define MX_I2C1                                 1

/* GPIO Configuration */

/* Pin PB8 */
#define MX_I2C1_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_I2C1_SCL_Pin                         PB8
#define MX_I2C1_SCL_GPIOx                       GPIOB
#define MX_I2C1_SCL_GPIO_PuPdOD                 GPIO_PULLUP
#define MX_I2C1_SCL_GPIO_Pin                    GPIO_PIN_8
#define MX_I2C1_SCL_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SCL_GPIO_Mode                   GPIO_MODE_AF_OD

/* Pin PB9 */
#define MX_I2C1_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_I2C1_SDA_Pin                         PB9
#define MX_I2C1_SDA_GPIOx                       GPIOB
#define MX_I2C1_SDA_GPIO_PuPdOD                 GPIO_PULLUP
#define MX_I2C1_SDA_GPIO_Pin                    GPIO_PIN_9
#define MX_I2C1_SDA_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SDA_GPIO_Mode                   GPIO_MODE_AF_OD

/* NVIC Configuration */

/* NVIC I2C1_EV_IRQn */
#define MX_I2C1_EV_IRQn_interruptPremptionPriority 0
#define MX_I2C1_EV_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_I2C1_EV_IRQn_Subriority              0

/* NVIC I2C1_ER_IRQn */
#define MX_I2C1_ER_IRQn_interruptPremptionPriority 0
#define MX_I2C1_ER_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_I2C1_ER_IRQn_Subriority              0

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                  1

/* GPIO Configuration */

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                 1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                 1

/* GPIO Configuration */

#endif  /* __MX_DEVICE_H */

