/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v14.0
processor: MKL27Z64xxx4
package_id: MKL27Z64VLH4
mcu_data: ksdk2_0
processor_version: 13.0.1
board: FRDM-KL27Z
external_user_signals: {}
pin_labels:
- {pin_num: '1', pin_signal: PTE0/CLKOUT32K/SPI1_MISO/LPUART1_TX/RTC_CLKOUT/CMP0_OUT/I2C1_SDA, label: 'J3[1]/CLKOUT32K', identifier: CLKOUT32K}
- {pin_num: '56', pin_signal: PTC11/I2C1_SDA, label: 'J2[3]/I2C1_SDA', identifier: I2C1_SDA}
- {pin_num: '21', pin_signal: PTE25/TPM0_CH1/I2C0_SDA, label: 'J1[8]/D3-TPM0_CH1', identifier: S3_PR}
- {pin_num: '23', pin_signal: PTA1/LPUART0_RX/TPM2_CH0, label: 'J1[2]/J25[1]/D0-UART0_RX', identifier: S1_G2}
- {pin_num: '24', pin_signal: PTA2/LPUART0_TX/TPM2_CH1, label: 'J1[4]/J26[1]/D1-UART0_TX', identifier: S1_R}
- {pin_num: '28', pin_signal: PTA12/TPM1_CH0, label: 'J1[6]/D2-TPM1_CH0', identifier: S3_PG}
- {pin_num: '29', pin_signal: PTA13/TPM1_CH1, label: 'J1[10]/D4-TPM1_CH1/D4-LED_BLUE', identifier: LED_BLUE}
- {pin_num: '20', pin_signal: PTE24/TPM0_CH0/I2C0_SCL, label: 'J1[12]/D5-TPM0_CH0', identifier: S3_PS}
- {pin_num: '54', pin_signal: CMP0_IN3/PTC9/I2C0_SDA/TPM0_CH5, label: 'J1[14]/D6-TPM0_CH5/CMP0_IN3', identifier: S2_PR}
- {pin_num: '53', pin_signal: CMP0_IN2/PTC8/I2C0_SCL/TPM0_CH4, label: 'J1[16]/D7-TPM0_CH4/CMP0_IN2', identifier: S2_PG}
- {pin_num: '19', pin_signal: PTE31/TPM0_CH4, label: 'J2[2]/D8-TPM0_CH4', identifier: S3_G2}
- {pin_num: '27', pin_signal: PTA5/USB_CLKIN/TPM0_CH2, label: 'J2[4]/D9-TPM0_CH2', identifier: S3_R}
- {pin_num: '49', pin_signal: PTC4/LLWU_P8/SPI0_PCS0/LPUART1_TX/TPM0_CH3/SPI1_PCS0, label: 'J2[6]/D10-SPI0_CS0', identifier: S3_O}
- {pin_num: '51', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_MOSI/EXTRG_IN/SPI0_MISO, label: 'J2[8]/D11-SPI0_MOSI', identifier: S3_G1}
- {pin_num: '52', pin_signal: CMP0_IN1/PTC7/SPI0_MISO/USB_SOF_OUT/SPI0_MOSI, label: 'J2[10]/D12-SPI0_MISO', identifier: S2_R}
- {pin_num: '50', pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/CMP0_OUT, label: 'J2[12]/D13-SPI0_SCK', identifier: S2_O}
- {pin_num: '63', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/SPI1_MOSI/LPUART0_RX/I2C1_SDA/SPI1_MISO/FXIO0_D6, label: 'J2[18]/J24[1]/D14-I2C1_SDA', identifier: S2_G1}
- {pin_num: '64', pin_signal: PTD7/SPI1_MISO/LPUART0_TX/I2C1_SCL/SPI1_MOSI/FXIO0_D7, label: 'J2[20]/J23[1]/D15-I2C1_SCL', identifier: S2_G2}
- {pin_num: '8', pin_signal: ADC0_DP1/ADC0_SE1/PTE16/SPI0_PCS0/UART2_TX/TPM_CLKIN0/FXIO0_D0, label: 'J4[2]/A0-ADC0_SE1', identifier: S2_PS}
- {pin_num: '43', pin_signal: ADC0_SE14/PTC0/EXTRG_IN/USB_SOF_OUT/CMP0_OUT, label: 'J4[4]/A1-ADC0_SE14', identifier: S1_PG}
- {pin_num: '9', pin_signal: ADC0_DP0/ADC0_SE0/PTE20/TPM1_CH0/LPUART0_TX/FXIO0_D4, label: 'J4[6]/A2-ADC0_SE0', identifier: S1_PR}
- {pin_num: '10', pin_signal: ADC0_DM0/ADC0_SE4a/PTE21/TPM1_CH1/LPUART0_RX/FXIO0_D5, label: 'J4[8]/A3-ADC0_SE4A', identifier: S1_PS}
- {pin_num: '36', pin_signal: ADC0_SE9/PTB1/I2C0_SDA/TPM1_CH1/SPI1_MISO/SPI1_MOSI, label: 'J4[10]/A4-I2C0_SDA/ADC0_SE9', identifier: S1_G1}
- {pin_num: '35', pin_signal: ADC0_SE8/PTB0/LLWU_P5/I2C0_SCL/TPM1_CH0/SPI1_MOSI/SPI1_MISO, label: 'J4[12]/A5-I2C0_SCL/ADC0_SE8', identifier: S1_O}
- {pin_num: '41', pin_signal: PTB18/TPM2_CH0, label: 'J2[11]/D11[1]/LED_RED', identifier: LED_RED}
- {pin_num: '25', pin_signal: PTA3/I2C1_SCL/TPM0_CH0/SWD_DIO, label: 'J11[2]/SWD_DIO'}
- {pin_num: '15', pin_signal: VREFL, label: GND}
- {pin_num: '14', pin_signal: VREF_OUT, label: VREFH_C15}
- {pin_num: '2', pin_signal: PTE1/SPI1_MOSI/LPUART1_RX/SPI1_MISO/I2C1_SCL, label: 'J3[3]'}
- {pin_num: '42', pin_signal: PTB19/TPM2_CH1, label: 'J2[13]/D11[4]/LED_GREEN', identifier: LED_GREEN}
- {pin_num: '16', pin_signal: VSSA, label: GND}
- {pin_num: '3', pin_signal: VDD3, label: P3V3_KL27Z}
- {pin_num: '4', pin_signal: VSS4, label: GND}
- {pin_num: '30', pin_signal: VDD36, label: P3V3_KL27Z}
- {pin_num: '48', pin_signal: VDD54, label: P3V3_KL27Z}
- {pin_num: '13', pin_signal: VDDA, label: P3V3_KL27Z}
- {pin_num: '7', pin_signal: USB_VDD, label: P3V3_KL27Z, identifier: USB_VDD}
- {pin_num: '57', pin_signal: PTD0/SPI0_PCS0/TPM0_CH0/FXIO0_D0, label: 'J1[1]'}
- {pin_num: '58', pin_signal: ADC0_SE5b/PTD1/SPI0_SCK/TPM0_CH1/FXIO0_D1, label: 'J1[3]'}
- {pin_num: '59', pin_signal: PTD2/SPI0_MOSI/UART2_RX/TPM0_CH2/SPI0_MISO/FXIO0_D2, label: 'J1[5]'}
- {pin_num: '60', pin_signal: PTD3/SPI0_MISO/UART2_TX/TPM0_CH3/SPI0_MOSI/FXIO0_D3, label: 'J1[7]'}
- {pin_num: '61', pin_signal: PTD4/LLWU_P14/SPI1_PCS0/UART2_RX/TPM0_CH4/FXIO0_D4, label: 'J1[9]/SDA_LED', identifier: SDA_LED}
- {pin_num: '62', pin_signal: ADC0_SE6b/PTD5/SPI1_SCK/UART2_TX/TPM0_CH5/FXIO0_D5, label: 'J1[11]/J3[2]/SDA_PTD5'}
- {pin_num: '5', pin_signal: USB0_DP, label: 'J10[3]', identifier: USB_DP}
- {pin_num: '6', pin_signal: USB0_DM, label: 'J10[2]', identifier: USB_DM}
- {pin_num: '22', pin_signal: PTA0/TPM0_CH5/SWD_CLK, label: 'J11[4]/KL27_SWD_CLK', identifier: TPM0_CH5}
- {pin_num: '26', pin_signal: PTA4/I2C1_SDA/TPM0_CH1/NMI_b, label: 'J2[5]/SW1', identifier: SW1}
- {pin_num: '31', pin_signal: VSS37, label: GND}
- {pin_num: '47', pin_signal: VSS53, label: GND}
- {pin_num: '55', pin_signal: PTC10/I2C1_SCL, label: 'J2[1]/I2C1_SCL', identifier: I2C1_SCL}
- {pin_num: '46', pin_signal: PTC3/LLWU_P7/SPI1_SCK/LPUART1_RX/TPM0_CH2/CLKOUT, label: 'J2[15]/U10[11]/J28[1]/INT1_ACCEL', identifier: INT1_ACCEL}
- {pin_num: '45', pin_signal: ADC0_SE11/PTC2/I2C1_SDA/TPM0_CH1, label: 'J3[15]/U10[9]/J27[1]/UART1_TX/INT2_ACCEL', identifier: INT2_ACCEL;INT1_MAG}
- {pin_num: '44', pin_signal: ADC0_SE15/PTC1/LLWU_P6/RTC_CLKIN/I2C1_SCL/TPM0_CH0, label: 'J3[13]/SW3', identifier: SW3}
- {pin_num: '40', pin_signal: PTB17/SPI1_MISO/LPUART0_TX/TPM_CLKIN1/SPI1_MOSI, label: 'J2[9]'}
- {pin_num: '39', pin_signal: PTB16/SPI1_MOSI/LPUART0_RX/TPM_CLKIN0/SPI1_MISO, label: 'J2[7]'}
- {pin_num: '38', pin_signal: ADC0_SE13/PTB3/I2C0_SDA/TPM2_CH1, label: 'J1[13]'}
- {pin_num: '37', pin_signal: ADC0_SE12/PTB2/I2C0_SCL/TPM2_CH0, label: 'J1[15]'}
- {pin_num: '34', pin_signal: PTA20/RESET_b, label: 'J3[6]/J11[10]/RST_K20D50_B', identifier: RESET}
- {pin_num: '33', pin_signal: XTAL0/PTA19/LPUART1_TX/TPM_CLKIN1/LPTMR0_ALT1, label: XTAL_32KHZ, identifier: XTAL0}
- {pin_num: '32', pin_signal: EXTAL0/PTA18/LPUART1_RX/TPM_CLKIN0, label: EXTAL_32KHZ, identifier: EXTAL0}
- {pin_num: '18', pin_signal: ADC0_SE23/CMP0_IN4/PTE30/TPM0_CH3/TPM_CLKIN1/LPUART1_TX/LPTMR0_ALT1, label: TOUCH_B, identifier: TOUCH_B}
- {pin_num: '17', pin_signal: CMP0_IN5/ADC0_SE4b/PTE29/TPM0_CH2/TPM_CLKIN0, label: TOUCH_A, identifier: TOUCH_A}
- {pin_num: '12', pin_signal: ADC0_DM3/ADC0_SE7a/PTE23/TPM2_CH1/UART2_RX/FXIO0_D7, label: THER_B, identifier: THER_B}
- {pin_num: '11', pin_signal: ADC0_DP3/ADC0_SE3/PTE22/TPM2_CH0/UART2_TX/FXIO0_D6, label: THER_A, identifier: THER_A}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
    BOARD_InitDEBUG_UARTPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '20', peripheral: GPIOE, signal: 'GPIO, 24', pin_signal: PTE24/TPM0_CH0/I2C0_SCL}
  - {pin_num: '19', peripheral: GPIOE, signal: 'GPIO, 31', pin_signal: PTE31/TPM0_CH4}
  - {pin_num: '28', peripheral: GPIOA, signal: 'GPIO, 12', pin_signal: PTA12/TPM1_CH0}
  - {pin_num: '8', peripheral: GPIOE, signal: 'GPIO, 16', pin_signal: ADC0_DP1/ADC0_SE1/PTE16/SPI0_PCS0/UART2_TX/TPM_CLKIN0/FXIO0_D0}
  - {pin_num: '9', peripheral: GPIOE, signal: 'GPIO, 20', pin_signal: ADC0_DP0/ADC0_SE0/PTE20/TPM1_CH0/LPUART0_TX/FXIO0_D4}
  - {pin_num: '10', peripheral: GPIOE, signal: 'GPIO, 21', pin_signal: ADC0_DM0/ADC0_SE4a/PTE21/TPM1_CH1/LPUART0_RX/FXIO0_D5}
  - {pin_num: '36', peripheral: GPIOB, signal: 'GPIO, 1', pin_signal: ADC0_SE9/PTB1/I2C0_SDA/TPM1_CH1/SPI1_MISO/SPI1_MOSI}
  - {pin_num: '35', peripheral: GPIOB, signal: 'GPIO, 0', pin_signal: ADC0_SE8/PTB0/LLWU_P5/I2C0_SCL/TPM1_CH0/SPI1_MOSI/SPI1_MISO}
  - {pin_num: '64', peripheral: GPIOD, signal: 'GPIO, 7', pin_signal: PTD7/SPI1_MISO/LPUART0_TX/I2C1_SCL/SPI1_MOSI/FXIO0_D7}
  - {pin_num: '63', peripheral: GPIOD, signal: 'GPIO, 6', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/SPI1_MOSI/LPUART0_RX/I2C1_SDA/SPI1_MISO/FXIO0_D6}
  - {pin_num: '50', peripheral: GPIOC, signal: 'GPIO, 5', pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/CMP0_OUT}
  - {pin_num: '52', peripheral: GPIOC, signal: 'GPIO, 7', pin_signal: CMP0_IN1/PTC7/SPI0_MISO/USB_SOF_OUT/SPI0_MOSI}
  - {pin_num: '51', peripheral: GPIOC, signal: 'GPIO, 6', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_MOSI/EXTRG_IN/SPI0_MISO}
  - {pin_num: '49', peripheral: GPIOC, signal: 'GPIO, 4', pin_signal: PTC4/LLWU_P8/SPI0_PCS0/LPUART1_TX/TPM0_CH3/SPI1_PCS0}
  - {pin_num: '53', peripheral: GPIOC, signal: 'GPIO, 8', pin_signal: CMP0_IN2/PTC8/I2C0_SCL/TPM0_CH4}
  - {pin_num: '54', peripheral: GPIOC, signal: 'GPIO, 9', pin_signal: CMP0_IN3/PTC9/I2C0_SDA/TPM0_CH5}
  - {pin_num: '23', peripheral: GPIOA, signal: 'GPIO, 1', pin_signal: PTA1/LPUART0_RX/TPM2_CH0}
  - {pin_num: '24', peripheral: GPIOA, signal: 'GPIO, 2', pin_signal: PTA2/LPUART0_TX/TPM2_CH1}
  - {pin_num: '21', peripheral: GPIOE, signal: 'GPIO, 25', pin_signal: PTE25/TPM0_CH1/I2C0_SDA}
  - {pin_num: '43', peripheral: GPIOC, signal: 'GPIO, 0', pin_signal: ADC0_SE14/PTC0/EXTRG_IN/USB_SOF_OUT/CMP0_OUT}
  - {pin_num: '27', peripheral: GPIOA, signal: 'GPIO, 5', pin_signal: PTA5/USB_CLKIN/TPM0_CH2}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);
    /* Port D Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortD);
    /* Port E Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortE);

    /* PORTA1 (pin 23) is configured as PTA1 */
    PORT_SetPinMux(BOARD_S1_G2_PORT, BOARD_S1_G2_PIN, kPORT_MuxAsGpio);

    /* PORTA12 (pin 28) is configured as PTA12 */
    PORT_SetPinMux(BOARD_S3_PG_PORT, BOARD_S3_PG_PIN, kPORT_MuxAsGpio);

    /* PORTA2 (pin 24) is configured as PTA2 */
    PORT_SetPinMux(BOARD_S1_R_PORT, BOARD_S1_R_PIN, kPORT_MuxAsGpio);

    /* PORTA5 (pin 27) is configured as PTA5 */
    PORT_SetPinMux(BOARD_S3_R_PORT, BOARD_S3_R_PIN, kPORT_MuxAsGpio);

    /* PORTB0 (pin 35) is configured as PTB0 */
    PORT_SetPinMux(BOARD_S1_O_PORT, BOARD_S1_O_PIN, kPORT_MuxAsGpio);

    /* PORTB1 (pin 36) is configured as PTB1 */
    PORT_SetPinMux(BOARD_S1_G1_PORT, BOARD_S1_G1_PIN, kPORT_MuxAsGpio);

    /* PORTC0 (pin 43) is configured as PTC0 */
    PORT_SetPinMux(BOARD_S1_PG_PORT, BOARD_S1_PG_PIN, kPORT_MuxAsGpio);

    /* PORTC4 (pin 49) is configured as PTC4 */
    PORT_SetPinMux(BOARD_S3_O_PORT, BOARD_S3_O_PIN, kPORT_MuxAsGpio);

    /* PORTC5 (pin 50) is configured as PTC5 */
    PORT_SetPinMux(BOARD_S2_O_PORT, BOARD_S2_O_PIN, kPORT_MuxAsGpio);

    /* PORTC6 (pin 51) is configured as PTC6 */
    PORT_SetPinMux(BOARD_S3_G1_PORT, BOARD_S3_G1_PIN, kPORT_MuxAsGpio);

    /* PORTC7 (pin 52) is configured as PTC7 */
    PORT_SetPinMux(BOARD_S2_R_PORT, BOARD_S2_R_PIN, kPORT_MuxAsGpio);

    /* PORTC8 (pin 53) is configured as PTC8 */
    PORT_SetPinMux(BOARD_S2_PG_PORT, BOARD_S2_PG_PIN, kPORT_MuxAsGpio);

    /* PORTC9 (pin 54) is configured as PTC9 */
    PORT_SetPinMux(BOARD_S2_PR_PORT, BOARD_S2_PR_PIN, kPORT_MuxAsGpio);

    /* PORTD6 (pin 63) is configured as PTD6 */
    PORT_SetPinMux(BOARD_S2_G1_PORT, BOARD_S2_G1_PIN, kPORT_MuxAsGpio);

    /* PORTD7 (pin 64) is configured as PTD7 */
    PORT_SetPinMux(BOARD_S2_G2_PORT, BOARD_S2_G2_PIN, kPORT_MuxAsGpio);

    /* PORTE16 (pin 8) is configured as PTE16 */
    PORT_SetPinMux(BOARD_S2_PS_PORT, BOARD_S2_PS_PIN, kPORT_MuxAsGpio);

    /* PORTE20 (pin 9) is configured as PTE20 */
    PORT_SetPinMux(BOARD_S1_PR_PORT, BOARD_S1_PR_PIN, kPORT_MuxAsGpio);

    /* PORTE21 (pin 10) is configured as PTE21 */
    PORT_SetPinMux(BOARD_S1_PS_PORT, BOARD_S1_PS_PIN, kPORT_MuxAsGpio);

    /* PORTE24 (pin 20) is configured as PTE24 */
    PORT_SetPinMux(BOARD_S3_PS_PORT, BOARD_S3_PS_PIN, kPORT_MuxAsGpio);

    /* PORTE25 (pin 21) is configured as PTE25 */
    PORT_SetPinMux(BOARD_S3_PR_PORT, BOARD_S3_PR_PIN, kPORT_MuxAsGpio);

    /* PORTE31 (pin 19) is configured as PTE31 */
    PORT_SetPinMux(BOARD_S3_G2_PORT, BOARD_S3_G2_PIN, kPORT_MuxAsGpio);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitLEDsPins:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitLEDsPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitLEDsPins(void)
{
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitButtonsPins:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitButtonsPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitButtonsPins(void)
{
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitTSIPins:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitTSIPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitTSIPins(void)
{
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitUSBPins:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {peripheral: LPUART0, signal: RX, pin_signal: CMP0_output}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitUSBPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitUSBPins(void)
{

    SIM->SOPT5 = ((SIM->SOPT5 &
                   /* Mask bits to zero which are setting */
                   (~(SIM_SOPT5_LPUART0RXSRC_MASK)))

                  /* LPUART0 Receive Data Source Select: CMP0 output. */
                  | SIM_SOPT5_LPUART0RXSRC(SOPT5_LPUART0RXSRC_CMP0));
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitACCEL_I2CPins:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitACCEL_I2CPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitACCEL_I2CPins(void)
{
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitMAGNET_I2CPins:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitMAGNET_I2CPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitMAGNET_I2CPins(void)
{
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitDEBUG_UARTPins:
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitDEBUG_UARTPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitDEBUG_UARTPins(void)
{
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitTHERPins:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitTHERPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitTHERPins(void)
{
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitOSCPins:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '32', peripheral: OSC0, signal: EXTAL0, pin_signal: EXTAL0/PTA18/LPUART1_RX/TPM_CLKIN0, slew_rate: no_init, pull_select: no_init, pull_enable: no_init}
  - {pin_num: '33', peripheral: OSC0, signal: XTAL0, pin_signal: XTAL0/PTA19/LPUART1_TX/TPM_CLKIN1/LPTMR0_ALT1, slew_rate: no_init, pull_select: no_init, pull_enable: no_init}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitOSCPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitOSCPins(void)
{
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);

    /* PORTA18 (pin 32) is configured as EXTAL0 */
    PORT_SetPinMux(BOARD_EXTAL0_PORT, BOARD_EXTAL0_PIN, kPORT_PinDisabledOrAnalog);

    /* PORTA19 (pin 33) is configured as XTAL0 */
    PORT_SetPinMux(BOARD_XTAL0_PORT, BOARD_XTAL0_PIN, kPORT_PinDisabledOrAnalog);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/