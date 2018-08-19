
/*
 * This file contains the source code for a simple application using APDS-9960 sensor.
 Device measures proximity and uses all four leds for indicating
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "APDS9960.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif




#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define DEVICE_SCL_PIN 3
#define DEVICE_SDA_PIN 4

nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(0);

uint8_t device_address = 0; // Address used to temporarily store the current address being checked
bool device_found = false; 
static uint8_t _buff[6];  //buffer for 6 data bytes from xyz registers

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false; 


void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


#ifdef ENABLE_LOOPBACK_TEST
/* Use flow control in loopback test. */
#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    bsp_board_leds_on();
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
    uint8_t   rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}
#else
/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#endif

__STATIC_INLINE void read_APDS9960_proxy()
{ 
  uint8_t l = _buff[0];  
  if (l <= 17)              {nrf_gpio_pin_set(LED_1); nrf_gpio_pin_set(LED_2); nrf_gpio_pin_set(LED_3); nrf_gpio_pin_set(LED_4); }
  if (l >= 18 && l <= 60)   { nrf_gpio_pin_clear(LED_1); nrf_gpio_pin_set(LED_2); nrf_gpio_pin_set(LED_3); nrf_gpio_pin_set(LED_4);  }
  if (l >= 61 && l <= 140)  { nrf_gpio_pin_clear(LED_1); nrf_gpio_pin_clear(LED_2); nrf_gpio_pin_set(LED_3); nrf_gpio_pin_set(LED_4);  }
  if (l >= 141 && l <= 210) { nrf_gpio_pin_clear(LED_1); nrf_gpio_pin_clear(LED_2); nrf_gpio_pin_clear(LED_3); nrf_gpio_pin_set(LED_4);  }
  if (l >= 211)             {nrf_gpio_pin_clear(LED_1); nrf_gpio_pin_clear(LED_2); nrf_gpio_pin_clear(LED_3); nrf_gpio_pin_clear(LED_4); }
  //printf("\n\r l = %d",l);
}


/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
             if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                read_APDS9960_proxy();
            }
            device_found = true;
            m_xfer_done = true;
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            printf("No address ACK on address: %#x!\r\n", device_address);
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            printf("No data ACK on address: %#x!\r\n", device_address);
            break;
        default:
            break;        
    }   
}

void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_config = {
       .scl                = DEVICE_SCL_PIN,
       .sda                = DEVICE_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&twi_instance, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&twi_instance);
}
void APDS9960_init(void)
{
    ret_code_t err_code;
    uint8_t store, addr_temp;
    uint8_t reg[2];

    /* Configuring APDS9960 default values for ambient light and proximity registers */
    reg[0] = APDS9960_ATIME; 
    reg[1] = DEFAULT_ATIME;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

  /* Configuring APDS9960 default values for wtime */
    reg[0] = APDS9960_WTIME; 
    reg[1] = DEFAULT_WTIME;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

  /* Configuring APDS9960 default values*/
    reg[0] = APDS9960_POFFSET_UR; 
    reg[1] = DEFAULT_POFFSET_UR;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);


    reg[0] = APDS9960_POFFSET_DL; 
    reg[1] = DEFAULT_POFFSET_DL;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    reg[0] = APDS9960_CONFIG1; 
    reg[1] = DEFAULT_CONFIG1;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    /* led current 50mA, prox gain 1X */
    reg[0] = APDS9960_CONTROL; 
    reg[1] = 0b01000000;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    /* No saturation interrupt, no led boost */
    reg[0] = APDS9960_CONFIG2; 
    reg[1] = 0x01;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    /* Poffset = 0 */
    reg[0] = APDS9960_POFFSET_UR; 
    reg[1] = 0x00;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    reg[0] = APDS9960_POFFSET_DL; 
    reg[1] = 0x00;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    /* enable all photo diodes */
    reg[0] = APDS9960_CONFIG3; 
    reg[1] = 0x00;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    /*  */
    reg[0] = APDS9960_GPENTH; 
    reg[1] = DEFAULT_GPENTH;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    /*  */
    reg[0] = APDS9960_GEXTH; 
    reg[1] = DEFAULT_GEXTH;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);


    /* Gesture conf1 */
    reg[0] = APDS9960_GCONF1; 
    reg[1] = DEFAULT_GCONF1;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    /* Gesture conf2 */
    reg[0] = APDS9960_GCONF2; 
    reg[1] = 0x01;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    /* Gesture offset */
    reg[0] = APDS9960_GOFFSET_U; 
    reg[1] = DEFAULT_GOFFSET;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    reg[0] = APDS9960_GOFFSET_R; 
    reg[1] = DEFAULT_GOFFSET;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    reg[0] = APDS9960_GOFFSET_L; 
    reg[1] = DEFAULT_GOFFSET;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    reg[0] = APDS9960_GOFFSET_D; 
    reg[1] = DEFAULT_GOFFSET;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);
    
    /* Gesture pulse 32us, 10 pulses*/
    reg[0] = APDS9960_GPULSE; 
    reg[1] = DEFAULT_GPULSE;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    /* Gesture all diodes enabled */
    reg[0] = APDS9960_GCONF3; 
    reg[1] = DEFAULT_GCONF3;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

    /* Gestures disabled */
    reg[0] = APDS9960_GCONF4; 
    reg[1] = DEFAULT_GIEN;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);

 
    /* Writing to APDS9960 On and proxymity enable */
    reg[0] = APDS9960_ENABLE;
    reg[1] = 0x05;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);


    /* Configuring APDS9960 pulse width and quantity 16us, 8 pulses */
    reg[0] = APDS9960_PPULSE; 
    reg[1] = DEFAULT_PROX_PPULSE;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, reg, sizeof(reg), false);   
    while (m_xfer_done == false);


    addr_temp = APDS9960_ID; // 0xAB in register WhoAmI
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, &addr_temp, 1, false); 
    while (m_xfer_done == false);

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&twi_instance, APDS9960_I2C_ADDR, &store, 1);
    while (m_xfer_done == false);
    
    printf("\n\r WhoAmI:  %#x!\r\n", store);
    

}

/*----------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------*/
int main(void)
{

    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);


    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_pin_set(LED_1);	

    nrf_gpio_cfg_output(LED_2);
    nrf_gpio_pin_set(LED_2);

    nrf_gpio_cfg_output(LED_3);
    nrf_gpio_pin_set(LED_3);

    nrf_gpio_cfg_output(LED_4);
    nrf_gpio_pin_set(LED_4);

    twi_init();
    APDS9960_init();
    printf("\r\nAPDS-9960 proximity function start.\r\n");

    if(device_found)
    {
        
        printf("\n\r DEVICE FOUND\r\n");
        
        
        while(true)
        {
            nrf_delay_ms(100);

        /*do
        {
            __WFE();
        }while (m_xfer_done == false); */

         m_xfer_done = false;
    

    /* Read proxy data from the specified address */
    uint8_t reg_addr = APDS9960_PDATA; 
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi_instance, APDS9960_I2C_ADDR, &reg_addr, 1, false);
    while (m_xfer_done == false);

    m_xfer_done = false;
    ret_code_t err_code = nrf_drv_twi_rx(&twi_instance, APDS9960_I2C_ADDR, _buff, 1);
    while (m_xfer_done == false);

        }
    }
    else
    {
         
        printf("**************************\n\rNO APDS-9960 DEVICE FOUND!\r\n**************************\n\r");
        while(true)
        {
            nrf_gpio_pin_toggle(LED_3);
            nrf_delay_ms(500);
        }      
    }


}



