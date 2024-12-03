/*******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of the I2C 
*              resource as Master. The I2C master sends the command packets to
*              the I2C slave to control an user LED.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Delay of 1000ms between commands */
#define CMD_TO_CMD_DELAY        (1000UL)

/* Packet positions */
#define PACKET_SOP_POS          (0UL)
#define PACKET_CMD_POS          (1UL)
#define PACKET_EOP_POS          (2UL)

/* Start and end of packet markers */
#define PACKET_SOP              (0x01UL)
#define PACKET_EOP              (0x17UL)

/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR          (0x24UL)

/* I2C bus frequency */
#define I2C_FREQ                (400000UL)

/* Command valid status */
#define STATUS_CMD_DONE         (0x00UL)

/* Packet size */
#define PACKET_SIZE             (3UL)

/*******************************************************************************
* Global Variables
*******************************************************************************/

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: master_write
********************************************************************************
* Summary:
* This function writes data form I2C master to I2C slave.
*
* Parameters:
*  CySCB_Type *base - The pointer to the I2C SCB instance.
*  cy_stc_scb_i2c_context_t *context - The pointer to the context structure
*  uint16_t dev_addr - 7 bit right justified slave address.
*  const uint8_t *data - The byte to write to I2C slave.
*  uint16_t size - The size of data
*  uint32_t timeout - The time for which this function can block
*  bool send_stop - the bool for stop sending or not
*
* Return:
*  cy_rslt_t
*
*******************************************************************************/
static cy_rslt_t master_write(CySCB_Type *base,
                              cy_stc_scb_i2c_context_t *context,
                              uint16_t dev_addr,
                              const uint8_t *data,
                              uint16_t size,
                              uint32_t timeout,
                              bool send_stop)
{
    cy_en_scb_i2c_status_t status = ((*context).state == CY_SCB_I2C_IDLE)
        ? Cy_SCB_I2C_MasterSendStart(base,
                                     dev_addr,
                                     CY_SCB_I2C_WRITE_XFER,
                                     timeout,
                                     context)
        : Cy_SCB_I2C_MasterSendReStart(base,
                                       dev_addr,
                                       CY_SCB_I2C_WRITE_XFER,
                                       timeout,
                                       context);

    if (status == CY_SCB_I2C_SUCCESS)
    {
        while (size > 0)
        {
            status = Cy_SCB_I2C_MasterWriteByte(base,
                                                *data,
                                                timeout,
                                                context);
            if (status != CY_SCB_I2C_SUCCESS)
            {
                break;
            }
            --size;
            ++data;
        }
    }

    if (send_stop)
    {
        /* SCB in I2C mode is very time sensitive.               */
        /* In practice we have to request STOP after each block, */
        /* otherwise it may break the transmission               */
        Cy_SCB_I2C_MasterSendStop(base, timeout, context);
    }

    return status;
}

/*******************************************************************************
* Function Name: master_read
********************************************************************************
* Summary:
* This function reads data form I2C slave.
*
* Parameters:
*  CySCB_Type *base - The pointer to the I2C SCB instance.
*  cy_stc_scb_i2c_context_t *context - The pointer to the context structure
*  uint16_t dev_addr - 7 bit right justified slave address.
*  uint8_t *data - The byte to read from I2C slave.
*  uint16_t size - The size of data
*  uint32_t timeout - The time for which this function can block
*  bool send_stop - the bool for stop sending or not
*
* Return:
*  cy_rslt_t
*
*******************************************************************************/
static cy_rslt_t master_read(CySCB_Type *base,
                             cy_stc_scb_i2c_context_t *context,
                             uint16_t dev_addr,
                             uint8_t *data,
                             uint16_t size,
                             uint32_t timeout,
                             bool send_stop)
{
    cy_en_scb_i2c_command_t ack = CY_SCB_I2C_ACK;

    /* Start transaction, send dev_addr */
    cy_en_scb_i2c_status_t status = (*context).state == CY_SCB_I2C_IDLE
        ? Cy_SCB_I2C_MasterSendStart(base,
                                     dev_addr,
                                     CY_SCB_I2C_READ_XFER,
                                     timeout,
                                     context)
        : Cy_SCB_I2C_MasterSendReStart(base,
                                       dev_addr,
                                       CY_SCB_I2C_READ_XFER,
                                       timeout,
                                       context);

    if (status == CY_SCB_I2C_SUCCESS)
    {
        while (size > 0) {
            if (size == 1)
            {
                ack = CY_SCB_I2C_NAK;
            }
            status = Cy_SCB_I2C_MasterReadByte(base,
                                               ack,
                                               (uint8_t *)data,
                                               timeout,
                                               context);
            if (status != CY_SCB_I2C_SUCCESS)
            {
                break;
            }
            --size;
            ++data;
        }
    }

    if (send_stop)
    {
        /* SCB in I2C mode is very time sensitive.               */
        /* In practice we have to request STOP after each block, */
        /* otherwise it may break the transmission               */
        Cy_SCB_I2C_MasterSendStop(base, timeout, context);
    }
    return status;
}

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
static void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*   1. I2C Master sends command packet to the slave
*   2. I2C Master reads the response packet to generate the next command
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint8_t cmd = CYBSP_LED_STATE_ON;
    uint8_t buffer[PACKET_SIZE];
    cy_stc_scb_i2c_context_t i2c_context;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

    /* Initialize retarget-io to use the debug UART port */
    Cy_SCB_UART_Init(UART_HW, &UART_config, NULL);
    Cy_SCB_UART_Enable(UART_HW);
    result = cy_retarget_io_init(UART_HW);

    /* retarget-io init failed. Stop program execution */
    handle_error(result);

    printf("retarget-io ver1.6 testing \r\n");

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "PDL: I2C Master "
           "****************** \r\n\n");

    /* I2C Master configuration settings */
    printf(">> Configuring I2C Master..... ");

    /* Deinit I2C before initialization */
    Cy_SCB_I2C_DeInit(I2C_MASETR_HW);

    /* Initialize I2C as a master */
    result = Cy_SCB_I2C_Init(I2C_MASETR_HW, &I2C_MASETR_config, &i2c_context);

    /* I2C master configuration failed. Stop program execution */
    handle_error(result);

    /* Enable I2C */
    Cy_SCB_I2C_Enable(I2C_MASETR_HW);

    printf("Done\r\n\n");


    /* Enable interrupts */
    __enable_irq();

    for (;;)
    {
        /* Create packet to be sent to slave */
        buffer[PACKET_SOP_POS] = PACKET_SOP;
        buffer[PACKET_EOP_POS] = PACKET_EOP;
        buffer[PACKET_CMD_POS] = cmd;

        /* Send packet with command to the slave */
        if (CY_RSLT_SUCCESS == master_write(I2C_MASETR_HW,
                                            &i2c_context,
                                            I2C_SLAVE_ADDR,
                                            buffer,
                                            PACKET_SIZE,
                                            0,
                                            true))
        {
            /* Read response packet from the slave */
            if (CY_RSLT_SUCCESS == master_read(I2C_MASETR_HW,
                                               &i2c_context,
                                               I2C_SLAVE_ADDR,
                                               buffer,
                                               PACKET_SIZE,
                                               0,
                                               true))
            {
                /* Check packet structure and status */
                if ((PACKET_SOP == buffer[PACKET_SOP_POS]) &&
                   (PACKET_EOP == buffer[PACKET_EOP_POS]) &&
                   (STATUS_CMD_DONE == buffer[PACKET_CMD_POS]))
                {
                    /* Next command to be written */
                    cmd = (cmd == CYBSP_LED_STATE_ON) ?
                           CYBSP_LED_STATE_OFF : CYBSP_LED_STATE_ON;
                }
                else
                {
                    handle_error(1);
                }
            }
            /* Give delay between commands */
            Cy_SysLib_Delay(CMD_TO_CMD_DELAY);
        }
    }
}

/* [] END OF FILE */