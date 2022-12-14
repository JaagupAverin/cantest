#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes

/* Standard identifier id[28:18]*/
#define WRITE_ID(id) (id << 18)
#define READ_ID(id)  (id >> 18)

#define CAN0_ID   0x000
#define CAN1_ID   0x100
#define KVASER_ID 0x200

uint8_t Can0MessageRAM[CAN0_MESSAGE_RAM_CONFIG_SIZE] __attribute__((aligned (32)));
static uint8_t Can0txFiFo[CAN0_TX_FIFO_BUFFER_SIZE];
static uint8_t Can0rxFiFo0[CAN0_RX_FIFO0_SIZE];
static uint8_t Can0rxFiFo1[CAN0_RX_FIFO1_SIZE];
static uint8_t Can0rxBuffer[CAN0_RX_BUFFER_SIZE];

uint8_t Can1MessageRAM[CAN1_MESSAGE_RAM_CONFIG_SIZE] __attribute__((aligned (32)));
static uint8_t Can1txFiFo[CAN1_TX_FIFO_BUFFER_SIZE];
static uint8_t Can1rxFiFo0[CAN1_RX_FIFO0_SIZE];
static uint8_t Can1rxFiFo1[CAN1_RX_FIFO1_SIZE];
static uint8_t Can1rxBuffer[CAN1_RX_BUFFER_SIZE];

// *****************************************************************************
// *****************************************************************************
// Section: Local functions
// *****************************************************************************
// *****************************************************************************

static void printCAN1Status() {
    uint32_t errorStatus = CAN1_REGS->CAN_PSR;
    uint32_t act = (errorStatus & CAN_PSR_ACT_Msk) >> CAN_PSR_ACT_Pos;
    printf("ACT: %lu, BRP: %lu\n", act, CAN1_REGS->CAN_TXBRP);
}

static uint8_t CANLengthToDlcGet(uint8_t length)
{
    uint8_t dlc = 0;

    if (length <= 8U)
    {
        dlc = length;
    }
    else if (length <= 12U)
    {
        dlc = 0x9U;
    }
    else if (length <= 16U)
    {
        dlc = 0xAU;
    }
    else if (length <= 20U)
    {
        dlc = 0xBU;
    }
    else if (length <= 24U)
    {
        dlc = 0xCU;
    }
    else if (length <= 32U)
    {
        dlc = 0xDU;
    }
    else if (length <= 48U)
    {
        dlc = 0xEU;
    }
    else
    {
        dlc = 0xFU;
    }
    return dlc;
}

static uint8_t CANDlcToLengthGet(uint8_t dlc)
{
    uint8_t msgLength[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};
    return msgLength[dlc];
}

/* Menu */
static void display_menu(void)
{
	printf("Menu :\r\n"
	       "  -- Select the action:\r\n"
	       "  0: Send Can0 FD standard message \r\n"
	       "  1: Send Can1 FD standard message \r\n"
	       "  2: Toggle Can0 TX mode \r\n"
	       "  m: Display menu \r\n\r\n");
}

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

void handleCan0Rx();
void handleCan1Rx();

void Can0Tx(unsigned int id, uint8_t len, uint8_t* data);
void Can1Tx(unsigned int id, uint8_t len, uint8_t* data);

bool tx_mode = false;
uint64_t tx_i = 0;
uint8_t tx_bfr[8];

int main ( void )
{
    SYS_Initialize ( NULL );
    
    /* Set Message RAM Configuration */
    CAN0_MessageRAMConfigSet(Can0MessageRAM);
    CAN1_MessageRAMConfigSet(Can1MessageRAM);

    display_menu();
     
    while ( true )
    {
        // Prevent the bus from being overrun:
        uint8_t can0_tx_available = CAN0_REGS->CAN_TXFQS & CAN_TXFQS_TFFL_Msk >> CAN_TXFQS_TFFL_Pos;
        uint8_t can1_tx_available = CAN1_REGS->CAN_TXFQS & CAN_TXFQS_TFFL_Msk >> CAN_TXFQS_TFFL_Pos;
        bool tx_ready = can0_tx_available > 4 && can1_tx_available > 4;
        if (tx_mode && tx_ready) {
            Can0Tx(CAN1_ID, 8, (uint8_t*)&tx_i);
            ++tx_i;
        }
        
        handleCan0Rx();
        handleCan1Rx();

        /* User input */
        if (SERCOM5_USART_ReceiverIsReady() == false)
        {
            continue;
        }
        uint8_t user_input = (uint8_t)SERCOM5_USART_ReadByte();
        printf("\r\n");
        switch (user_input)
        {
            case '0':
                Can0Tx(KVASER_ID, 6, (uint8_t*)"Hello");
                printCAN1Status();
                break;  
            case '1':
                Can1Tx(KVASER_ID, 6, (uint8_t*)"World");
                printCAN1Status();
                break;
            case '2':
                tx_mode = !tx_mode;
                tx_i = 0;
                printf("TX mode toggled\r\n");
                break;                 
            case 'm':
            case 'M':
                display_menu();
                break;
            default:
                printf(" Invalid Input \r\n");
                break;
        }  
    }
    return ( EXIT_FAILURE );
}

// Modifies frame content and forwards to CAN1
void Can0HandleMessages(uint8_t numberOfMessages, CAN_RX_BUFFER* rxBuf, uint8_t rxBufLen)
{
    for (uint8_t i= 0; i < numberOfMessages; i++)
    {
        //printf(" Can0Rx\r\n");
        uint8_t msgLength = CANDlcToLengthGet(rxBuf->dlc);
        uint8_t length = msgLength;
        ++rxBuf->data[length - 1];
        Can0Tx(CAN1_ID, length, rxBuf->data);
        rxBuf += rxBufLen;
    }
}

// Modifies frame and forwards frame back to Kvaser
void Can1HandleMessages(uint8_t numberOfMessages, CAN_RX_BUFFER* rxBuf, uint8_t rxBufLen)
{
    for (uint8_t i= 0; i < numberOfMessages; i++)
    {
        //printf(" Can1Rx\r\n");
        uint8_t msgLength = CANDlcToLengthGet(rxBuf->dlc);
        uint8_t length = msgLength;
        ++rxBuf->data[length - 1];
        Can1Tx(KVASER_ID, length, rxBuf->data);
        rxBuf += rxBufLen;
    }
}

void handleCan0Rx() {
    uint8_t bufferNumber = 0;
    uint8_t numberOfMessage = 0;
    CAN_ERROR status = 0;
    
    if (CAN0_InterruptGet(CAN_INTERRUPT_DRX_MASK))
    {    
        CAN0_InterruptClear(CAN_INTERRUPT_DRX_MASK);

        /* Check CAN Status */
        status = CAN0_ErrorGet();

        if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
        {
            if (CAN0_RxBufferNumberGet(&bufferNumber))
            {
                memset(Can0rxBuffer, 0x00, CAN0_RX_BUFFER_ELEMENT_SIZE);
                if (CAN0_MessageReceive(bufferNumber, (CAN_RX_BUFFER *)Can0rxBuffer) == true)
                {
                    Can0HandleMessages(1, (CAN_RX_BUFFER *)Can0rxBuffer, CAN0_RX_BUFFER_ELEMENT_SIZE);
                }
                else
                {
                    printf(" Error in received message\r\n");
                }
            }
        }
        else
        {
            printf(" Error in received message\r\n");
        }
    }

    /* Rx FIFO0 */
    if (CAN0_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
    {    
        CAN0_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

        /* Check CAN Status */
        status = CAN0_ErrorGet();

        if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
        {
            numberOfMessage = CAN0_RxFifoFillLevelGet(CAN_RX_FIFO_0);
            if (numberOfMessage != 0)
            {
                memset(Can0rxFiFo0, 0x00, (numberOfMessage * CAN0_RX_FIFO0_ELEMENT_SIZE));
                if (CAN0_MessageReceiveFifo(CAN_RX_FIFO_0, numberOfMessage, (CAN_RX_BUFFER *)Can0rxFiFo0) == true)
                {
                    Can0HandleMessages(numberOfMessage, (CAN_RX_BUFFER *)Can0rxFiFo0, CAN0_RX_FIFO0_ELEMENT_SIZE);
                }
                else
                {
                    printf(" Error in received message\r\n");
                }
            }
        }
        else
        {
            printf(" Error in received message\r\n");
        }
    }

    /* Rx FIFO1 */
    if (CAN0_InterruptGet(CAN_INTERRUPT_RF1N_MASK))
    {    
        CAN0_InterruptClear(CAN_INTERRUPT_RF1N_MASK);

        /* Check CAN Status */
        status = CAN0_ErrorGet();

        if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
        {
            numberOfMessage = CAN0_RxFifoFillLevelGet(CAN_RX_FIFO_1);
            if (numberOfMessage != 0)
            {
                memset(Can0rxFiFo1, 0x00, (numberOfMessage * CAN0_RX_FIFO1_ELEMENT_SIZE));
                if (CAN0_MessageReceiveFifo(CAN_RX_FIFO_1, numberOfMessage, (CAN_RX_BUFFER *)Can0rxFiFo1) == true)
                {
                    Can0HandleMessages(numberOfMessage, (CAN_RX_BUFFER *)Can0rxFiFo1, CAN0_RX_FIFO1_ELEMENT_SIZE);
                }
                else
                {
                    printf(" Error in received message\r\n");
                }
            }
        }
        else
        {
            printf(" Error in received message\r\n");
        }
    }
}

void handleCan1Rx() {
    uint8_t bufferNumber = 0;
    uint8_t numberOfMessage = 0;
    CAN_ERROR status = 0;

    if (CAN1_InterruptGet(CAN_INTERRUPT_DRX_MASK))
    {    
        CAN1_InterruptClear(CAN_INTERRUPT_DRX_MASK);

        /* Check CAN Status */
        status = CAN1_ErrorGet();

        if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
        {
            if (CAN1_RxBufferNumberGet(&bufferNumber))
            {
                memset(Can1rxBuffer, 0x00, CAN1_RX_BUFFER_ELEMENT_SIZE);
                if (CAN1_MessageReceive(bufferNumber, (CAN_RX_BUFFER *)Can1rxBuffer) == true)
                {
                    Can1HandleMessages(1, (CAN_RX_BUFFER *)Can1rxBuffer, CAN1_RX_BUFFER_ELEMENT_SIZE);
                }
                else
                {
                    printf(" Error in received message\r\n");
                }
            }
        }
        else
        {
            printf(" Error in received message\r\n");
        }
    }

    /* Rx FIFO0 */
    if (CAN1_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
    {    
        CAN1_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

        /* Check CAN Status */
        status = CAN1_ErrorGet();

        if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
        {
            numberOfMessage = CAN1_RxFifoFillLevelGet(CAN_RX_FIFO_0);
            if (numberOfMessage != 0)
            {
                memset(Can1rxFiFo0, 0x00, (numberOfMessage * CAN1_RX_FIFO0_ELEMENT_SIZE));
                if (CAN1_MessageReceiveFifo(CAN_RX_FIFO_0, numberOfMessage, (CAN_RX_BUFFER *)Can1rxFiFo0) == true)
                {
                    Can1HandleMessages(numberOfMessage, (CAN_RX_BUFFER *)Can1rxFiFo0, CAN1_RX_FIFO0_ELEMENT_SIZE);
                }
                else
                {
                    printf(" Error in received message\r\n");
                }
            }
        }
        else
        {
            printf(" Error in received message\r\n");
        }
    }

    /* Rx FIFO1 */
    if (CAN1_InterruptGet(CAN_INTERRUPT_RF1N_MASK))
    {    
        CAN1_InterruptClear(CAN_INTERRUPT_RF1N_MASK);

        /* Check CAN Status */
        status = CAN1_ErrorGet();

        if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
        {
            numberOfMessage = CAN1_RxFifoFillLevelGet(CAN_RX_FIFO_1);
            if (numberOfMessage != 0)
            {
                memset(Can1rxFiFo1, 0x00, (numberOfMessage * CAN1_RX_FIFO1_ELEMENT_SIZE));
                if (CAN1_MessageReceiveFifo(CAN_RX_FIFO_1, numberOfMessage, (CAN_RX_BUFFER *)Can1rxFiFo1) == true)
                {
                    Can1HandleMessages(numberOfMessage, (CAN_RX_BUFFER *)Can1rxFiFo1, CAN1_RX_FIFO1_ELEMENT_SIZE);
                }
                else
                {
                    printf(" Error in received message\r\n");
                }
            }
        }
        else
        {
            printf(" Error in received message\r\n");
        }
    }
}

void Can0Tx(unsigned int id, uint8_t len, uint8_t* data) {
    CAN_TX_BUFFER *txBuffer = NULL;

    memset(Can0txFiFo, 0x00, CAN0_TX_FIFO_BUFFER_ELEMENT_SIZE);
    txBuffer = (CAN_TX_BUFFER *)Can0txFiFo;
    txBuffer->id = WRITE_ID(id);
    txBuffer->dlc = CANLengthToDlcGet(len);
    txBuffer->fdf = 1;
    txBuffer->brs = 1;
    for (int i = 0; i < len; i++){
        txBuffer->data[i] = data[i];
    }                
    if (CAN0_MessageTransmitFifo(1, txBuffer) == false)
    {
        printf(" Can0Tx Failed \r\n");
    }   
}

void Can1Tx(unsigned int id, uint8_t len, uint8_t* data) {
    CAN_TX_BUFFER *txBuffer = NULL;

    memset(Can1txFiFo, 0x00, CAN1_TX_FIFO_BUFFER_ELEMENT_SIZE);
    txBuffer = (CAN_TX_BUFFER *)Can1txFiFo;
    txBuffer->id = WRITE_ID(id);
    txBuffer->dlc = CANLengthToDlcGet(len);
    txBuffer->fdf = 1;
    txBuffer->brs = 1;
    for (int i = 0; i < len; i++){
        txBuffer->data[i] = data[i];
    }                

    if (CAN1_MessageTransmitFifo(1, txBuffer) == false)
    {
        printf(" Can1Tx Failed \r\n");
    }         
}

/*******************************************************************************
 End of File
*/
