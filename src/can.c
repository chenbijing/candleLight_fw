/*

   The MIT License (MIT)

   Copyright (c) 2016 Hubert Denkmair

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.

 */

#include "can.h"
#include "config.h"
#include "device.h"
#include "gpio.h"
#include "gs_usb.h"
#include "hal_include.h"

#ifndef STM32G0
	// The STM32F0 only has one CAN interface, define it as CAN1 as
	// well, so it doesn't need to be handled separately.
	#if !defined(CAN1) && defined(CAN)
	#define CAN1 CAN
	#endif
#endif

#if defined(STM32G0)
FDCAN_HandleTypeDef hfdcan2;

#define CAN_ESR_EWGF_Pos       (0U)                                            
#define CAN_ESR_EWGF_Msk       (0x1UL << CAN_ESR_EWGF_Pos)                      /*!< 0x00000001 */
#define CAN_ESR_EWGF           CAN_ESR_EWGF_Msk                                /*!<Error Warning Flag */
#define CAN_ESR_EPVF_Pos       (1U)                                            
#define CAN_ESR_EPVF_Msk       (0x1UL << CAN_ESR_EPVF_Pos)                      /*!< 0x00000002 */
#define CAN_ESR_EPVF           CAN_ESR_EPVF_Msk                                /*!<Error Passive Flag */
#define CAN_ESR_BOFF_Pos       (2U)                                            
#define CAN_ESR_BOFF_Msk       (0x1UL << CAN_ESR_BOFF_Pos)                      /*!< 0x00000004 */
#define CAN_ESR_BOFF           CAN_ESR_BOFF_Msk                                /*!<Bus-Off Flag */

#define CAN_ESR_LEC_Pos        (4U)                                            
#define CAN_ESR_LEC_Msk        (0x7UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000070 */
#define CAN_ESR_LEC            CAN_ESR_LEC_Msk                                 /*!<LEC[2:0] bits (Last Error Code) */
#define CAN_ESR_LEC_0          (0x1UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000010 */
#define CAN_ESR_LEC_1          (0x2UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000020 */
#define CAN_ESR_LEC_2          (0x4UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000040 */

#define CAN_ESR_TEC_Pos        (16U)                                           
#define CAN_ESR_TEC_Msk        (0xFFUL << CAN_ESR_TEC_Pos)                      /*!< 0x00FF0000 */
#define CAN_ESR_TEC            CAN_ESR_TEC_Msk                                 /*!<Least significant byte of the 9-bit Transmit Error Counter */
#define CAN_ESR_REC_Pos        (24U)                                           
#define CAN_ESR_REC_Msk        (0xFFUL << CAN_ESR_REC_Pos)                      /*!< 0xFF000000 */
#define CAN_ESR_REC            CAN_ESR_REC_Msk                                 /*!<Receive Error Counter */

typedef struct
{
  __IO uint32_t id_section;
  __IO uint32_t dlc_section;
  __IO uint32_t data[64 / 4];
}FDCAN_FIFO_TypeDef;

#define FDCAN_XTD (1<<30)
#define FDCAN_RTR (1<<29)

typedef struct
{
  __IO uint32_t FLS[28]; // Filter list standard
  __IO uint32_t FLE[16]; // Filter list extended
  FDCAN_FIFO_TypeDef RXF0[3];
  FDCAN_FIFO_TypeDef RXF1[3];
  __IO uint32_t TEF[6]; // Tx event FIFO
  FDCAN_FIFO_TypeDef TXFIFO[3];
}FDCAN_MSG_RAM_TypeDef;

typedef struct
{
  FDCAN_MSG_RAM_TypeDef fdcan1;
  FDCAN_MSG_RAM_TypeDef fdcan2;
}FDCAN_RAM_TypeDef;

FDCAN_RAM_TypeDef *fdcan_ram = (FDCAN_RAM_TypeDef *)(SRAMCAN_BASE);

#define SOC_CAN FDCAN2
#define MSG_RAM fdcan_ram->fdcan2

#define CANMSG_ID_RTR (1<<30)
#define CANMSG_ID_EFF (1<<31)

#endif

#ifndef STM32G0
// Completely reset the CAN pheriperal, including bus-state and error counters
static void rcc_reset(CAN_TypeDef *instance)
{
#ifdef CAN1
	if (instance == CAN1) {
		__HAL_RCC_CAN1_FORCE_RESET();
		__HAL_RCC_CAN1_RELEASE_RESET();
	}
#endif

#ifdef CAN2
	if (instance == CAN2) {
		__HAL_RCC_CAN2_FORCE_RESET();
		__HAL_RCC_CAN2_RELEASE_RESET();
	}
#endif
}
#endif

void can_init(can_data_t *hcan, CAN_TypeDef *instance)
{
	device_can_init(hcan, instance);
}

bool can_set_bittiming(can_data_t *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
	if (  (brp>0) && (brp<=1024)
	   && (phase_seg1>0) && (phase_seg1<=16)
	   && (phase_seg2>0) && (phase_seg2<=8)
	   && (sjw>0) && (sjw<=4)
		  ) {
		hcan->brp = brp & 0x3FF;
		hcan->phase_seg1 = phase_seg1;
		hcan->phase_seg2 = phase_seg2;
		hcan->sjw = sjw;
		return true;
	} else {
		return false;
	}
}

#if defined(STM32G0)
void FDCAN_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0;
	sFilterConfig.FilterID2 = 0;
	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

	HAL_FDCAN_Start(&hfdcan2);
}
#endif

void can_enable(can_data_t *hcan, bool loop_back, bool listen_only, bool one_shot)
{
#if defined(STM32F0) || defined(STM32F4)
	CAN_TypeDef *can = hcan->instance;

	uint32_t mcr = CAN_MCR_INRQ
				   | CAN_MCR_ABOM
				   | CAN_MCR_TXFP
				   | (one_shot ? CAN_MCR_NART : 0);

	uint32_t btr = ((uint32_t)(hcan->sjw-1)) << 24
				   | ((uint32_t)(hcan->phase_seg1-1)) << 16
				   | ((uint32_t)(hcan->phase_seg2-1)) << 20
				   | (hcan->brp - 1)
				   | (loop_back ? CAN_MODE_LOOPBACK : 0)
				   | (listen_only ? CAN_MODE_SILENT : 0);

	// Reset CAN peripheral
	can->MCR |= CAN_MCR_RESET;
	while ((can->MCR & CAN_MCR_RESET) != 0);                                                 // reset bit is set to zero after reset
	while ((can->MSR & CAN_MSR_SLAK) == 0);                                                // should be in sleep mode after reset

	// Completely reset while being of the bus
	rcc_reset(can);

	can->MCR |= CAN_MCR_INRQ;
	while ((can->MSR & CAN_MSR_INAK) == 0);

	can->MCR = mcr;
	can->BTR = btr;

	can->MCR &= ~CAN_MCR_INRQ;
	while ((can->MSR & CAN_MSR_INAK) != 0);

	uint32_t filter_bit = 0x00000001;
	can->FMR |= CAN_FMR_FINIT;
	can->FMR &= ~CAN_FMR_CAN2SB;
	can->FA1R &= ~filter_bit;        // disable filter
	can->FS1R |= filter_bit;         // set to single 32-bit filter mode
	can->FM1R &= ~filter_bit;        // set filter mask mode for filter 0
	can->sFilterRegister[0].FR1 = 0;     // filter ID = 0
	can->sFilterRegister[0].FR2 = 0;     // filter Mask = 0
	can->FFA1R &= ~filter_bit;       // assign filter 0 to FIFO 0
	can->FA1R |= filter_bit;         // enable filter
	can->FMR &= ~CAN_FMR_FINIT;

#elif defined(STM32G0)
	// hfdcan2.Instance = FDCAN2;
	hfdcan2.Instance = hcan->instance;
	hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;

	hfdcan2.Init.AutoRetransmission = one_shot ? DISABLE : ENABLE;
	if (loop_back && listen_only) hfdcan2.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
	else if (loop_back) hfdcan2.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
	else if (listen_only) hfdcan2.Init.Mode = FDCAN_MODE_BUS_MONITORING;
	else hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;

	hfdcan2.Init.TransmitPause = DISABLE;
	hfdcan2.Init.ProtocolException = DISABLE;

	hfdcan2.Init.NominalPrescaler = hcan->brp;
	hfdcan2.Init.NominalSyncJumpWidth = hcan->sjw;
	hfdcan2.Init.NominalTimeSeg1 = hcan->phase_seg1;
	hfdcan2.Init.NominalTimeSeg2 = hcan->phase_seg2;
		
	hfdcan2.Init.DataPrescaler = hcan->brp;
	hfdcan2.Init.DataSyncJumpWidth = hcan->sjw;
	hfdcan2.Init.DataTimeSeg1 = hcan->phase_seg1;
	hfdcan2.Init.DataTimeSeg2 = hcan->phase_seg2;

	hfdcan2.Init.StdFiltersNbr = 1;
	hfdcan2.Init.ExtFiltersNbr = 0;
	hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    HAL_FDCAN_Init(&hfdcan2);
	FDCAN_Config();
#endif

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, ~GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif
}

void can_disable(can_data_t *hcan)
{
	CAN_TypeDef *can = hcan->instance;
#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif

#if defined(STM32F0) || defined(STM32F4)
	can->MCR |= CAN_MCR_INRQ ; // send can controller into initialization mode
#elif defined(STM32G0)
	can->CCCR |= FDCAN_CCCR_INIT ; // send can controller into initialization mode
#endif
}

bool can_is_enabled(can_data_t *hcan)
{
	CAN_TypeDef *can = hcan->instance;
#if defined(STM32F0) || defined(STM32F4)
	return (can->MCR & CAN_MCR_INRQ) == 0;
#elif defined(STM32G0)
	(void) hcan;
	return (can->CCCR & FDCAN_CCCR_INIT) == 0;
#endif
}

bool can_is_rx_pending(can_data_t *hcan)
{
#if defined(STM32F0) || defined(STM32F4)	
	CAN_TypeDef *can = hcan->instance;
	return ((can->RF0R & CAN_RF0R_FMP0) != 0);
#elif defined(STM32G0)
	(void) hcan;
	return ((FDCAN2->RXF0S & FDCAN_RXF0S_F0FL) != 0);
#endif	
}

bool can_receive(can_data_t *hcan, struct gs_host_frame *rx_frame)
{
#if defined(STM32F0) || defined(STM32F4)
	CAN_TypeDef *can = hcan->instance;

	if (can_is_rx_pending(hcan)) {
		CAN_FIFOMailBox_TypeDef *fifo = &can->sFIFOMailBox[0];

		if (fifo->RIR &  CAN_RI0R_IDE) {
			rx_frame->can_id = CAN_EFF_FLAG | ((fifo->RIR >> 3) & 0x1FFFFFFF);
		} else {
			rx_frame->can_id = (fifo->RIR >> 21) & 0x7FF;
		}

		if (fifo->RIR & CAN_RI0R_RTR)  {
			rx_frame->can_id |= CAN_RTR_FLAG;
		}

		rx_frame->can_dlc = fifo->RDTR & CAN_RDT0R_DLC;

		rx_frame->data[0] = (fifo->RDLR >>  0) & 0xFF;
		rx_frame->data[1] = (fifo->RDLR >>  8) & 0xFF;
		rx_frame->data[2] = (fifo->RDLR >> 16) & 0xFF;
		rx_frame->data[3] = (fifo->RDLR >> 24) & 0xFF;
		rx_frame->data[4] = (fifo->RDHR >>  0) & 0xFF;
		rx_frame->data[5] = (fifo->RDHR >>  8) & 0xFF;
		rx_frame->data[6] = (fifo->RDHR >> 16) & 0xFF;
		rx_frame->data[7] = (fifo->RDHR >> 24) & 0xFF;

		can->RF0R |= CAN_RF0R_RFOM0; // release FIFO

		return true;
	} else {
		return false;
	}
#elif defined(STM32G0)
	(void) hcan;
	uint32_t rxf0s = SOC_CAN->RXF0S;

	if (rxf0s & FDCAN_RXF0S_F0FL) {
		// Read and ack data packet
		uint32_t idx = (rxf0s & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
		FDCAN_FIFO_TypeDef *rxf0 = &MSG_RAM.RXF0[idx];
		uint32_t ids = rxf0->id_section;

		if (ids & FDCAN_XTD)
			rx_frame->can_id = (ids & 0x1fffffff) | CANMSG_ID_EFF;
		else
			rx_frame->can_id = (ids >> 18) & 0x7ff;
		rx_frame->can_id |= ids & FDCAN_RTR ? CANMSG_ID_RTR : 0;
		rx_frame->can_dlc = (rxf0->dlc_section >> 16) & 0x0f;

		rx_frame->data[0] = (rxf0->data[0] >>  0) & 0xFF;
		rx_frame->data[1] = (rxf0->data[0] >>  8) & 0xFF;
		rx_frame->data[2] = (rxf0->data[0] >> 16) & 0xFF;
		rx_frame->data[3] = (rxf0->data[0] >> 24) & 0xFF;
		rx_frame->data[4] = (rxf0->data[1] >>  0) & 0xFF;
		rx_frame->data[5] = (rxf0->data[1] >>  8) & 0xFF;
		rx_frame->data[6] = (rxf0->data[1] >> 16) & 0xFF;
		rx_frame->data[7] = (rxf0->data[1] >> 24) & 0xFF;

		SOC_CAN->RXF0A = idx;

		return true;
	} else {
		return false;
	}
#endif
}

#if defined(STM32F0) || defined(STM32F4)
static CAN_TxMailBox_TypeDef *can_find_free_mailbox(can_data_t *hcan)
{
	CAN_TypeDef *can = hcan->instance;

	uint32_t tsr = can->TSR;
	if ( tsr & CAN_TSR_TME0 ) {
		return &can->sTxMailBox[0];
	} else if ( tsr & CAN_TSR_TME1 ) {
		return &can->sTxMailBox[1];
	} else if ( tsr & CAN_TSR_TME2 ) {
		return &can->sTxMailBox[2];
	} else {
		return 0;
	}
}
#endif

bool can_send(can_data_t *hcan, struct gs_host_frame *frame)
{
#if defined(STM32F0) || defined(STM32F4)

	CAN_TxMailBox_TypeDef *mb = can_find_free_mailbox(hcan);
	if (mb != 0) {

		/* first, clear transmission request */
		mb->TIR &= CAN_TI0R_TXRQ;

		if (frame->can_id & CAN_EFF_FLAG) { // extended id
			mb->TIR = CAN_ID_EXT | (frame->can_id & 0x1FFFFFFF) << 3;
		} else {
			mb->TIR = (frame->can_id & 0x7FF) << 21;
		}

		if (frame->can_id & CAN_RTR_FLAG) {
			mb->TIR |= CAN_RTR_REMOTE;
		}

		mb->TDTR &= 0xFFFFFFF0;
		mb->TDTR |= frame->can_dlc & 0x0F;

		mb->TDLR =
			  ( frame->data[3] << 24 )
			| ( frame->data[2] << 16 )
			| ( frame->data[1] <<  8 )
			| ( frame->data[0] <<  0 );

		mb->TDHR =
			  ( frame->data[7] << 24 )
			| ( frame->data[6] << 16 )
			| ( frame->data[5] <<  8 )
			| ( frame->data[4] <<  0 );

		/* request transmission */
		mb->TIR |= CAN_TI0R_TXRQ;

		return true;
	} else {
		return false;
	}

#elif defined(STM32G0)
	(void) hcan;
    uint32_t txfqs = SOC_CAN->TXFQS;
    if (txfqs & FDCAN_TXFQS_TFQF)
        // No space in transmit fifo - wait for irq
        return false;

    uint32_t w_index = ((txfqs & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos);
    FDCAN_FIFO_TypeDef *txfifo = &MSG_RAM.TXFIFO[w_index];
    uint32_t ids;
    if (frame->can_id & CANMSG_ID_EFF)
        ids = (frame->can_id & 0x1fffffff) | FDCAN_XTD;
    else
        ids = (frame->can_id & 0x7ff) << 18;
    ids |= frame->can_id & CANMSG_ID_RTR ? FDCAN_RTR : 0;
    txfifo->id_section = ids;
    txfifo->dlc_section = (frame->can_dlc & 0x0f) << 16;

	txfifo->data[0] =
		  ( frame->data[3] << 24 )
		| ( frame->data[2] << 16 )
		| ( frame->data[1] <<  8 )
		| ( frame->data[0] <<  0 );

	txfifo->data[1] =
		  ( frame->data[7] << 24 )
		| ( frame->data[6] << 16 )
		| ( frame->data[5] <<  8 )
		| ( frame->data[4] <<  0 );

    SOC_CAN->TXBAR = ((uint32_t)1 << w_index);

	return true;

#endif
}

uint32_t can_get_error_status(can_data_t *hcan)
{
#if defined(STM32F0) || defined(STM32F4)
	CAN_TypeDef *can = hcan->instance;
	return can->ESR;
#elif defined(STM32G0)
	uint32_t err = hcan->instance->PSR;
	/* Write 7 to LEC so we know if it gets set to the same thing again */
	hcan->instance->PSR = 7;
	return err;
#endif
}

static bool status_is_active(uint32_t err)
{
#if defined(STM32F0) || defined(STM32F4)
	return !(err & (CAN_ESR_BOFF | CAN_ESR_EPVF));
#elif defined(STM32G0)
	return !(err & (FDCAN_PSR_BO | FDCAN_PSR_EP));
#endif
}

bool can_parse_error_status(uint32_t err, uint32_t last_err, can_data_t *hcan, struct gs_host_frame *frame)
{
	/* We build up the detailed error information at the same time as we decide
	 * whether there's anything worth sending. This variable tracks that final
	 * result. */
	bool should_send = false;
	(void) hcan;

	frame->echo_id = 0xFFFFFFFF;
	frame->can_id  = CAN_ERR_FLAG;
	frame->can_dlc = CAN_ERR_DLC;
	frame->data[0] = CAN_ERR_LOSTARB_UNSPEC;
	frame->data[1] = CAN_ERR_CRTL_UNSPEC;
	frame->data[2] = CAN_ERR_PROT_UNSPEC;
	frame->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	frame->data[4] = CAN_ERR_TRX_UNSPEC;
	frame->data[5] = 0;
	frame->data[6] = 0;
	frame->data[7] = 0;

	/* We transitioned from passive/bus-off to active, so report the edge. */
	if (!status_is_active(last_err) && status_is_active(err)) {
		frame->can_id |= CAN_ERR_CRTL;
		frame->data[1] |= CAN_ERR_CRTL_ACTIVE;
		should_send = true;
	}

#if defined (STM32G0)
	if (err & FDCAN_PSR_BO) {
		if (!(last_err & FDCAN_PSR_BO)) {
			/* We transitioned to bus-off. */
			frame->can_id |= CAN_ERR_BUSOFF;
			should_send = true;
		}
	}
	/* The Linux sja1000 driver puts these counters here. Seems like as good a
	* place as any. */
	// TX error count
	frame->data[6] = ((hcan->instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);
	// RX error count
	frame->data[7] = ((hcan->instance->ECR & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);

	if (err & FDCAN_PSR_EP) {
		if (!(last_err & FDCAN_PSR_EP)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
			should_send = true;
		}
	}
	else if (err & FDCAN_PSR_EW) {
		if (!(last_err & FDCAN_PSR_EW)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
			should_send = true;
		}
	}

	uint8_t lec = err & FDCAN_PSR_LEC;
#else
	if (err & CAN_ESR_BOFF) {
		if (!(last_err & CAN_ESR_BOFF)) {
			/* We transitioned to bus-off. */
			frame->can_id |= CAN_ERR_BUSOFF;
			should_send = true;
		}
		// - tec (overflowed) / rec (looping, likely used for recessive counting)
		//   are not valid in the bus-off state.
		// - The warning flags remains set, error passive will cleared.
		// - LEC errors will be reported, while the device isn't even allowed to send.
		//
		// Hence only report bus-off, ignore everything else.
		return should_send;
	}

	uint8_t tx_error_cnt = (err>>16) & 0xFF;
	uint8_t rx_error_cnt = (err>>24) & 0xFF;
	/* The Linux sja1000 driver puts these counters here. Seems like as good a
	 * place as any. */
	frame->data[6] = tx_error_cnt;
	frame->data[7] = rx_error_cnt;

	if (err & CAN_ESR_EPVF) {
		if (!(last_err & CAN_ESR_EPVF)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
			should_send = true;
		}
	} else if (err & CAN_ESR_EWGF) {
		if (!(last_err & CAN_ESR_EWGF)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
			should_send = true;
		}
	}

	uint8_t lec = (err>>4) & 0x07;
#endif
	switch (lec) {
		case 0x01: /* stuff error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_STUFF;
			should_send = true;
			break;
		case 0x02: /* form error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_FORM;
			should_send = true;
			break;
		case 0x03: /* ack error */
			frame->can_id |= CAN_ERR_ACK;
			should_send = true;
			break;
		case 0x04: /* bit recessive error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_BIT1;
			should_send = true;
			break;
		case 0x05: /* bit dominant error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_BIT0;
			should_send = true;
			break;
		case 0x06: /* CRC error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
			should_send = true;
			break;
		default: /* 0=no error, 7=no change */
			break;
	}

	return should_send;
}
