//------------------------------------------------------------------------------
//  Description:  Header file for TJ_DMA.c
//
//  
//

//------------------------------------------------------------------------------
// Change Log:
//------------------------------------------------------------------------------
// Version:  
// Comments: 
//           
//
// Version:  1.00
// Comments: Initial Release Version
//------------------------------------------------------------------------------



#ifndef TJ_DMA_H
#define TJ_DMA_H

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

void TJ_init_dma(void);
void start_dma (void);
interrupt void local_D_INTCH1_ISR(void);
interrupt void local_D_INTCH2_ISR(void);

#endif  // end of TJ_DMA_H definition









