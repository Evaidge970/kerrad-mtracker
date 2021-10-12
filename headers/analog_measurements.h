#pragma once

#include "hardware_interface.h"

class AdcConverter
{
public:
	static void SetSampler()
	{

		AdcRegs.ADCTRL3.bit.SMODE_SEL = 0;	// próbkowanie sekwencyjne
		AdcRegs.ADCTRL3.bit.ADCCLKPS = 0xF;

		AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 7;
		AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 1;
		AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1;
		AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 1;
		AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 1;
		AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 1;
		AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 1;
		AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 1;
		AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 1;
		AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;
	}

	static void Read()
	{
	   /*
	   res = AdcRegs.ADCRESULT0;
	   res += AdcRegs.ADCRESULT1;
	   res += AdcRegs.ADCRESULT2;
	   res += AdcRegs.ADCRESULT3;
	   res += AdcRegs.ADCRESULT4;
	   res += AdcRegs.ADCRESULT5;
	   res += AdcRegs.ADCRESULT6;
	   res += AdcRegs.ADCRESULT7;
	   res >>= 3;
	   */
	}

	static void StartConversion()
	{
		AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;
	}
};
