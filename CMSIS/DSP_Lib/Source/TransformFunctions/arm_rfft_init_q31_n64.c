/* ----------------------------------------------------------------------
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.
*
* $Date:        19. March 2015
* $Revision: 	V.1.4.5
*
* Project: 	    CMSIS DSP Library
* Title:	    arm_rfft_init_q31.c
*
* Description:	RFFT & RIFFT Q31 initialisation function
*
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

#include "arm_math.h"
#include "arm_common_tables.h"
#include "arm_const_structs.h"

/**
* @ingroup groupTransforms
*/

/**
* @addtogroup RealFFT
* @{
*/

/**
* \par
* Generation fixed-point realCoefAQ31 array in Q31 format:
* \par
* n = 4096
* <pre>for (i = 0; i < n; i++)
* {
*    pATable[2 * i] = 0.5 * (1.0 - sin (2 * PI / (double) (2 * n) * (double) i));
*    pATable[2 * i + 1] = 0.5 * (-1.0 * cos (2 * PI / (double) (2 * n) * (double) i));
* }</pre>
* \par
* Convert to fixed point Q31 format
*     round(pATable[i] * pow(2, 31))
*/


static const q31_t realCoefAQ31[64] =
{
    0x40000000, 0xC0000000, 0x39BA1651, 0xC04EE4B8,
    0x3383A3E2, 0xC13AD060, 0x2D6BF9D1, 0xC2C17D52,
    0x27821D59, 0xC4DF2862, 0x21D4A2C8, 0xC78E9A1D,
    0x1C71898D, 0xCAC933AE, 0x176619B6, 0xCE86FF2A,
    0x12BEC333, 0xD2BEC333, 0x0E86FF2A, 0xD76619B6,
    0x0AC933AE, 0xDC71898D, 0x078E9A1D, 0xE1D4A2C8,
    0x04DF2862, 0xE7821D59, 0x02C17D52, 0xED6BF9D1,
    0x013AD060, 0xF383A3E2, 0x004EE4B8, 0xF9BA1651,
    0x00000000, 0x00000000, 0x004EE4B8, 0x0645E9AF,
    0x013AD060, 0x0C7C5C1E, 0x02C17D52, 0x1294062F,
    0x04DF2862, 0x187DE2A7, 0x078E9A1D, 0x1E2B5D38,
    0x0AC933AE, 0x238E7673, 0x0E86FF2A, 0x2899E64A,
    0x12BEC333, 0x2D413CCD, 0x176619B6, 0x317900D6,
    0x1C71898D, 0x3536CC52, 0x21D4A2C8, 0x387165E3,
    0x27821D59, 0x3B20D79E, 0x2D6BF9D1, 0x3D3E82AE,
    0x3383A3E2, 0x3EC52FA0, 0x39BA1651, 0x3FB11B48
};


/**
* \par
* Generation of realCoefBQ31 array:
* \par
*  n = 4096
* <pre>for (i = 0; i < n; i++)
* {
*    pBTable[2 * i] = 0.5 * (1.0 + sin (2 * PI / (double) (2 * n) * (double) i));
*    pBTable[2 * i + 1] = 0.5 * (1.0 * cos (2 * PI / (double) (2 * n) * (double) i));
* } </pre>
* \par
* Convert to fixed point Q31 format
*     round(pBTable[i] * pow(2, 31))
*
*/


static const q31_t realCoefBQ31[64] =
{
    0x40000000, 0x40000000, 0x4645E9AF, 0x3FB11B48,
    0x4C7C5C1E, 0x3EC52FA0, 0x5294062F, 0x3D3E82AE,
    0x587DE2A7, 0x3B20D79E, 0x5E2B5D38, 0x387165E3,
    0x638E7673, 0x3536CC52, 0x6899E64A, 0x317900D6,
    0x6D413CCD, 0x2D413CCD, 0x717900D6, 0x2899E64A,
    0x7536CC52, 0x238E7673, 0x787165E3, 0x1E2B5D38,
    0x7B20D79E, 0x187DE2A7, 0x7D3E82AE, 0x1294062F,
    0x7EC52FA0, 0x0C7C5C1E, 0x7FB11B48, 0x0645E9AF,
    0x80000000, 0x00000000, 0x7FB11B48, 0xF9BA1651,
    0x7EC52FA0, 0xF383A3E2, 0x7D3E82AE, 0xED6BF9D1,
    0x7B20D79E, 0xE7821D59, 0x787165E3, 0xE1D4A2C8,
    0x7536CC52, 0xDC71898D, 0x717900D6, 0xD76619B6,
    0x6D413CCD, 0xD2BEC333, 0x6899E64A, 0xCE86FF2A,
    0x638E7673, 0xCAC933AE, 0x5E2B5D38, 0xC78E9A1D,
    0x587DE2A7, 0xC4DF2862, 0x5294062F, 0xC2C17D52,
    0x4C7C5C1E, 0xC13AD060, 0x4645E9AF, 0xC04EE4B8
};

/**
* @brief  Initialization function for the Q31 RFFT/RIFFT.
* @param[in, out] *S             points to an instance of the Q31 RFFT/RIFFT structure.
* @param[in]      fftLenReal     length of the FFT.
* @param[in]      ifftFlagR      flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform.
* @param[in]      bitReverseFlag flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output.
* @return		The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_ARGUMENT_ERROR if <code>fftLenReal</code> is not a supported value.
*
* \par Description:
* \par
* The parameter <code>fftLenReal</code>	Specifies length of RFFT/RIFFT Process. Supported FFT Lengths are 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192.
* \par
* The parameter <code>ifftFlagR</code> controls whether a forward or inverse transform is computed.
* Set(=1) ifftFlagR to calculate RIFFT, otherwise RFFT is calculated.
* \par
* The parameter <code>bitReverseFlag</code> controls whether output is in normal order or bit reversed order.
* Set(=1) bitReverseFlag for output to be in normal order otherwise output is in bit reversed order.
* \par    7
* This function also initializes Twiddle factor table.
*/

arm_status arm_rfft_init_q31_n64(
    arm_rfft_instance_q31* S,
    uint32_t fftLenReal,
    uint32_t ifftFlagR,
    uint32_t bitReverseFlag)
{
    /*  Initialise the default arm status */
    arm_status status = ARM_MATH_SUCCESS;

    /*  Initialize the Real FFT length */
    S->fftLenReal = (uint16_t)fftLenReal;

    /*  Initialize the Twiddle coefficientA pointer */
    S->pTwiddleAReal = (q31_t*)realCoefAQ31;

    /*  Initialize the Twiddle coefficientB pointer */
    S->pTwiddleBReal = (q31_t*)realCoefBQ31;

    /*  Initialize the Flag for selection of RFFT or RIFFT */
    S->ifftFlagR = (uint8_t)ifftFlagR;

    /*  Initialize the Flag for calculation Bit reversal or not */
    S->bitReverseFlagR = (uint8_t)bitReverseFlag;

    /*  Initialization of coef modifier depending on the FFT length */
    switch (S->fftLenReal)
    {
    case 64u:
        S->twidCoefRModifier = 1u;
        S->pCfft = &arm_cfft_sR_q31_len32;
        break;
    case 32u:
        S->twidCoefRModifier = 2u;
        S->pCfft = &arm_cfft_sR_q31_len16;
        break;
    default:
        /*  Reporting argument error if rfftSize is not valid value */
        status = ARM_MATH_ARGUMENT_ERROR;
        break;
    }

    /* return the status of RFFT Init function */
    return (status);
}

/**
* @} end of RealFFT group
*/
