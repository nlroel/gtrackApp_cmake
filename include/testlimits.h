/*
* Copyright (C) 2024 Texas Instruments Incorporated
*
* All rights reserved not granted herein.
* Limited License.  
*
* Texas Instruments Incorporated grants a world-wide, royalty-free, 
* non-exclusive license under copyrights and patents it now or hereafter 
* owns or controls to make, have made, use, import, offer to sell and sell ("Utilize")
* this software subject to the terms herein.  With respect to the foregoing patent 
* license, such license is granted  solely to the extent that any such patent is necessary 
* to Utilize the software alone.  The patent license shall not apply to any combinations which 
* include this software, other than combinations with devices manufactured by or for TI ("TI Devices").  
* No hardware patent is licensed hereunder.
*
* Redistributions must preserve existing copyright notices and reproduce this license (including the 
* above copyright notice and the disclaimer and (if applicable) source code license limitations below) 
* in the documentation and/or other materials provided with the distribution
*
* Redistribution and use in binary form, without modification, are permitted provided that the following
* conditions are met:
*
*	* No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any 
*     software provided in binary form.
*	* any redistribution and use are licensed by TI for use only with TI Devices.
*	* Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
*
* If software source code is provided to you, modification and redistribution of the source code are permitted 
* provided that the following conditions are met:
*
*   * any redistribution and use of the source code, including any resulting derivative works, are licensed by 
*     TI for use only with TI Devices.
*   * any redistribution and use of any object code compiled from the source code and any resulting derivative 
*     works, are licensed by TI for use only with TI Devices.
*
* Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or 
* promote products derived from this software without specific prior written permission.
*
* DISCLAIMER.
*
* THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
* BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
* IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _TESTLIMITS_H
#define _TESTLIMITS_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef SUBSYS_MSS
#ifdef GTRACK_3D
/* Acceptable Resource Limits for 3D Tracker @ R4F */
#define LIMIT_PROG_MEMORY_SIZE        20000 /* shall take < 20k of proggram memory */
#define LIMIT_DATA_MEMORY_SIZE_MODULE 3200 /* 3.2k data memory for module */
#define LIMIT_DATA_MEMORY_SIZE_UNIT   1300 /* 1.3k of data memory per unit, so 20 tracks shall take < 3.2k+20*1.3 = 29.2k data memory  */
#define LIMIT_USEC_PER_TRACK_AVE      250 /* shall take < 250us per track at average */
#define LIMIT_USEC_PER_TRACK_MAX      350 /* shall take < 350us per track at maximum */
#else
/* Acceptable Resource Limits for 2D Tracker @ R4F */
#define LIMIT_PROG_MEMORY_SIZE        18000 /* shall take  < 18k of program memory */
#define LIMIT_DATA_MEMORY_SIZE_MODULE 2800 /* 2.8k data memory for module */
#define LIMIT_DATA_MEMORY_SIZE_UNIT   800 /* 0.8k of data memory per unit, so 20 tracks shall take < 2.8k+20*0.8 = 18.8k data memory  */
#define LIMIT_USEC_PER_TRACK_AVE      200 /* shall take < 200us per track at average */
#define LIMIT_USEC_PER_TRACK_MAX      250 /* shall take < 250us per track at maximum */
#endif
#else
#ifdef GTRACK_3D
/* Acceptable Resource Limits for 3D Tracker @ C67 */
#define LIMIT_PROG_MEMORY_SIZE        31000 /* shall take < 31k of proggram memory */
#define LIMIT_DATA_MEMORY_SIZE_MODULE 3200 /* 3.2k data memory for module */
#define LIMIT_DATA_MEMORY_SIZE_UNIT   1300 /* 1.3k of data memory per unit, so 20 tracks shall take < 3.2k+20*1.3 = 29.2k data memory  */
#define LIMIT_USEC_PER_TRACK_AVE      120 /* shall take < 120us per track at average */
#define LIMIT_USEC_PER_TRACK_MAX      200 /* shall take < 200us per track at maximum */
#else
/* Acceptable Resource Limits for 2D Tracker @ C67 */
#define LIMIT_PROG_MEMORY_SIZE        29000 /* shall take  < 29k of program memory */
#define LIMIT_DATA_MEMORY_SIZE_MODULE 2800 /* 3k data memory for module */
#define LIMIT_DATA_MEMORY_SIZE_UNIT   800 /* 0.8k of data memory per unit, so 20 tracks shall take < 2.8k+20*0.8 = 18.8k data memory  */
#define LIMIT_USEC_PER_TRACK_AVE      80 /* shall take < 80us per tarck at average */
#define LIMIT_USEC_PER_TRACK_MAX      120 /* shall take < 120us per track at maximum */
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif
