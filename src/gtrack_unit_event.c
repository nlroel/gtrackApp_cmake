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

#include <math.h>

#include <gtrack.h>
#include <gtrack_int.h>

/**
 *  @b Description
 *  @n
 *		GTRACK Module calls this function to run GTRACK unit level state machine
 *
 *  @param[in]  handle
 *		This is handle to GTARCK unit
 *  @param[in]  num
 *		This is number of associated measurments
 *
 *  \ingroup GTRACK_ALG_UNIT_FUNCTION
 *
 *  @retval
 *      None
 */
void gtrack_unitEvent(void *handle, uint16_t num)
{
    GtrackUnitInstance *inst;
    uint16_t            thre;
    uint16_t            numBoxes;

    inst = (GtrackUnitInstance *)handle;

    switch (inst->state)
    {
        case TRACK_STATE_DETECTION:
            if (num > inst->allocationParams->pointsThre)
            {
                /* Hit Event */
                inst->detect2freeCount = 0;
                inst->detect2activeCount++;
                if (inst->detect2activeCount > inst->stateParams->det2actThre)
                {
                    inst->state = TRACK_STATE_ACTIVE;
#ifdef GTRACK_LOG_ENABLED
                    if (inst->verbose & VERBOSE_STATE_INFO)
                        gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d] DET=>ACTIVE\n", inst->heartBeatCount, inst->uid);
#endif
                }
            }
            else
            {
                if (num == 0)
                {
                    /* Miss */
                    inst->detect2freeCount++;
                    if (inst->detect2activeCount > 0)
                        inst->detect2activeCount--;

                    if (inst->detect2freeCount > inst->stateParams->det2freeThre)
                    {
                        inst->state = TRACK_STATE_FREE;
#ifdef GTRACK_LOG_ENABLED
                        if (inst->verbose & VERBOSE_STATE_INFO)
                            gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d] DET=>FREE\n", inst->heartBeatCount, inst->uid);
#endif
                    }
                }
                else
                    inst->detect2freeCount = 0;
            }
            break;

        case TRACK_STATE_ACTIVE:
            if (num)
            {
                /* Hit Event */
                inst->active2freeCount = 0;
            }
            else
            {
                /* Miss */
                inst->active2freeCount++;

                /* Set threshold based on whether target is static, or is in exit zone */
                /* If target is static and inside the static box */
                if (inst->sceneryParams->numStaticBoxes)
                {
                    thre = inst->stateParams->exit2freeThre;
                    for (numBoxes = 0; numBoxes < inst->sceneryParams->numStaticBoxes; numBoxes++)
                    {
                        if (gtrack_isPointInsideBox((GTRACK_cartesian_position *)inst->S_hat, &inst->sceneryParams->staticBox[numBoxes]))
                        {
                            if (inst->isTargetStatic)
                                thre = inst->stateParams->static2freeThre;
                            else
                                thre = inst->stateParams->active2freeThre;
                            break;
                        }
                    }
                }
                else
                {
                    /* Normal moving target */
                    thre = inst->stateParams->active2freeThre;
                }

                /* Threshold can not be more than lifetime of the target */
                if (thre > inst->heartBeatCount)
                    thre = (uint16_t)inst->heartBeatCount;

                if (inst->active2freeCount > thre)
                {
                    inst->state = TRACK_STATE_FREE;
#ifdef GTRACK_LOG_ENABLED
                    if (inst->verbose & VERBOSE_STATE_INFO)
                        gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d] ACTIVE=>FREE\n", inst->heartBeatCount, inst->uid);
#endif
                }
            }
            break;

        default:
            break;
    }
}
