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

#include <string.h>
#include <math.h>
#include <float.h>
#include <gtrack.h>
#include <gtrack_int.h>

/**
 *  @b Description
 *  @n
 *	   Algorithm level step funtion
 *      Application shall call this function to process one frame of measurements with a given instance of the algorithm
 *
 *  @param[in]  handle
 *      Handle to GTRACK module
 *  @param[in]  point
 *      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
 *  @param[in]  var
 *      Pointer to an array of input measurment variances. Shall be set to NULL if variances are unknown
 *  @param[in]  mNum
 *      Number of input measurements
 *  @param[out]  t
 *      Pointer to an array of \ref GTRACK_targetDesc. Application shall provide sufficient space for the expected number of targets.
 *      This function populates the descritions for each of the tracked target
 *  @param[out]  tNum
 *      Pointer to a uint16_t value.
 *      Function returns a number of populated target descriptos
 *  @param[out]  mIndex
 *      Pointer to an array of uint8_t indices. Application shall provide sufficient space to index all measurment points.
 *      This function populates target indices, indicating which tracking ID was assigned to each measurment. See Target ID defeinitions, example \ref GTRACK_ID_POINT_NOT_ASSOCIATED
 *	   Shall be set to NULL when indices aren't required.
 *  @param[out]  bench
 *      Pointer to an array of benchmarking results. Each result is a 32bit timestamp. The array size shall be \ref GTRACK_BENCHMARK_SIZE.
 *      This function populates the array with the timestamps of free runing CPU cycles count.
 *	   Shall be set to NULL when benchmarking isn't required.
 *
 *  \ingroup GTRACK_ALG_EXTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */

void gtrack_step(void *handle, GTRACK_measurementPoint *point, GTRACK_measurement_vector *var, uint16_t mNum, GTRACK_targetDesc *t, uint16_t *tNum, uint8_t *mIndex, uint32_t *bench)
{
    GtrackModuleInstance *inst;
    uint16_t              n;
    uint16_t              numBoxes;

    GTRACK_cartesian_position pos;

    inst = (GtrackModuleInstance *)handle;

    inst->heartBeat++;

#ifdef GTRACK_LOG_ENABLED
    if (inst->verbose & VERBOSE_WARNING_INFO)
        gtrack_log(GTRACK_VERBOSE_WARNING, "Frame #%llu, %u Target(s), %hu Measurements\n", inst->heartBeat, gtrack_listGetCount(&inst->activeList), mNum);
#endif

    if (mNum > inst->maxNumPoints)
        mNum = inst->maxNumPoints;

    memset(inst->isUniqueIndex, 0xFF, (inst->maxNumPoints >> 3) + 1);

    for (n = 0; n < mNum; n++)
    {
        inst->bestScore[n] = FLT_MAX;

        if (inst->params.sceneryParams.numBoundaryBoxes)
        {

            /* If boundaries exists, set to outside, and overwrite if inside the boundary */
            inst->bestIndex[n] = GTRACK_ID_POINT_BEHIND_THE_WALL;
            gtrack_sph2cart(&point[n].vector, &pos);

            for (numBoxes = 0; numBoxes < inst->params.sceneryParams.numBoundaryBoxes; numBoxes++)
            {
                if (gtrack_isPointInsideBox(&pos, &inst->params.sceneryParams.boundaryBox[numBoxes]))
                {
                    /* Valid point */
                    inst->bestIndex[n] = GTRACK_ID_POINT_NOT_ASSOCIATED;
                    break;
                }
            }
        }
        else
        {
            /* Valid point */
            inst->bestIndex[n] = GTRACK_ID_POINT_NOT_ASSOCIATED;
        }
    }

    if (bench != NULL)
    {
        /* Collect benchamrks */
        bench[GTRACK_BENCHMARK_SETUP] = gtrack_getCycleCount();
        gtrack_modulePredict(inst);
        bench[GTRACK_BENCHMARK_PREDICT] = gtrack_getCycleCount();
        gtrack_moduleAssociate(inst, point, mNum);
        bench[GTRACK_BENCHMARK_ASSOCIATE] = gtrack_getCycleCount();
        gtrack_moduleAllocate(inst, point, mNum);
        bench[GTRACK_BENCHMARK_ALLOCATE] = gtrack_getCycleCount();
        gtrack_moduleUpdate(inst, point, var, mNum);
        bench[GTRACK_BENCHMARK_UPDATE] = gtrack_getCycleCount();
        gtrack_moduleReport(inst, t, tNum);
        bench[GTRACK_BENCHMARK_REPORT] = gtrack_getCycleCount();
    }
    else
    {
        gtrack_modulePredict(inst);
        gtrack_moduleAssociate(inst, point, mNum);
        gtrack_moduleAllocate(inst, point, mNum);
        gtrack_moduleUpdate(inst, point, var, mNum);
        gtrack_moduleReport(inst, t, tNum);
    }

    /* If requested, report uids associated with measurment vector */
    if (mIndex != 0)
    {
        for (n = 0; n < mNum; n++)
        {
            mIndex[n] = inst->bestIndex[n];
        }
    }
}
