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
#include <float.h>
#include <gtrack.h>
#include <gtrack_int.h>

#define PI 3.14159265358979323846f

/**
 *  @b Description
 *  @n
 *      This is a MODULE level predict function. The function is called by external step function to perform unit level kalman filter predictions
 *
 *  @param[in]  inst
 *      Pointer to GTRACK module instance
 *
 *  \ingroup GTRACK_ALG_MODULE_FUNCTION
 *
 *  @retval
 *      None
 */

void gtrack_modulePredict(GtrackModuleInstance *inst)
{
    GTrack_ListElem *tElem;
    uint16_t         uid;

    tElem = gtrack_listGetFirst(&inst->activeList);
    while (tElem != 0)
    {
        uid = tElem->data;
        if (uid > inst->maxNumTracks)
        {
            /* This should never happen */
            gtrack_assert(0);
        }

        gtrack_unitPredict(inst->hTrack[uid]);

        tElem = gtrack_listGetNext(tElem);
    }
}

/**
 *  @b Description
 *  @n
 *      This is a MODULE level associatiation function. The function is called by external step function to associate measurement points with known targets
 *
 *  @param[in]  inst
 *      Pointer to GTRACK module instance
 *  @param[in]  point
 *      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
 *  @param[in]  num
 *      Number of input measurements
 *
 *  \ingroup GTRACK_ALG_MODULE_FUNCTION
 *
 *  @retval
 *      None
 */

void gtrack_moduleAssociate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, uint16_t num)
{
    GTrack_ListElem *tElem;
    uint16_t         uid;

    tElem = gtrack_listGetFirst(&inst->activeList);
    while (tElem != 0)
    {
        uid = tElem->data;
        gtrack_unitScore(inst->hTrack[uid], point, inst->bestScore, inst->bestIndex, inst->isUniqueIndex, num);

        tElem = gtrack_listGetNext(tElem);
    }
}

/**
 *  @b Description
 *  @n
 *      This is a MODULE level allocation function. The function is called by external step function to allocate new targets for the non-associated measurement points
 *
 *  @param[in]  inst
 *      Pointer to GTRACK module instance
 *  @param[in]  point
 *      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
 *  @param[in]  num
 *      Number of input measurements
 *
 *  \ingroup GTRACK_ALG_MODULE_FUNCTION
 *
 *  @retval
 *      None
 */
void gtrack_moduleAllocate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, uint16_t num)
{
    uint16_t n, k;

    //	float un[3], uk[3];
    //	float unSum[3];
    GTRACK_measurementUnion mCenter;
    GTRACK_measurementUnion mCurrent;
    GTRACK_measurementUnion mSum;

    GTRACK_measurement_vector hs;
    uint16_t                  allocNum;
    float                     dist;
    float                     allocSNR;
    GTrack_ListElem          *tElemFree;
    GTrack_ListElem          *tElemActive;
    uint16_t                  uid;
    bool                      isBehind;
    bool                      isSnrThresholdPassed;

    for (n = 0; n < num; n++)
    {
        if (inst->bestIndex[n] == GTRACK_ID_POINT_NOT_ASSOCIATED)
        {

            tElemFree = gtrack_listGetFirst(&inst->freeList);
            if (tElemFree == 0)
            {

#ifdef GTRACK_LOG_ENABLED
                if (inst->verbose & VERBOSE_WARNING_INFO)
                    gtrack_log(GTRACK_VERBOSE_WARNING, "Maximum number of tracks reached!");
#endif
                return;
            }

            inst->allocIndex[0] = n;
            allocNum            = 1;
            allocSNR            = point[n].snr;

            mCenter.vector = point[n].vector;
            mSum.vector    = point[n].vector;

            for (k = n + 1; k < num; k++)
            {
                if (inst->bestIndex[k] == GTRACK_ID_POINT_NOT_ASSOCIATED)
                {

                    mCurrent.vector = point[k].vector;

                    mCurrent.vector.doppler = gtrack_unrollRadialVelocity(inst->params.maxRadialVelocity, mCenter.vector.doppler, mCurrent.vector.doppler);

                    if (fabsf(mCurrent.vector.doppler - mCenter.vector.doppler) < inst->params.allocationParams.maxVelThre)
                    {
                        dist = gtrack_calcDistance(&mCenter.vector, &mCurrent.vector);
                        if (sqrtf(dist) < inst->params.allocationParams.maxDistanceThre)
                        {

                            inst->allocIndex[allocNum] = k;

                            allocNum++;
                            allocSNR += point[k].snr;
                            // Update the centroid
                            gtrack_vectorAdd(GTRACK_MEASUREMENT_VECTOR_SIZE, mCurrent.array, mSum.array, mSum.array);
                            gtrack_vectorScalarMul(GTRACK_MEASUREMENT_VECTOR_SIZE, mSum.array, 1.0f / (float)allocNum, mCenter.array);
                        }
                    }
                }
            }
            if ((allocNum > inst->params.allocationParams.pointsThre) &&
                (fabsf(mCenter.vector.doppler) > inst->params.allocationParams.velocityThre))
            {
                isBehind    = false;
                tElemActive = gtrack_listGetFirst(&inst->activeList);
                while (tElemActive != 0)
                {
                    uid = tElemActive->data;
                    gtrack_unitGetH(inst->hTrack[uid], (float *)&hs);

                    if (gtrack_isPointBehindTarget(&mCenter.vector, &hs))
                    {
                        isBehind = true;
                        break;
                    }
                    tElemActive = gtrack_listGetNext(tElemActive);
                }

                if (isBehind)
                    isSnrThresholdPassed = allocSNR > inst->params.allocationParams.snrThreObscured;
                else
                    isSnrThresholdPassed = allocSNR > inst->params.allocationParams.snrThre;

                if (isSnrThresholdPassed)
                {

                    /* Associate points with new uid  */
                    for (k = 0; k < allocNum; k++)
                        inst->bestIndex[inst->allocIndex[k]] = (uint8_t)tElemFree->data;

                    /* Allocate new tracker */
                    inst->targetNumTotal++;
                    inst->targetNumCurrent++;
                    tElemFree = gtrack_listDequeue(&inst->freeList);

                    gtrack_unitStart(inst->hTrack[tElemFree->data], inst->heartBeat, inst->targetNumTotal, &mCenter.vector);
                    gtrack_listEnqueue(&inst->activeList, tElemFree);
                }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This is a MODULE level update function. The function is called by external step function to perform unit level kalman filter updates
 *
 *  @param[in]  inst
 *      Pointer to GTRACK module instance
 *  @param[in]  point
 *      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
 *  @param[in]  var
 *      Pointer to an array of input measurment variances. Set to NULL if variances are unknown
 *  @param[in]  num
 *      Number of input measurements
 *
 *  \ingroup GTRACK_ALG_MODULE_FUNCTION
 *
 *  @retval
 *      None
 */

void gtrack_moduleUpdate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, GTRACK_measurement_vector *var, uint16_t num)
{
    GTrack_ListElem *tElem;
    GTrack_ListElem *tElemToRemove;
    uint16_t         uid;
    TrackState       state;

    tElem = gtrack_listGetFirst(&inst->activeList);
    while (tElem != 0)
    {
        uid   = tElem->data;
        state = gtrack_unitUpdate(inst->hTrack[uid], point, var, inst->bestIndex, inst->isUniqueIndex, num);
        if (state == TRACK_STATE_FREE)
        {
            tElemToRemove = tElem;
            tElem         = gtrack_listGetNext(tElem);
            gtrack_listRemoveElement(&inst->activeList, tElemToRemove);

            gtrack_unitStop(inst->hTrack[tElemToRemove->data]);
            inst->targetNumCurrent--;
            gtrack_listEnqueue(&inst->freeList, tElemToRemove);
        }
        else
            tElem = gtrack_listGetNext(tElem);
    }
}
/**
 *  @b Description
 *  @n
 *      This is a MODULE level report function. The function is called by external step function to obtain unit level data
 *
 *  @param[in]  inst
 *      Pointer to GTRACK module instance
 *  @param[out]  t
 *      Pointer to an array of \ref GTRACK_targetDesc.
 *      This function populates the descritions for each of the tracked target
 *  @param[out]  tNum
 *      Pointer to a uint16_t value.
 *      Function returns a number of populated target descriptos
 *
 *  \ingroup GTRACK_ALG_MODULE_FUNCTION
 *
 *  @retval
 *      None
 */

void gtrack_moduleReport(GtrackModuleInstance *inst, GTRACK_targetDesc *t, uint16_t *tNum)
{
    GTrack_ListElem *tElem;
    uint16_t         uid;
    uint16_t         num = 0;


    tElem = gtrack_listGetFirst(&inst->activeList);
    while (tElem != 0)
    {
        uid = tElem->data;
        gtrack_unitReport(inst->hTrack[uid], &t[num++]);
        tElem = gtrack_listGetNext(tElem);
    }
    *tNum = num;
}
