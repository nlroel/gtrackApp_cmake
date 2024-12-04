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
#include <string.h>

#include <gtrack.h>
#include <gtrack_int.h>


/**
 *  @b Description
 *  @n
 *		GTRACK Module calls this function to obtain the measurement vector scoring from the GTRACK unit perspective
 *
 *  @param[in]  handle
 *		This is handle to GTRACK unit
 *  @param[in]  point
 *		This is an array of measurement points
 *  @param[inout]  bestScore
 *		This is a pointer current scoresheet with best scores. Only better scores are updated
 *  @param[inout]  bestInd
 *		This is a pointer current scoresheet winners. Only better scores winners are updated
 *  @param[out]  isUnique
 *       This is an array indicating whether point belongs to a single target (1) or not (0)
 *  @param[in]  num
 *		Number of measurement points
 *
 *  \ingroup GTRACK_ALG_UNIT_FUNCTION
 *
 *  @retval
 *      None
 */
void gtrack_unitScore(void *handle, GTRACK_measurementPoint *point, float *bestScore, uint8_t *bestInd, uint8_t *isUnique, uint16_t num)
{
    GtrackUnitInstance *inst;
    uint16_t            n;
    uint16_t            m;

    bool  isWithinLimits;
    float mdp, md;

    GTRACK_measurementUnion u_tilda;

    float score;
    float logdet;

    float rvOut;

    inst = (GtrackUnitInstance *)handle;

    logdet = logf(inst->gC_det);

#ifdef GTRACK_LOG_ENABLED
    if (inst->verbose & VERBOSE_ASSOSIATION_INFO)
    {
        gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d]: Scoring: G=%5.2f, log(det)=%5.2f\n", inst->heartBeatCount, inst->uid, inst->G, logdet);
    }
#endif

    for (n = 0; n < num; n++)
    {

        if (bestInd[n] == GTRACK_ID_POINT_BEHIND_THE_WALL)
            continue;

        gtrack_vectorSub(GTRACK_MEASUREMENT_VECTOR_SIZE, point[n].array, inst->H_s.array, u_tilda.array);

        if (inst->velocityHandling < VELOCITY_LOCKED)
        {
            /* Radial velocity estimation is not yet known, unroll based on velocity measured at allocation time */
            rvOut                  = gtrack_unrollRadialVelocity(inst->maxRadialVelocity, inst->allocationVelocity, point[n].vector.doppler);
            u_tilda.vector.doppler = rvOut - inst->allocationVelocity;
        }
        else
        {
            /* Radial velocity estimation is known */
            rvOut                  = gtrack_unrollRadialVelocity(inst->maxRadialVelocity, inst->H_s.vector.doppler, point[n].vector.doppler);
            u_tilda.vector.doppler = rvOut - inst->H_s.vector.doppler;
        }

        /* Any point outside the limits is outside the gate */
        isWithinLimits = true;
        for (m = 0; m < GTRACK_MEASUREMENT_VECTOR_SIZE; m++)
        {
            if (fabs(u_tilda.array[m]) > inst->H_limits.array[m])
            {
                isWithinLimits = false;
                break;
            }
        }
        if (isWithinLimits == false)
            continue;

        /* For the gating purposes we compute partial Mahalanobis distance, ignoring doppler */
        gtrack_computeMahalanobisPartial(u_tilda.array, inst->gC_inv, &mdp);
        /* Gating Step */
        if (mdp < inst->G)
        {
            if (bestInd[n] < GTRACK_NUM_TRACKS_MAX)
                /* This point is not unique. Clear the indication */
                isUnique[n >> 3] &= ~(1 << (n & 0x0007));

            /* Scoring */
            gtrack_computeMahalanobis(u_tilda.array, inst->gC_inv, &md);
            score = logdet + md;
            if (score < bestScore[n])
            {
                /* If we are the best, register our score, and the index */
                bestScore[n]            = score;
                bestInd[n]              = (uint8_t)inst->uid;
                point[n].vector.doppler = rvOut;
            }
        }
    }
    memcpy(inst->ec, inst->gC_inv, sizeof(inst->ec));
}
