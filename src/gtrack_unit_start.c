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

#include <gtrack.h>
#include <gtrack_int.h>

extern const float pInit[];
extern const float spreadInit[];

/**
 *  @b Description
 *  @n
 *		GTRACK Module calls this function to start target tracking. This function is called during modules' allocation step,
 *		once new set of points passes allocation thresholds
 *
 *  @param[in]  handle
 *		This is handle to GTRACK unit
 *  @param[in]  timeStamp
 *		This is an allocation time stamp
 *  @param[in]  tid
 *		This is a target identifier given to a unit
 *  @param[in]  uCenter
 *		This is a pointer to the centroid in measurement coordinates
 *
 *  \ingroup GTRACK_ALG_UNIT_FUNCTION
 *
 *  @retval
 *      None
 */

void gtrack_unitStart(void *handle, uint64_t timeStamp, uint32_t tid, GTRACK_measurement_vector *uCenter)
{
    GtrackUnitInstance     *inst;
    GTRACK_measurementUnion u;
    int                     n;

    inst = (GtrackUnitInstance *)handle;

    inst->tid                = tid;
    inst->heartBeatCount     = timeStamp;
    inst->allocationTime     = timeStamp;
    inst->allocationRange    = uCenter->range;
    inst->allocationVelocity = uCenter->doppler;
    inst->estNumOfPoints     = 0;

    /* Initialize state and counters */
    inst->state                  = TRACK_STATE_DETECTION;
    inst->active2freeCount       = 0;
    inst->detect2activeCount     = 0;
    inst->detect2freeCount       = 0;
    inst->currentStateVectorType = inst->stateVectorType;

    inst->isTargetStatic = false;

    /* Radial Velocity initialization */
    /* Radial Velocity handling is set to start with range rate filtering */
    inst->velocityHandling = VELOCITY_INIT;

    u.vector = *uCenter;

    u.vector.doppler = gtrack_unrollRadialVelocity(inst->maxRadialVelocity, inst->initialRadialVelocity, uCenter->doppler);
    inst->rangeRate  = u.vector.doppler;

    /* Initialize a-priori State information */
    gtrack_spherical2cartesian(inst->currentStateVectorType, u.array, inst->S_apriori_hat);
    memcpy(inst->H_s.array, u.array, sizeof(u.array)); /* Initialize Hs to measurment vector */

    /* Initialize measurments error limits */
    gtrack_calcMeasurementLimits(u.vector.range, &inst->gatingParams->limits, &inst->H_limits.vector);

    /* Initialize a-priori Process covariance */

    /* P_apriori_hat = eye(6) */
    /* memcpy(inst->P_apriori_hat, eye6x6, sizeof(eye6x6)); */

    /* P_apriori_hat = diag([0,0,0.5,0.5,1,1]) */
    memset(inst->P_apriori_hat, 0, sizeof(inst->P_apriori_hat));
    for (n = 0; n < GTRACK_STATE_VECTOR_SIZE; n++)
        inst->P_apriori_hat[n * GTRACK_STATE_VECTOR_SIZE + n] = pInit[n];

    memset(inst->gD, 0, sizeof(inst->gD));
    for (n = 0; n < GTRACK_MEASUREMENT_VECTOR_SIZE; n++)
        inst->estSpread.array[n] = spreadInit[n];

    inst->G       = inst->gatingParams->gain;
    inst->sFactor = 1.0f;


#ifdef GTRACK_LOG_ENABLED
    if (inst->verbose & VERBOSE_STATE_INFO)
        gtrack_log(GTRACK_VERBOSE_DEBUG, "%llu: uid[%d] ALLOCATED, TID %d, Range %5.2f, Angle %2.4f, Doppler %5.1f=>%5.1f\n", inst->heartBeatCount, inst->uid, inst->tid, u.vector.range, u.array[1], uCenter->doppler, u.vector.doppler);
#endif
}
