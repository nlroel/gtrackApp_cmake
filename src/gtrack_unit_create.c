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

#include <gtrack.h>
#include <gtrack_int.h>

/**
 *  @b Description
 *  @n
 *		GTRACK Module calls this function to instantiate GTRACK Unit with desired configuration parameters.
 *		Function returns a handle, which is used my module to call units' methods
 *
 *  @param[in]  params
 *		This is a pointer to the configuration structure. The structure contains all parameters that are exposed by GTRACK alrorithm.
 *		The configuration does not need to persist.
 *  @param[out] errCode
 *      Error code populated on error, see \ref GTRACK_ERROR_CODE
 *  \ingroup GTRACK_ALG_UNIT_FUNCTION
 *
 *  @retval
 *      Handle to GTRACK unit
 */

void *gtrack_unitCreate(TrackingParams *params, int32_t *errCode)
{
    GtrackUnitInstance *inst;

    *errCode = GTRACK_EOK;

    inst = (GtrackUnitInstance *)gtrack_alloc(1, sizeof(GtrackUnitInstance));
    if (inst == NULL)
    {
        *errCode = GTRACK_ENOMEM;
        goto exit;
    }

    memset(inst, 0, sizeof(GtrackUnitInstance));
    /* Parameters */
    inst->gatingParams     = &params->gatingParams;
    inst->stateParams      = &params->stateParams;
    inst->allocationParams = &params->allocationParams;
    inst->unrollingParams  = &params->unrollingParams;
    inst->variationParams  = &params->variationParams;
    inst->sceneryParams    = &params->sceneryParams;

    memcpy(inst->maxAcceleration, params->maxAcceleration, sizeof(inst->maxAcceleration));

    inst->uid               = params->uid;
    inst->isTargetStatic    = false;
    inst->maxRadialVelocity = params->maxRadialVelocity;


    inst->radialVelocityResolution = params->radialVelocityResolution;
    inst->verbose                  = params->verbose;
    inst->initialRadialVelocity    = params->initialRadialVelocity;

    inst->F = params->F;
    inst->Q = params->Q;

    switch (params->stateVectorType)
    {

        case GTRACK_STATE_VECTORS_2DA:
            inst->stateVectorType         = GTRACK_STATE_VECTORS_2DA;
            inst->stateVectorDimNum       = 2;
            inst->stateVectorDimLength    = 3;
            inst->stateVectorLength       = 6;
            inst->measurementVectorLength = 3;
            break;

        case GTRACK_STATE_VECTORS_3DA:
            inst->stateVectorType         = GTRACK_STATE_VECTORS_3DA;
            inst->stateVectorDimNum       = 3;
            inst->stateVectorDimLength    = 3;
            inst->stateVectorLength       = 9;
            inst->measurementVectorLength = 4;
            break;

        default:
            *errCode = GTRACK_EINVAL;
            goto exit;
    }

    inst->dt    = params->deltaT;
    inst->state = TRACK_STATE_FREE;

exit:
    if (*errCode != GTRACK_EOK)
    {
        if (inst != NULL)
            gtrack_free(inst, sizeof(GtrackUnitInstance));
        inst = NULL;
    }
    return (void *)inst;
}
