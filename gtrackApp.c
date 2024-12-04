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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "float.h"
#include "math.h"
#include "testlimits.h"
#include <gtrack.h>

#include <stdbool.h>


/* This test application wants to modify default parameters */
GTRACK_sceneryParams appSceneryParams = {
    1,
    { { -4.0f, 4.0f, 0.5f, 7.5f, -1.0f, 3.0f }, { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f } }, /* one boundary box {x1,x2,y1,y2,z1,z2} */
    1,
    { { -3.0f, 3.0f, 2.0f, 6.0f, -0.5f, 2.5f }, { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f } } /* one static box {x1,x2,y1,y2,z1,z2} */
};
GTRACK_gatingParams appGatingParams = {
    3.f,
    { 1.5f, 1.5f, 2.f, 0.f } /* Gating Gain 8x, Limits are set to 2m in depth, width, 2m (if applicable) in height and no limits in doppler */
};
GTRACK_stateParams appStateParams = {
    10U,
    5U,
    50U,
    100U,
    5U /* det2act, det2free, act2free, stat2free, exit2free */
};
GTRACK_allocationParams appAllocationParams = {
    60.f,
    200.f,
    0.1f,
    5U,
    1.5f,
    2.f /* 60 in clear, 200 obscured SNRs, 0.1m/s minimal velocity, 5 points, 1.5m in distance, 2m/s in velocity */
};
/* Using standard deviation of uniformly distributed variable in the range [a b]: 1/sqrt(12)*(b-a) */
GTRACK_varParams appVariationParams = {
    /* Standard deviation of uniformly distributed number in range [a b]: sqrt(1/12)*(b-a) */
    0,
    0,
    0,
    0 /* 1m in width, depth, and height, 2 m/s for doppler */
};

typedef struct
{
    uint32_t frameNum;
    uint32_t numTLVs;

} binFileHeader;

typedef enum
{
    POINT_CLOUD_TYPE = 6,
    TARGET_LIST_TYPE
} binFileElements;

typedef struct
{
    uint32_t type;
    uint32_t length;
} binFileTLV;

typedef struct
{
    uint32_t tid;
    float    S[GTRACK_STATE_VECTOR_SIZE];
} checkTargetDescr;


GTRACK_measurementPoint pointCloud[GTRACK_NUM_POINTS_MAX];
GTRACK_targetDesc       targetDescr[GTRACK_NUM_TRACKS_MAX];

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
int main(int arg0, char** arg1)
{
    GTRACK_moduleConfig       config;
    GTRACK_advancedParameters advParams;


    checkTargetDescr checkDescr[20];

    void *hTrackModule;
    bool  performanceTestResultPass = false;
    bool  frameError                = false;

    uint32_t gtrackStartTime;
    uint32_t benchmarkCycles;
    uint32_t benchmarks[GTRACK_BENCHMARK_SIZE + 1];

    int32_t errCode;

    binFileHeader frameHeader;
    uint32_t      fileNumber;
    size_t        result;

    uint32_t gtick;
    uint16_t mNum;
    uint16_t tNum;
    uint16_t tCheckNum;

    uint32_t n, k;

    FILE               *fCloud;
    char                fileName[120];
    binFileTLV          tlv;
    float               distX, distY;
    bool                tidFound;
    GTRACK_boundaryBox *box;

    uint32_t programBytesUsed;

    uint32_t benchmarkPerTrack;
    uint64_t benchmarkPerTrackTotal = 0;
    uint32_t benchmarkPerTrackCount = 0;
    uint32_t benchmarkPerTrackAve;
    uint32_t benchmarkPerTrackMax = 0;
    uint32_t benchmarkPerTrackMin = 0xFFFFFFFF;

    uint32_t cyclesPerSecond;
    uint32_t cyclesPerCCNT;
    float    uSecondsPerCycle;

    memset((void *)&config, 0, sizeof(GTRACK_moduleConfig));
#ifdef GTRACK_3D
    printf("Gtrack is configured for 3D\n");
    config.stateVectorType = GTRACK_STATE_VECTORS_3DA; // Track three dimensions with acceleration
#else
    printf("Gtrack is configured for 2D\n");
    config.stateVectorType = GTRACK_STATE_VECTORS_2DA; // Track two dimensions with acceleration
#endif
    config.verbose                  = GTRACK_VERBOSE_NONE;
    config.deltaT                   = 0.05f; // 50ms frames
    config.maxRadialVelocity        = 5.29f; // Radial velocity from sensor is limited to +/- maxURV (in m/s)
    config.radialVelocityResolution = 0.083f; // Radial velocity resolution (in m/s)
    config.maxAcceleration[0]       = 0.1f; // Target acceleration is not excedding 0.5 m/s2 in lateral direction
    config.maxAcceleration[1]       = 0.1f; // Target acceleration is not excedding 0.5 m/s2 in longitudinal direction
    config.maxAcceleration[2]       = 0.1f; // Target acceleration is not excedding 0.5 m/s2 in vertical direction
    config.maxNumPoints             = 250;
    config.maxNumTracks             = 20;
    config.initialRadialVelocity    = 0; // Expected target radial velocity at the moment of detection, m/s


    /* Here, we want to set allocation, gating, and threshold parameters, leaving the rest to default */
    memset((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
    advParams.allocationParams = &appAllocationParams;
    advParams.gatingParams     = &appGatingParams;
    advParams.stateParams      = &appStateParams;
    advParams.sceneryParams    = &appSceneryParams;
    advParams.variationParams  = &appVariationParams;

    config.advParams = &advParams;

    printf("Tracker Configuration\n");
    printf("\tstateVectorType: %d\n", config.stateVectorType);
    printf("\tmaxNumPoints: %d\n", config.maxNumPoints);
    printf("\tmaxNumTracks: %d\n", config.maxNumTracks);
    printf("\tmaxRadialVelocity: %f\n", config.maxRadialVelocity);
    printf("\tradialVelocityResolution: %f\n", config.radialVelocityResolution);
    printf("\tdeltaT: %f\n", config.deltaT);

    printf("\tinitialRadialVelocity: %f\n", config.initialRadialVelocity);
    printf("\tmaxAcceleration: [%f, %f, %f]\n", config.maxAcceleration[0], config.maxAcceleration[1], config.maxAcceleration[2]);

    printf("\tscenery:\n");
    for (n = 0; n < config.advParams->sceneryParams->numBoundaryBoxes; n++)
    {
        box = &config.advParams->sceneryParams->boundaryBox[n];
        printf("\t\t BoundaryBox %d: [%f, %f, %f, %f, %f, %f]\n", n, box->x1, box->x2, box->y1, box->y2, box->z1, box->z2);
    }
    for (n = 0; n < config.advParams->sceneryParams->numBoundaryBoxes; n++)
    {
        box = &config.advParams->sceneryParams->staticBox[n];
        printf("\t\t StaticBox %d: [%f, %f, %f, %f, %f, %f]\n", n, box->x1, box->x2, box->y1, box->y2, box->z1, box->z2);
    }
    printf("\tallocation: [%f, %f, %f, %d, %f, %f]\n", config.advParams->allocationParams->snrThre, config.advParams->allocationParams->snrThreObscured, config.advParams->allocationParams->velocityThre, config.advParams->allocationParams->pointsThre, config.advParams->allocationParams->maxDistanceThre, config.advParams->allocationParams->maxVelThre);
    printf("\tgating: [%f, %f, %f, %f, %f]\n", config.advParams->gatingParams->gain, config.advParams->gatingParams->limitsArray[0], config.advParams->gatingParams->limitsArray[1], config.advParams->gatingParams->limitsArray[2], config.advParams->gatingParams->limitsArray[3]);
    printf("\tthresholds: [%d, %d, %d, %d, %d]\n", config.advParams->stateParams->det2actThre, config.advParams->stateParams->det2freeThre, config.advParams->stateParams->active2freeThre, config.advParams->stateParams->static2freeThre, config.advParams->stateParams->exit2freeThre);

    hTrackModule = gtrack_create(&config, &errCode);

    uSecondsPerCycle = 1000000.f / cyclesPerSecond;

    gtrackStartTime = gtrack_getCycleCount();
    cyclesPerCCNT   = gtrack_getCycleCount() - gtrackStartTime;
    printf("Calibration: CCNT read is %u Cycles\n", cyclesPerCCNT);


    fileNumber = 1;

    while (1)
    {
#ifdef GTRACK_3D
        sprintf(fileName, "../usecases/people_counting/fHistScene4_3d_%04u.bin", fileNumber);
#else
        sprintf(fileName, "../usecases/people_counting/fHistScene4_2d_%04u.bin", fileNumber);
#endif
        fCloud = fopen(fileName, "rb");
        if (fCloud == 0)
            break;

        if (fileNumber == 1)
        {
            performanceTestResultPass = true;
        }

        while (1)
        {
            // Read frame Header
            result = fread(&frameHeader, sizeof(frameHeader), 1, fCloud);
            if (result != 1)
                break;

            frameError = false;
            mNum       = 0;
            tCheckNum  = 0;

            for (n = 0; n < frameHeader.numTLVs; n++)
            {
                result = fread(&tlv, 8, 1, fCloud);
                if (result != 1)
                    break;

                switch (tlv.type)
                {
                    case 6:
                        result = fread(pointCloud, tlv.length, 1, fCloud);
                        if (result != 1)
                            break;

                        mNum = (uint16_t)tlv.length / sizeof(GTRACK_measurementPoint);
                        break;

                    case 7:
                        result = fread(checkDescr, tlv.length, 1, fCloud);
                        if (result != 1)
                            break;
                        tCheckNum = (uint16_t)tlv.length / sizeof(checkTargetDescr);
                        break;

                    default:
                        break;
                }
            }

            gtick = frameHeader.frameNum;

            // Limit the points
            if (mNum > config.maxNumPoints)
                mNum = config.maxNumPoints;

            gtrackStartTime = gtrack_getCycleCount();
            gtrack_step(hTrackModule, pointCloud, 0, mNum, targetDescr, &tNum, 0, benchmarks);
            if (tCheckNum != tNum)
            {
                printf("Error, Frame #%u: missmatched number of targets\n", gtick);
                performanceTestResultPass = false;
                frameError                = true;
            }
            for (n = 0; n < tNum; n++)
            {
                tidFound = false;
                for (k = 0; k < tNum; k++)
                {
                    if (targetDescr[n].uid == checkDescr[k].tid)
                    {
                        tidFound = true;
                        break;
                    }
                }

                if (tidFound == false)
                {
                    printf("Error, Frame #%u: tid not found\n", gtick);
                    performanceTestResultPass = false;
                    frameError                = true;
                    break;
                }

                distX = targetDescr[n].S[0] - checkDescr[k].S[0];
                distY = targetDescr[n].S[1] - checkDescr[k].S[1];
                if (sqrt(distX * distX + distY * distY) > 2)
                {
                    printf("Error, Frame #%u: missmatched distance\n", gtick);
                    performanceTestResultPass = false;
                    frameError                = true;
                    break;
                }
            }

            benchmarkCycles = gtrack_getCycleCount() - gtrackStartTime;

            if (tNum)
            {
                benchmarkPerTrack = (uint32_t)benchmarkCycles / tNum;
                benchmarkPerTrackTotal += benchmarkPerTrack;
                benchmarkPerTrackCount += 1;

                if (benchmarkPerTrack < benchmarkPerTrackMin)
                    benchmarkPerTrackMin = benchmarkPerTrack;
                if (benchmarkPerTrack > benchmarkPerTrackMax)
                    benchmarkPerTrackMax = benchmarkPerTrack;
            }
            printf("Frame #%u, %u Targets, %u Points, %u Tracks, %u Cycles = (", gtick, tCheckNum, mNum, tNum, benchmarkCycles);
            for (n = 0; n < GTRACK_BENCHMARK_SIZE; n++)
            {
                printf("%u, ", benchmarks[n] - gtrackStartTime);
                gtrackStartTime = benchmarks[n];
            }
            printf("%u)", benchmarks[n] - gtrackStartTime);

            if (frameError)
                printf("... ERROR\n");
            else
                printf("... OK\n");
        }
        fileNumber++;
        fclose(fCloud);
    }

    if (fileNumber == 1)
    {
        printf("ERROR: No valid files found\n");
    }

    gtrack_delete(hTrackModule);

    printf("**************************************************************************************************************\n");
    if (config.stateVectorType == GTRACK_STATE_VECTORS_3DA)
        printf("Tracking in 3D space\n");
    else
        printf("Tracking in 2D space\n");

    printf("Configured for %d maxPoints and %d maxTracks\n", config.maxNumPoints, config.maxNumTracks);

    if (benchmarkPerTrackCount > 0)
    {
        benchmarkPerTrackAve = (uint32_t)(benchmarkPerTrackTotal / benchmarkPerTrackCount);
        printf("Cycles, per target {mean, min, max} = %u, %u, %u\n", benchmarkPerTrackAve, benchmarkPerTrackMin, benchmarkPerTrackMax);
        printf("uSec, per target {mean, min, max} = %u, %u, %u\n", (uint32_t)(benchmarkPerTrackAve * uSecondsPerCycle), (uint32_t)(benchmarkPerTrackMin * uSecondsPerCycle), (uint32_t)(benchmarkPerTrackMax * uSecondsPerCycle));
    }
    else
    {
        printf("ERROR: No benchmark results produced\n");
    }

    /* Test Results */
    /* Performance Test */
    if (performanceTestResultPass != true)
    {
        printf("ERROR: Tracking performance does not match with reference\n");
    }
    else
        printf("PERFORMANCE TEST PASSED\n");

    return 0;
}
