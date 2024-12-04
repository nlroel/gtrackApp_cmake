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

#include <stdint.h>

#include <gtrack_listlib.h>

/**************************************************************************
 ***************************** ListLib Functions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is called to initialize the list.
 *
 *  @param[in]  list
 *      List to be initialized
 *
 *  @retval
 *      Not Applicable
 */
void gtrack_listInit(GTrack_ListObj *list)
{
    /* Initialize the fields in the list object. */
    list->count = 0;
    list->begin = 0;
    list->end   = 0;
}

/**
 *  @b Description
 *  @n
 *      The function is called to check if the list is empty or not
 *
 *  @param[in]  list
 *      List to be checked.
 *
 *  @retval
 *      1	-	List is empty
 *  @retval
 *      0	-	List
 */
int32_t gtrack_isListEmpty(GTrack_ListObj *list)
{
    if (list->count == 0)
        return 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is called to add an element to the end of the list.
 *
 *  @param[in]  list
 *      List from where the element is to be added
 *  @param[in]  elem
 *      Element to be added to the end of the list
 *
 *  @retval
 *      Not Applicable.
 */
void gtrack_listEnqueue(GTrack_ListObj *list, GTrack_ListElem *elem)
{
    if (list->begin == 0)
    {
        elem->prev = 0;
        elem->next = 0;

        list->begin = elem;
        list->end   = elem;
    }
    else
    {
        elem->prev = list->end;
        elem->next = 0;

        list->end->next = elem;
        list->end       = elem;
    }
    list->count++;
}

/**
 *  @b Description
 *  @n
 *      The function is called to dequeue an element from the head of the list.
 *
 *  @param[in]  list
 *      List from where the element is to be removed
 *
 *  @retval
 *      Head of the list (NULL if the list was empty)
 */
GTrack_ListElem *gtrack_listDequeue(GTrack_ListObj *list)
{
    GTrack_ListElem *elem;

    if (list->begin == 0)
        return 0;

    elem        = list->begin;
    list->begin = elem->next;
    list->count--;

    if (list->begin == 0)
        list->end = 0;

    elem->next = 0;
    elem->prev = 0;

    return elem;
}

/**
 *  @b Description
 *  @n
 *      The function is called to get the number of elements in the list.
 *
 *  @param[in]  list
 *      List for which the number of elements are required.
 *
 *  @retval
 *      Counter
 */
uint32_t gtrack_listGetCount(GTrack_ListObj *list)
{
    return list->count;
}
/**
 *  @b Description
 *  @n
 *      The function is called to get the first elements in the list.
 *
 *  @param[in]  list
 *      List for which the number of elements are required.
 *
 *  @retval
 *      First Element
 */
GTrack_ListElem *gtrack_listGetFirst(GTrack_ListObj *list)
{
    return list->begin;
}
/**
 *  @b Description
 *  @n
 *      The function is called to get the next elements in the list.
 *
 *  @param[in]  elem
 *      List for which the number of elements are required.
 *
 *  @retval
 *      Next Element
 */
GTrack_ListElem *gtrack_listGetNext(GTrack_ListElem *elem)
{
    return elem->next;
}

/**
 *  @b Description
 *  @n
 *      The function is called to remove the specific element from
 *      the list.
 *
 *  @param[in]  list
 *      List from which the element is to be removed.
 *  @param[in]  elem
 *      The element to be removed.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t gtrack_listRemoveElement(GTrack_ListObj *list, GTrack_ListElem *elem)
{
    if (elem->prev == 0)
    {
        if (list->begin != elem)
            return -1;

        if (elem->next == 0)
        {
            /* Next could be NULL only if that is the last element */
            if (list->end != elem)
                return -1;

            if (list->count != 1)
                return -1;

            /* That was the only element, list is emty now */
            list->begin = 0;
            list->end   = 0;
            list->count = 0;
            return 0;
        }

        if (elem->next->prev != elem)
            return -1;

        /* That was the first element, update the list begin */
        list->begin      = elem->next;
        elem->next->prev = 0;
        list->count--;
        return 0;
    }

    if (elem->prev->next != elem)
        return -1;

    if (elem->next == 0)
    {
        /* Next could be NULL only if that is the last element */
        if (list->end != elem)
            return -1;

        /* Yes, it is valid last element */
        elem->prev->next = 0;
        list->end        = elem->prev;
        list->count--;
        return 0;
    }

    if (elem->next->prev != elem)
        return -1;

    elem->prev->next = elem->next;
    elem->next->prev = elem->prev;
    list->count--;
    return 0;
}
