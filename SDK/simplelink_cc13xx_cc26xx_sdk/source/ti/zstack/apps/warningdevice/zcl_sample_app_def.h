/**************************************************************************************************
  Filename:       zcl_sample_app_def.h

  Description:    Generic sample app definitions for UI used by zcl_sampleapp_ui.c/.h


  Copyright 2006-2014 Texas Instruments Incorporated.

  All rights reserved not granted herein.
  Limited License.

  Texas Instruments Incorporated grants a world-wide, royalty-free,
  non-exclusive license under copyrights and patents it now or hereafter
  owns or controls to make, have made, use, import, offer to sell and sell
  ("Utilize") this software subject to the terms herein. With respect to the
  foregoing patent license, such license is granted solely to the extent that
  any such patent is necessary to Utilize the software alone. The patent
  license shall not apply to any combinations which include this software,
  other than combinations with devices manufactured by or for TI ("TI
  Devices"). No hardware patent is licensed hereunder.

  Redistributions must preserve existing copyright notices and reproduce
  this license (including the above copyright notice and the disclaimer and
  (if applicable) source code license limitations below) in the documentation
  and/or other materials provided with the distribution.

  Redistribution and use in binary form, without modification, are permitted
  provided that the following conditions are met:

    * No reverse engineering, decompilation, or disassembly of this software
      is permitted with respect to any software provided in binary form.
    * Any redistribution and use are licensed by TI for use only with TI Devices.
    * Nothing shall obligate TI to provide you with source code for the software
      licensed and provided to you in object code.

  If software source code is provided to you, modification and redistribution
  of the source code are permitted provided that the following conditions are
  met:

    * Any redistribution and use of the source code, including any resulting
      derivative works, are licensed by TI for use only with TI Devices.
    * Any redistribution and use of any object code compiled from the source
      code and any resulting derivative works, are licensed by TI for use
      only with TI Devices.

  Neither the name of Texas Instruments Incorporated nor the names of its
  suppliers may be used to endorse or promote products derived from this
  software without specific prior written permission.

  DISCLAIMER.

  THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
  OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************************************/

#include "zcl_samplewarningdevice.h"

#ifndef APPLICATION_UI_ZCL_SAMPLE_APP_DEF_H_
#define APPLICATION_UI_ZCL_SAMPLE_APP_DEF_H_


#ifndef CUI_DISABLE


#define SAMPLE_APP_MENUS  4

#define CUI_APP_MENU    CUI_MENU_ITEM_INT_ACTION("<ENROLLMENT MODE >", (CUI_pFnIntercept_t) zclSampleWarningDevice_UiActionChangeEnrollmentMode) \
                        CUI_MENU_ITEM_INT_ACTION("<  DISCOVER CIE  >", (CUI_pFnIntercept_t) zclSampleWarningDevice_UiActionDiscoverCIE) \
                        CUI_MENU_ITEM_ACTION    ("<SEND ENROLL REQ >", (CUI_pFnAction_t) zclSampleWarningDevice_UiActionSendEnroll) \
                        CUI_MENU_ITEM_ACTION    ("<  TOGGLE ALARM  >", (CUI_pFnAction_t) zclSampleWarningDevice_UiActionToggleAlarm)

#define APP_TITLE_STR "TI Sample Warning Device"

#endif  // #ifndef CUI_DISABLE


#endif /* APPLICATION_UI_ZCL_SAMPLEAPPDEF_H_ */
