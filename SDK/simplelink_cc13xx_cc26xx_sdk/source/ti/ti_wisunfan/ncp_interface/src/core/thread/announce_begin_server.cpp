/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the Announce Begin Server.
 */

#include "announce_begin_server.hpp"

#include <openthread/platform/radio.h>

#include "coap/coap_message.hpp"
#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "meshcop/meshcop_tlvs.hpp"
#include "thread/thread_netif.hpp"
#include "thread/thread_uri_paths.hpp"

namespace ot {

AnnounceBeginServer::AnnounceBeginServer(Instance &aInstance)
    : AnnounceSenderBase(aInstance, &AnnounceBeginServer::HandleTimer)
    , mAnnounceBegin(OT_URI_PATH_ANNOUNCE_BEGIN, &AnnounceBeginServer::HandleRequest, this)
{
    Get<Coap::Coap>().AddResource(mAnnounceBegin);
}

otError AnnounceBeginServer::SendAnnounce(uint32_t aChannelMask)
{
    return SendAnnounce(aChannelMask, kDefaultCount, kDefaultPeriod);
}

otError AnnounceBeginServer::SendAnnounce(uint32_t aChannelMask, uint8_t aCount, uint16_t aPeriod)
{
    return AnnounceSenderBase::SendAnnounce(Mac::ChannelMask(aChannelMask), aCount, aPeriod, kDefaultJitter);
}

void AnnounceBeginServer::HandleRequest(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    static_cast<AnnounceBeginServer *>(aContext)->HandleRequest(*static_cast<Coap::Message *>(aMessage),
                                                                *static_cast<const Ip6::MessageInfo *>(aMessageInfo));
}

void AnnounceBeginServer::HandleRequest(Coap::Message &aMessage, const Ip6::MessageInfo &aMessageInfo)
{
    uint32_t         mask;
    uint8_t          count;
    uint16_t         period;
    Ip6::MessageInfo responseInfo(aMessageInfo);

    VerifyOrExit(aMessage.GetCode() == OT_COAP_CODE_POST, OT_NOOP);
    VerifyOrExit((mask = MeshCoP::ChannelMaskTlv::GetChannelMask(aMessage)) != 0, OT_NOOP);

    SuccessOrExit(Tlv::ReadUint8Tlv(aMessage, MeshCoP::Tlv::kCount, count));
    SuccessOrExit(Tlv::ReadUint16Tlv(aMessage, MeshCoP::Tlv::kPeriod, period));

    SendAnnounce(mask, count, period);

    if (aMessage.IsConfirmable() && !aMessageInfo.GetSockAddr().IsMulticast())
    {
        SuccessOrExit(Get<Coap::Coap>().SendEmptyAck(aMessage, responseInfo));
        otLogInfoMeshCoP("sent announce begin response");
    }

exit:
    return;
}

void AnnounceBeginServer::HandleTimer(Timer &aTimer)
{
    aTimer.GetOwner<AnnounceBeginServer>().AnnounceSenderBase::HandleTimer();
}

} // namespace ot
