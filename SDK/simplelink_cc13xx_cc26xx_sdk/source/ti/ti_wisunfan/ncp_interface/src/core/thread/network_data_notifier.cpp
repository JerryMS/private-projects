/*
 *  Copyright (c) 2020, The OpenThread Authors.
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
 *   This file implements transmissions of SVR_DATA.ntf messages to the Leader.
 */

#include "network_data_notifier.hpp"

#if OPENTHREAD_FTD || OPENTHREAD_CONFIG_BORDER_ROUTER_ENABLE || OPENTHREAD_CONFIG_TMF_NETDATA_SERVICE_ENABLE

#include "common/code_utils.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "thread/network_data_leader.hpp"
#include "thread/network_data_local.hpp"

namespace ot {
namespace NetworkData {

Notifier::Notifier(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mNotifierCallback(aInstance, &Notifier::HandleStateChanged, this)
    , mTimer(aInstance, &Notifier::HandleTimer, this)
    , mNextDelay(0)
    , mWaitingForResponse(false)
{
}

void Notifier::HandleServerDataUpdated(void)
{
    mNextDelay = 0;
    SynchronizeServerData();
}

void Notifier::SynchronizeServerData(void)
{
    otError error = OT_ERROR_NOT_FOUND;

    VerifyOrExit(Get<Mle::MleRouter>().IsAttached() && !mWaitingForResponse, OT_NOOP);

    VerifyOrExit((mNextDelay == 0) || !mTimer.IsRunning(), OT_NOOP);

#if OPENTHREAD_FTD
    mNextDelay = kDelayRemoveStaleChildren;
    error      = Get<Leader>().RemoveStaleChildEntries(&Notifier::HandleCoapResponse, this);
    VerifyOrExit(error == OT_ERROR_NOT_FOUND, OT_NOOP);
#endif

#if OPENTHREAD_CONFIG_BORDER_ROUTER_ENABLE || OPENTHREAD_CONFIG_TMF_NETDATA_SERVICE_ENABLE
    mNextDelay = kDelaySynchronizeServerData;
    error      = Get<Local>().UpdateInconsistentServerData(&Notifier::HandleCoapResponse, this);
    VerifyOrExit(error == OT_ERROR_NOT_FOUND, OT_NOOP);
#endif

exit:
    switch (error)
    {
    case OT_ERROR_NONE:
        mWaitingForResponse = true;
        break;
    case OT_ERROR_NO_BUFS:
        mTimer.Start(kDelayNoBufs);
        break;
#if OPENTHREAD_FTD
    case OT_ERROR_INVALID_STATE:
        mTimer.Start(Time::SecToMsec(Get<Mle::MleRouter>().GetRouterSelectionJitterTimeout() + 1));
        break;
#endif
    case OT_ERROR_NOT_FOUND:
        break;
    default:
        OT_ASSERT(false);
        OT_UNREACHABLE_CODE(break);
    }
}

void Notifier::HandleStateChanged(ot::Notifier::Callback &aCallback, otChangedFlags aFlags)
{
    aCallback.GetOwner<Notifier>().HandleStateChanged(aFlags);
}

void Notifier::HandleStateChanged(otChangedFlags aFlags)
{
    if (aFlags & (OT_CHANGED_THREAD_ROLE | OT_CHANGED_THREAD_CHILD_REMOVED))
    {
        mNextDelay = 0;
    }

    if (aFlags & (OT_CHANGED_THREAD_NETDATA | OT_CHANGED_THREAD_ROLE | OT_CHANGED_THREAD_CHILD_REMOVED))
    {
        SynchronizeServerData();
    }
}

void Notifier::HandleTimer(Timer &aTimer)
{
    aTimer.GetOwner<Notifier>().HandleTimer();
}

void Notifier::HandleTimer(void)
{
    SynchronizeServerData();
}

void Notifier::HandleCoapResponse(void *               aContext,
                                  otMessage *          aMessage,
                                  const otMessageInfo *aMessageInfo,
                                  otError              aResult)
{
    OT_UNUSED_VARIABLE(aMessage);
    OT_UNUSED_VARIABLE(aMessageInfo);

    static_cast<Notifier *>(aContext)->HandleCoapResponse(aResult);
}

void Notifier::HandleCoapResponse(otError aResult)
{
    mWaitingForResponse = false;

    switch (aResult)
    {
    case OT_ERROR_NONE:
        mTimer.Start(mNextDelay + 1);
        break;

    case OT_ERROR_RESPONSE_TIMEOUT:
    case OT_ERROR_ABORT:
        SynchronizeServerData();
        break;

    default:
        OT_ASSERT(false);
        OT_UNREACHABLE_CODE(break);
    }
}

} // namespace NetworkData
} // namespace ot

#endif // OPENTHREAD_FTD || OPENTHREAD_CONFIG_BORDER_ROUTER_ENABLE || OPENTHREAD_CONFIG_TMF_NETDATA_SERVICE_ENABLE
