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

#ifndef IP6_MPL_HPP_
#define IP6_MPL_HPP_

/**
 * @file
 *   This file includes definitions for MPL.
 */

#include "openthread-core-config.h"

#include "common/locator.hpp"
#include "common/message.hpp"
#include "common/timer.hpp"
#include "net/ip6_headers.hpp"

namespace ot {
namespace Ip6 {

/**
 * @addtogroup core-ip6-mpl
 *
 * @brief
 *   This module includes definitions for MPL.
 *
 * @{
 *
 */

/**
 * This class implements MPL header generation and parsing.
 *
 */
OT_TOOL_PACKED_BEGIN
class OptionMpl : public OptionHeader
{
public:
    enum
    {
        kType      = 0x6d, /* 01 1 01101 */
        kMinLength = 2
    };

    /**
     * This method initializes the MPL header.
     *
     */
    void Init(void)
    {
        OptionHeader::SetType(kType);
        OptionHeader::SetLength(sizeof(*this) - sizeof(OptionHeader));
        mControl = 0;
    }

    /**
     * This method returns the total MPL Option length value including option
     * header.
     *
     * @returns The total IPv6 Option Length.
     *
     */
    uint8_t GetTotalLength(void) const { return OptionHeader::GetLength() + sizeof(OptionHeader); }

    /**
     * MPL Seed Id lengths.
     */
    enum SeedIdLength
    {
        kSeedIdLength0  = 0 << 6, ///< 0-byte MPL Seed Id Length.
        kSeedIdLength2  = 1 << 6, ///< 2-byte MPL Seed Id Length.
        kSeedIdLength8  = 2 << 6, ///< 8-byte MPL Seed Id Length.
        kSeedIdLength16 = 3 << 6, ///< 16-byte MPL Seed Id Length.
    };

    /**
     * This method returns the MPL Seed Id Length value.
     *
     * @returns The MPL Seed Id Length value.
     *
     */
    SeedIdLength GetSeedIdLength(void) { return static_cast<SeedIdLength>(mControl & kSeedIdLengthMask); }

    /**
     * This method sets the MPL Seed Id Length value.
     *
     * @param[in]  aSeedIdLength  The MPL Seed Length.
     *
     */
    void SetSeedIdLength(SeedIdLength aSeedIdLength)
    {
        mControl = static_cast<uint8_t>((mControl & ~kSeedIdLengthMask) | aSeedIdLength);
    }

    /**
     * This method indicates whether or not the MPL M flag is set.
     *
     * @retval TRUE   If the MPL M flag is set.
     * @retval FALSE  If the MPL M flag is not set.
     *
     */
    bool IsMaxFlagSet(void) const { return (mControl & kMaxFlag) != 0; }

    /**
     * This method clears the MPL M flag.
     *
     */
    void ClearMaxFlag(void) { mControl &= ~kMaxFlag; }

    /**
     * This method sets the MPL M flag.
     *
     */
    void SetMaxFlag(void) { mControl |= kMaxFlag; }

    /**
     * This method returns the MPL Sequence value.
     *
     * @returns The MPL Sequence value.
     *
     */
    uint8_t GetSequence(void) const { return mSequence; }

    /**
     * This method sets the MPL Sequence value.
     *
     * @param[in]  aSequence  The MPL Sequence value.
     *
     */
    void SetSequence(uint8_t aSequence) { mSequence = aSequence; }

    /**
     * This method returns the MPL Seed Id value.
     *
     * @returns The MPL Seed Id value.
     *
     */
    uint16_t GetSeedId(void) const { return HostSwap16(mSeedId); }

    /**
     * This method sets the MPL Seed Id value.
     *
     * @param[in]  aSeedId  The MPL Seed Id value.
     *
     */
    void SetSeedId(uint16_t aSeedId) { mSeedId = HostSwap16(aSeedId); }

private:
    enum
    {
        kSeedIdLengthMask = 3 << 6,
        kMaxFlag          = 1 << 5
    };
    uint8_t  mControl;
    uint8_t  mSequence;
    uint16_t mSeedId;
} OT_TOOL_PACKED_END;

/**
 * This class implements MPL message processing.
 *
 */
class Mpl : public InstanceLocator
{
public:
    /**
     * This constructor initializes the MPL object.
     *
     * @param[in]  aInstance  A reference to the OpenThread instance.
     *
     */
    explicit Mpl(Instance &aInstance);

    /**
     * This method initializes the MPL option.
     *
     * @param[in]  aOption   A reference to the MPL header to initialize.
     * @param[in]  aAddress  A reference to the IPv6 Source Address.
     *
     */
    void InitOption(OptionMpl &aOption, const Address &aAddress);

    /**
     * This method processes an MPL option. When the MPL module acts as an MPL Forwarder
     * it disseminates MPL Data Message using Trickle timer expirations. When acts as an
     * MPL Seed it allows to send the first MPL Data Message directly, then sets up Trickle
     * timer expirations for subsequent retransmissions.
     *
     * @param[in]  aMessage    A reference to the message.
     * @param[in]  aAddress    A reference to the IPv6 Source Address.
     * @param[in]  aIsOutbound TRUE if this message was locally generated, FALSE otherwise.
     *
     * @retval OT_ERROR_NONE  Successfully processed the MPL option.
     * @retval OT_ERROR_DROP  The MPL message is a duplicate and should be dropped.
     *
     */
    otError ProcessOption(Message &aMessage, const Address &aAddress, bool aIsOutbound);

    /**
     * This method returns the MPL Seed Id value.
     *
     * @returns The MPL Seed Id value.
     *
     */
    uint16_t GetSeedId(void) const { return mSeedId; }

    /**
     * This method sets the MPL Seed Id value.
     *
     * @param[in]  aSeedId  The MPL Seed Id value.
     *
     */
    void SetSeedId(uint16_t aSeedId) { mSeedId = aSeedId; }

    /**
     * This method sets the IPv6 matching address, that allows to elide MPL Seed Id.
     *
     * @param[in] aAddress The reference to the IPv6 matching address.
     *
     */
    void SetMatchingAddress(const Address &aAddress) { mMatchingAddress = &aAddress; }

#if OPENTHREAD_FTD
    /**
     * This method gets the MPL number of Trickle timer expirations that occur before
     * terminating the Trickle algorithm's retransmission of a given MPL Data Message.
     *
     * @returns The MPL number of Trickle timer expirations.
     *
     */
    uint8_t GetTimerExpirations(void) const { return mTimerExpirations; }

    /**
     * This method sets the MPL number of Trickle timer expirations that occur before
     * terminating the Trickle algorithm's retransmission of a given MPL Data Message.
     *
     * @param[in]  aTimerExpirations  The number of Trickle timer expirations.
     *
     */
    void SetTimerExpirations(uint8_t aTimerExpirations) { mTimerExpirations = aTimerExpirations; }

    /**
     * This method returns a reference to the buffered message set.
     *
     * @returns A reference to the buffered message set.
     *
     */
    const MessageQueue &GetBufferedMessageSet(void) const { return mBufferedMessageSet; }
#endif // OPENTHREAD_FTD

private:
    enum
    {
        kNumSeedEntries      = OPENTHREAD_CONFIG_MPL_SEED_SET_ENTRIES,
        kSeedEntryLifetime   = OPENTHREAD_CONFIG_MPL_SEED_SET_ENTRY_LIFETIME,
        kSeedEntryLifetimeDt = 1000,
        kDataMessageInterval = 64
    };

    struct SeedEntry
    {
        uint16_t mSeedId;
        uint8_t  mSequence;
        uint8_t  mLifetime;
    };

    static void HandleSeedSetTimer(Timer &aTimer);
    void        HandleSeedSetTimer(void);

    otError UpdateSeedSet(uint16_t aSeedId, uint8_t aSequence);

    SeedEntry      mSeedSet[kNumSeedEntries];
    const Address *mMatchingAddress;
    TimerMilli     mSeedSetTimer;
    uint16_t       mSeedId;
    uint8_t        mSequence;

#if OPENTHREAD_FTD
    struct Metadata
    {
        otError AppendTo(Message &aMessage) const { return aMessage.Append(this, sizeof(*this)); }
        void    ReadFrom(const Message &aMessage);
        void    RemoveFrom(Message &aMessage) const;
        int     UpdateIn(Message &aMessage) const;
        void    GenerateNextTransmissionTime(TimeMilli aCurrentTime, uint8_t aInterval);

        TimeMilli mTransmissionTime;
        uint16_t  mSeedId;
        uint8_t   mSequence;
        uint8_t   mTransmissionCount;
        uint8_t   mIntervalOffset;
    };

    static void HandleRetransmissionTimer(Timer &aTimer);
    void        HandleRetransmissionTimer(void);

    void AddBufferedMessage(Message &aMessage, uint16_t aSeedId, uint8_t aSequence, bool aIsOutbound);

    MessageQueue mBufferedMessageSet;
    TimerMilli   mRetransmissionTimer;
    uint8_t      mTimerExpirations;
#endif // OPENTHREAD_FTD
};

/**
 * @}
 *
 */

} // namespace Ip6
} // namespace ot

#endif // IP6_MPL_HPP_
