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
 *   This file includes definitions for forwarding IPv6 datagrams across the Thread mesh.
 */

#ifndef MESH_FORWARDER_HPP_
#define MESH_FORWARDER_HPP_

#include "openthread-core-config.h"

#include "common/locator.hpp"
#include "common/tasklet.hpp"
#include "mac/channel_mask.hpp"
#include "mac/data_poll_sender.hpp"
#include "mac/mac.hpp"
#include "net/ip6.hpp"
#include "thread/address_resolver.hpp"
#include "thread/indirect_sender.hpp"
#include "thread/lowpan.hpp"
#include "thread/network_data_leader.hpp"
#include "thread/topology.hpp"

namespace ot {

enum
{
    kReassemblyTimeout = OPENTHREAD_CONFIG_6LOWPAN_REASSEMBLY_TIMEOUT,
};

/**
 * @addtogroup core-mesh-forwarding
 *
 * @brief
 *   This module includes definitions for mesh forwarding within Thread.
 *
 * @{
 */

/**
 * This class represents an IPv6 fragment priority entry
 *
 */
class FragmentPriorityEntry
{
public:
    /**
     * This method returns the fragment datagram tag value.
     *
     * @returns The fragment datagram tag value.
     *
     */
    uint16_t GetDatagramTag(void) const { return mDatagramTag; }

    /**
     * This method sets the fragment datagram tag value.
     *
     * @param[in]  aDatagramTag  The fragment datagram tag value.
     *
     */
    void SetDatagramTag(uint16_t aDatagramTag) { mDatagramTag = aDatagramTag; }

    /**
     * This method returns the source Rloc16 of the fragment.
     *
     * @returns The source Rloc16 value.
     *
     */
    uint16_t GetSrcRloc16(void) const { return mSrcRloc16; }

    /**
     * This method sets the source Rloc16 value of the fragment.
     *
     * @param[in]  aSrcRloc16  The source Rloc16 value.
     *
     */
    void SetSrcRloc16(uint16_t aSrcRloc16) { mSrcRloc16 = aSrcRloc16; }

    /**
     * This method returns the fragment priority value.
     *
     * @returns The fragment priority value.
     *
     */
    uint8_t GetPriority(void) const { return mPriority; }

    /**
     * This method sets the fragment priority value.
     *
     * @param[in]  aPriority  The fragment priority value.
     *
     */
    void SetPriority(uint8_t aPriority) { mPriority = aPriority; }

    /**
     * This method returns the fragment priority entry's remaining lifetime.
     *
     * @returns The fragment priority entry's remaining lifetime.
     *
     */
    uint8_t GetLifetime(void) const { return mLifetime; }

    /**
     * This method sets the remaining lifetime of the fragment priority entry.
     *
     * @param[in]  aLifetime  The remaining lifetime of the fragment priority entry (in seconds).
     *
     */
    void SetLifetime(uint8_t aLifetime)
    {
        if (aLifetime > kMaxLifeTime)
        {
            aLifetime = kMaxLifeTime;
        }

        mLifetime = aLifetime;
    }

    /**
     * This method decrements the entry lifetime.
     *
     */
    void DecrementLifetime(void) { mLifetime--; }

private:
    enum
    {
        kMaxLifeTime = 5, ///< The maximum lifetime of the fragment entry (in seconds).
    };

    uint16_t mSrcRloc16;    ///< The source Rloc16 of the datagram.
    uint16_t mDatagramTag;  ///< The datagram tag of the fragment header.
    uint8_t  mPriority : 3; ///< The priority level of the first fragment.
    uint8_t  mLifetime : 3; ///< The lifetime of the entry (in seconds). 0 means the entry is invalid.
};

/**
 * This class implements mesh forwarding within Thread.
 *
 */
class MeshForwarder : public InstanceLocator
{
    friend class Mac::Mac;
    friend class Instance;
    friend class DataPollSender;
    friend class IndirectSender;

public:
    /**
     * This constructor initializes the object.
     *
     * @param[in]  aInstance     A reference to the OpenThread instance.
     *
     */
    explicit MeshForwarder(Instance &aInstance);

    /**
     * This method enables mesh forwarding and the IEEE 802.15.4 MAC layer.
     *
     */
    void Start(void);

    /**
     * This method disables mesh forwarding and the IEEE 802.15.4 MAC layer.
     *
     */
    void Stop(void);

    /**
     * This method submits a message to the mesh forwarder for forwarding.
     *
     * @param[in]  aMessage  A reference to the message.
     *
     * @retval OT_ERROR_NONE     Successfully enqueued the message.
     * @retval OT_ERROR_ALREADY  The message was already enqueued.
     * @retval OT_ERROR_DROP     The message could not be sent and should be dropped.
     *
     */
    otError SendMessage(Message &aMessage);

    /**
     * This method is called by the address resolver when an EID-to-RLOC mapping has been resolved.
     *
     * @param[in]  aEid    A reference to the EID that has been resolved.
     * @param[in]  aError  OT_ERROR_NONE on success and OT_ERROR_DROP otherwise.
     *
     */
    void HandleResolved(const Ip6::Address &aEid, otError aError);

    /**
     * This method indicates whether or not rx-on-when-idle mode is enabled.
     *
     * @retval TRUE   The rx-on-when-idle mode is enabled.
     * @retval FALSE  The rx-on-when-idle-mode is disabled.
     *
     */
    bool GetRxOnWhenIdle(void) const;

    /**
     * This method sets the rx-on-when-idle mode
     *
     * @param[in]  aRxOnWhenIdle  TRUE to enable, FALSE otherwise.
     *
     */
    void SetRxOnWhenIdle(bool aRxOnWhenIdle);

    /**
     * This method sets the scan parameters for MLE Discovery Request messages.
     *
     * @param[in]  aScanChannels  A reference to channel mask indicating which channels to scan.
     *                            If @p aScanChannels is empty, then all channels are used instead.
     *
     */
    void SetDiscoverParameters(const Mac::ChannelMask &aScanChannels);

    /**
     * This method frees any indirect messages queued for children that are no longer attached.
     *
     */
    void UpdateIndirectMessages(void);

    /**
     * This method frees any messages queued for an existing child.
     *
     * @param[in]  aChild    A reference to the child.
     * @param[in]  aSubType  The message sub-type to remove.
     *                       Use Message::kSubTypeNone remove all messages for @p aChild.
     *
     */
    void RemoveMessages(Child &aChild, uint8_t aSubType);

    /**
     * This method frees unicast/multicast MLE Data Responses from Send Message Queue if any.
     *
     */
    void RemoveDataResponseMessages(void);

    /**
     * This method evicts the message with lowest priority in the send queue.
     *
     * @param[in]  aPriority  The highest priority level of the evicted message.
     *
     * @retval OT_ERROR_NONE       Successfully evicted a low priority message.
     * @retval OT_ERROR_NOT_FOUND  No low priority messages available to evict.
     *
     */
    otError EvictMessage(uint8_t aPriority);

    /**
     * This method returns a reference to the send queue.
     *
     * @returns  A reference to the send queue.
     *
     */
    const PriorityQueue &GetSendQueue(void) const { return mSendQueue; }

    /**
     * This method returns a reference to the reassembly queue.
     *
     * @returns  A reference to the reassembly queue.
     *
     */
    const MessageQueue &GetReassemblyQueue(void) const { return mReassemblyList; }

    /**
     * This method returns a reference to the IP level counters.
     *
     * @returns A reference to the IP level counters.
     *
     */
    const otIpCounters &GetCounters(void) const { return mIpCounters; }

    /**
     * This method resets the IP level counters.
     *
     */
    void ResetCounters(void) { memset(&mIpCounters, 0, sizeof(mIpCounters)); }

#if OPENTHREAD_FTD
    /**
     * This method returns a reference to the resolving queue.
     *
     * @returns  A reference to the resolving queue.
     *
     */
    const PriorityQueue &GetResolvingQueue(void) const { return mResolvingQueue; }
#endif

private:
    enum
    {
        kStateUpdatePeriod  = 1000,                     ///< State update period in milliseconds.
        kDefaultMsgPriority = Message::kPriorityNormal, ///< Default message priority.

        /**
         * The number of fragment priority entries.
         *
         */
        kNumFragmentPriorityEntries = OPENTHREAD_CONFIG_NUM_FRAGMENT_PRIORITY_ENTRIES,
    };

    enum MessageAction ///< Defines the action parameter in `LogMessageInfo()` method.
    {
        kMessageReceive,         ///< Indicates that the message was received.
        kMessageTransmit,        ///< Indicates that the message was sent.
        kMessagePrepareIndirect, ///< Indicates that the message is being prepared for indirect tx.
        kMessageDrop,            ///< Indicates that the outbound message is being dropped (e.g., dst unknown).
        kMessageReassemblyDrop,  ///< Indicates that the message is being dropped from reassembly list.
        kMessageEvict,           ///< Indicates that the message was evicted.
    };

    void    SendIcmpErrorIfDstUnreach(const Message &     aMessage,
                                      const Mac::Address &aMacSource,
                                      const Mac::Address &aMacDest);
    otError CheckReachability(const uint8_t *     aFrame,
                              uint16_t            aFrameLength,
                              const Mac::Address &aMeshSource,
                              const Mac::Address &aMeshDest);
    void    UpdateRoutes(const uint8_t *     aFrame,
                         uint16_t            aFrameLength,
                         const Mac::Address &aMeshSource,
                         const Mac::Address &aMeshDest);

    otError  DecompressIp6Header(const uint8_t *     aFrame,
                                 uint16_t            aFrameLength,
                                 const Mac::Address &aMacSource,
                                 const Mac::Address &aMacDest,
                                 Ip6::Header &       aIp6Header,
                                 uint8_t &           aHeaderLength,
                                 bool &              aNextHeaderCompressed);
    otError  FrameToMessage(const uint8_t *     aFrame,
                            uint16_t            aFrameLength,
                            uint16_t            aDatagramSize,
                            const Mac::Address &aMacSource,
                            const Mac::Address &aMacDest,
                            Message *&          aMessage);
    otError  GetIp6Header(const uint8_t *     aFrame,
                          uint16_t            aFrameLength,
                          const Mac::Address &aMacSource,
                          const Mac::Address &aMacDest,
                          Ip6::Header &       aIp6Header);
    void     GetMacDestinationAddress(const Ip6::Address &aIp6Addr, Mac::Address &aMacAddr);
    void     GetMacSourceAddress(const Ip6::Address &aIp6Addr, Mac::Address &aMacAddr);
    Message *GetDirectTransmission(void);
    otError  PrepareDiscoverRequest(void);
    void     HandleMesh(uint8_t *               aFrame,
                        uint16_t                aFrameLength,
                        const Mac::Address &    aMacSource,
                        const otThreadLinkInfo &aLinkInfo);
    void     HandleFragment(const uint8_t *         aFrame,
                            uint16_t                aFrameLength,
                            const Mac::Address &    aMacSource,
                            const Mac::Address &    aMacDest,
                            const otThreadLinkInfo &aLinkInfo);
    void     HandleLowpanHC(const uint8_t *         aFrame,
                            uint16_t                aFrameLength,
                            const Mac::Address &    aMacSource,
                            const Mac::Address &    aMacDest,
                            const otThreadLinkInfo &aLinkInfo);
    uint16_t PrepareDataFrame(Mac::TxFrame &      aFrame,
                              Message &           aMessage,
                              const Mac::Address &aMacSource,
                              const Mac::Address &aMacDest,
                              bool                aAddMeshHeader = false,
                              uint16_t            aMeshSource    = 0xffff,
                              uint16_t            aMeshDest      = 0xffff);

    void    SendMesh(Message &aMessage, Mac::TxFrame &aFrame);
    void    SendDestinationUnreachable(uint16_t aMeshSource, const Message &aMessage);
    otError UpdateIp6Route(Message &aMessage);
    otError UpdateIp6RouteFtd(Ip6::Header &ip6Header, Message &aMessage);
    otError UpdateMeshRoute(Message &aMessage);
    bool    UpdateReassemblyList(void);
    bool    UpdateFragmentLifetime(void);
    void    UpdateFragmentPriority(Lowpan::FragmentHeader &aFragmentHeader,
                                   uint16_t                aFragmentLength,
                                   uint16_t                aSrcRloc16,
                                   uint8_t                 aPriority);
    otError HandleDatagram(Message &aMessage, const otThreadLinkInfo &aLinkInfo, const Mac::Address &aMacSource);
    void    ClearReassemblyList(void);
    void    RemoveMessage(Message &aMessage);
    void    HandleDiscoverComplete(void);

    void      HandleReceivedFrame(Mac::RxFrame &aFrame);
    otError   HandleFrameRequest(Mac::TxFrame &aFrame);
    Neighbor *UpdateNeighborOnSentFrame(Mac::TxFrame &aFrame, otError aError, const Mac::Address &aMacDest);
    void      HandleSentFrame(Mac::TxFrame &aFrame, otError aError);

    static void HandleDiscoverTimer(Timer &aTimer);
    void        HandleDiscoverTimer(void);
    static void HandleUpdateTimer(Timer &aTimer);
    void        HandleUpdateTimer(void);
    static void ScheduleTransmissionTask(Tasklet &aTasklet);
    void        ScheduleTransmissionTask(void);

    otError GetFramePriority(const uint8_t *     aFrame,
                             uint16_t            aFrameLength,
                             const Mac::Address &aMacSource,
                             const Mac::Address &aMacDest,
                             uint8_t &           aPriority);
    otError GetFragmentPriority(Lowpan::FragmentHeader &aFragmentHeader, uint16_t aSrcRloc16, uint8_t &aPriority);
    otError GetForwardFramePriority(const uint8_t *     aFrame,
                                    uint16_t            aFrameLength,
                                    const Mac::Address &aMeshSource,
                                    const Mac::Address &aMeshDest,
                                    uint8_t &           aPriority);

    FragmentPriorityEntry *FindFragmentPriorityEntry(uint16_t aTag, uint16_t aSrcRloc16);
    FragmentPriorityEntry *GetUnusedFragmentPriorityEntry(void);

    otError GetDestinationRlocByServiceAloc(uint16_t aServiceAloc, uint16_t &aMeshDest);

    void LogMessage(MessageAction aAction, const Message &aMessage, const Mac::Address *aAddress, otError aError);
    void LogFrame(const char *aActionText, const Mac::Frame &aFrame, otError aError);
    void LogFragmentFrameDrop(otError                       aError,
                              uint16_t                      aFrameLength,
                              const Mac::Address &          aMacSource,
                              const Mac::Address &          aMacDest,
                              const Lowpan::FragmentHeader &aFragmentHeader,
                              bool                          aIsSecure);
    void LogLowpanHcFrameDrop(otError             aError,
                              uint16_t            aFrameLength,
                              const Mac::Address &aMacSource,
                              const Mac::Address &aMacDest,
                              bool                aIsSecure);

#if (OPENTHREAD_CONFIG_LOG_LEVEL >= OT_LOG_LEVEL_NOTE) && (OPENTHREAD_CONFIG_LOG_MAC == 1)
    const char *MessageActionToString(MessageAction aAction, otError aError);
    const char *MessagePriorityToString(const Message &aMessage);

    otError ParseIp6UdpTcpHeader(const Message &aMessage,
                                 Ip6::Header &  aIp6Header,
                                 uint16_t &     aChecksum,
                                 uint16_t &     aSourcePort,
                                 uint16_t &     aDestPort);
#if OPENTHREAD_FTD
    otError DecompressIp6UdpTcpHeader(const Message &     aMessage,
                                      uint16_t            aOffset,
                                      const Mac::Address &aMeshSource,
                                      const Mac::Address &aMeshDest,
                                      Ip6::Header &       aIp6Header,
                                      uint16_t &          aChecksum,
                                      uint16_t &          aSourcePort,
                                      uint16_t &          aDestPort);
    otError LogMeshFragmentHeader(MessageAction       aAction,
                                  const Message &     aMessage,
                                  const Mac::Address *aMacAddress,
                                  otError             aError,
                                  uint16_t &          aOffset,
                                  Mac::Address &      aMeshSource,
                                  Mac::Address &      aMeshDest,
                                  otLogLevel          aLogLevel);
    void    LogMeshIpHeader(const Message &     aMessage,
                            uint16_t            aOffset,
                            const Mac::Address &aMeshSource,
                            const Mac::Address &aMeshDest,
                            otLogLevel          aLogLevel);
    void    LogMeshMessage(MessageAction       aAction,
                           const Message &     aMessage,
                           const Mac::Address *aAddress,
                           otError             aError,
                           otLogLevel          aLogLevel);
#endif
    void LogIp6SourceDestAddresses(Ip6::Header &aIp6Header,
                                   uint16_t     aSourcePort,
                                   uint16_t     aDestPort,
                                   otLogLevel   aLogLevel);
    void LogIp6Message(MessageAction       aAction,
                       const Message &     aMessage,
                       const Mac::Address *aAddress,
                       otError             aError,
                       otLogLevel          aLogLevel);
#endif // #if (OPENTHREAD_CONFIG_LOG_LEVEL >= OT_LOG_LEVEL_NOTE) && (OPENTHREAD_CONFIG_LOG_MAC == 1)

    TimerMilli mDiscoverTimer;
    TimerMilli mUpdateTimer;

    PriorityQueue mSendQueue;
    MessageQueue  mReassemblyList;
    uint16_t      mFragTag;
    uint16_t      mMessageNextOffset;

    Message *mSendMessage;

    Mac::Address mMacSource;
    Mac::Address mMacDest;
    uint16_t     mMeshSource;
    uint16_t     mMeshDest;
    bool         mAddMeshHeader;

    bool mSendBusy;

    Tasklet mScheduleTransmissionTask;
    bool    mEnabled;

    Mac::ChannelMask mScanChannels;
    uint8_t          mScanChannel;
    uint16_t         mRestorePanId;
    bool             mScanning;

    otIpCounters mIpCounters;

#if OPENTHREAD_FTD
    FragmentPriorityEntry mFragmentEntries[kNumFragmentPriorityEntries];
    PriorityQueue         mResolvingQueue;
    IndirectSender        mIndirectSender;
#endif

    DataPollSender mDataPollSender;
};

/**
 * @}
 *
 */

} // namespace ot

#endif // MESH_FORWARDER_HPP_
