#
#  Copyright (c) 2016, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

include $(abs_top_nlbuild_autotools_dir)/automake/pre.am

# Always package (e.g. for 'make dist') these subdirectories.

DIST_SUBDIRS                            = \
    platform                              \
    $(NULL)

# Always build (e.g. for 'make all') these subdirectories.

SUBDIRS                                 = \
    platform                              \
    $(NULL)

openthread_headers                      = \
    backbone_router.h                     \
    backbone_router_ftd.h                 \
    border_agent.h                        \
    border_router.h                       \
    channel_manager.h                     \
    channel_monitor.h                     \
    child_supervision.h                   \
    cli.h                                 \
    coap_secure.h                         \
    coap.h                                \
    commissioner.h                        \
    crypto.h                              \
    dataset.h                             \
    dataset_ftd.h                         \
    diag.h                                \
    dns.h                                 \
    entropy.h                             \
    error.h                               \
    heap.h                                \
    icmp6.h                               \
    instance.h                            \
    ip6.h                                 \
    jam_detection.h                       \
    joiner.h                              \
    link.h                                \
    link_raw.h                            \
    logging.h                             \
    message.h                             \
    ncp.h                                 \
    netdata.h                             \
    netdiag.h                             \
    network_time.h                        \
    random_crypto.h                       \
    random_noncrypto.h                    \
    server.h                              \
    sntp.h                                \
    tasklet.h                             \
    thread.h                              \
    thread_ftd.h                          \
    udp.h                                 \
    $(NULL)

openthreaddir = $(includedir)/openthread
dist_openthread_HEADERS = $(openthread_headers)

include_HEADERS                         = \
    $(NULL)

noinst_HEADERS                          = \
    config.h                              \
    $(NULL)

install-headers: install-includeHEADERS

include $(abs_top_nlbuild_autotools_dir)/automake/post.am
