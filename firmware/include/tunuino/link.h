/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_LINK_H
#define TUNUINO_LINK_H
#include "tunuino/dispatch.h"
#include "tunuino/stream_rx.h"
#include "tunuino/stream_tx.h"
#include "tunuino/live_mailbox.h"
#include <stdbool.h>
typedef enum {
    TUNUINO_LINK_RECEIVING = 0, TUNUINO_LINK_RESPONDING,
    TUNUINO_LINK_ARGUMENT, TUNUINO_LINK_RX_ERROR,
    TUNUINO_LINK_DISPATCH_ERROR, TUNUINO_LINK_TX_ERROR, TUNUINO_LINK_CLOSED
} tunuino_link_status;
/* Caller-owned, serialized operations. Initialize before use. Fields private
 * except state/rx_status/dispatch_status/tx_status (read-only diagnostics).
 * Context pointers must stay valid; synchronize providers during feed/dispatch.
 * No arguments or input/output buffers may overlap this object. */
typedef struct {
    tunuino_stream_rx rx;
    tunuino_stream_tx tx;
    tunuino_dispatch_context context;
    tunuino_link_status state;
    tunuino_rx_status rx_status;
    tunuino_dispatch_status dispatch_status;
    tunuino_tx_status tx_status;
    tunuino_live_mailbox *live_source;
    tunuino_live_sample live_snapshot;
    uint32_t live_max_age_ms;
    bool plain_response;
} tunuino_link;
/* Only on a fresh/clean transport, after canceling/draining old driver use. */
tunuino_link_status tunuino_link_init(tunuino_link *link, const tunuino_dispatch_context *context,
    uint32_t rx_timeout_ms, uint32_t tx_timeout_ms, uint64_t now_ms);
/* Optional acquisition mailbox. context.live is ignored; all other providers
 * and capabilities retain their existing meaning. The link owns the mailbox's
 * single reader until transport use stops; never read it from another task.
 * Refreshes an internal live copy immediately before dispatching each complete
 * request, using now_ms in the acquisition clock's time domain. Age >= the
 * positive max_age_ms yields NOT_READY for live requests. Identity/maps remain
 * usable. Publication cannot cancel a response already built for transmission.
 * Mailbox and providers must outlive the link. No arguments may overlap link.
 * Re-init/unbinding requires stopped transport use, as with ordinary init. */
tunuino_link_status tunuino_link_init_mailbox(tunuino_link *link,
    const tunuino_dispatch_context *context, tunuino_live_mailbox *mailbox,
    uint32_t max_age_ms, uint32_t rx_timeout_ms, uint32_t tx_timeout_ms, uint64_t now_ms);
/* Consumes at most one request. While responding, consumes zero bytes. Caller
 * retains remaining input until the response is completely transmitted. */
tunuino_link_status tunuino_link_feed(tunuino_link *link, const uint8_t *bytes,
    size_t length, uint64_t now_ms, size_t *consumed);
tunuino_link_status tunuino_link_poll(tunuino_link *link, uint64_t now_ms);
/* At most one driver transfer outstanding; advance only on its completion. */
tunuino_link_status tunuino_link_tx_view(tunuino_link *link, uint64_t now_ms,
    const uint8_t **bytes, size_t *remaining);
tunuino_link_status tunuino_link_tx_advance(tunuino_link *link, size_t completed, uint64_t now_ms);
/* Stops both directions; does not cancel driver operations by itself. */
void tunuino_link_abort(tunuino_link *link);
#endif
