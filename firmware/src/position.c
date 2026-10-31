/* Copyright (c) 2026 Tunuino. */
#include "tunuino/position.h"
#include <stddef.h>
#include <string.h>

static void clear_cam(tunuino_position *p) {
    p->cam_candidate = p->cycle_valid = p->revolutions_since_cam = 0;
    memset(&p->cam,0,sizeof(p->cam));
}
static tunuino_position_status update(tunuino_position *p, uint64_t now,
    int crank_edge, tunuino_crank_reading *crank) {
    tunuino_crank_status status = tunuino_crank_read(&p->crank,now,crank);
    if (status != TUNUINO_CRANK_SYNCED) {
        p->crank_valid = 0; clear_cam(p);
        p->state = status == TUNUINO_CRANK_CLOCK ? TUNUINO_POSITION_CLOCK
            : status == TUNUINO_CRANK_ARGUMENT ? TUNUINO_POSITION_ARGUMENT
            : status == TUNUINO_CRANK_LOST ? TUNUINO_POSITION_CRANK_LOST : TUNUINO_POSITION_ACQUIRING;
        return p->state;
    }
    if (!p->crank_valid) {
        p->crank_valid = 1; clear_cam(p); p->state = TUNUINO_POSITION_CRANK_ONLY;
    } else if (crank_edge && crank->tooth_index == 0 && p->cam_candidate) {
        ++p->revolutions_since_cam;
    }
    if (p->cam_candidate && (p->revolutions_since_cam > 2
        || (p->revolutions_since_cam == 2 && crank->tooth_index > p->config.cam_window_last))) {
        clear_cam(p); p->state = TUNUINO_POSITION_CAM_LOST;
    }
    return p->state;
}
tunuino_position_status tunuino_position_init(tunuino_position *p,
    const tunuino_position_config *config, uint64_t now) {
    if (p == NULL) { return TUNUINO_POSITION_ARGUMENT; }
    memset(p,0,sizeof(*p)); p->state = TUNUINO_POSITION_ARGUMENT;
    if (config == NULL || config->cam_revolution_phase > 1
        || config->cam_window_first > config->cam_window_last
        || config->crank.slots <= config->crank.missing
        || config->cam_window_last >= config->crank.slots - config->crank.missing) { return p->state; }
    if (tunuino_crank_init(&p->crank,&config->crank,now) != TUNUINO_CRANK_ACQUIRING) { return p->state; }
    p->config = *config; p->configured = 1; p->state = TUNUINO_POSITION_ACQUIRING;
    return p->state;
}
tunuino_position_status tunuino_position_poll(tunuino_position *p, uint64_t now) {
    if (p == NULL || !p->configured) { return TUNUINO_POSITION_ARGUMENT; }
    tunuino_crank_reading crank;
    return update(p,now,0,&crank);
}
tunuino_position_status tunuino_position_crank_edge(tunuino_position *p, uint64_t now) {
    if (p == NULL || !p->configured) { return TUNUINO_POSITION_ARGUMENT; }
    (void)tunuino_crank_edge(&p->crank,now);
    tunuino_crank_reading crank;
    return update(p,now,1,&crank);
}
tunuino_position_status tunuino_position_cam_edge(tunuino_position *p, uint64_t now) {
    if (p == NULL || !p->configured) { return TUNUINO_POSITION_ARGUMENT; }
    tunuino_crank_reading crank;
    (void)update(p,now,0,&crank);
    if (!crank.valid) { return p->state; }
    if (crank.tooth_index < p->config.cam_window_first || crank.tooth_index > p->config.cam_window_last
        || (p->cam_candidate && p->revolutions_since_cam != 2)) {
        clear_cam(p); p->state = TUNUINO_POSITION_CAM_LOST;
        return p->state;
    }
    p->cycle_valid = p->cam_candidate;
    p->cam_candidate = 1; p->revolutions_since_cam = 0;
    p->state = p->cycle_valid ? TUNUINO_POSITION_SYNCED : TUNUINO_POSITION_CRANK_ONLY;
    memset(&p->cam,0,sizeof(p->cam));
    if (p->cycle_valid && p->crank.period_us > 0) {
        double fraction = (double)(now - p->crank.last_edge_us) / p->crank.period_us;
        unsigned width = crank.tooth_index + 1u == (unsigned)p->config.crank.slots - p->config.crank.missing
            ? p->config.crank.missing + 1u : 1u;
        /* Interpolate only within the expected next physical tooth. Never
         * extrapolate a late capture across an unobserved crank edge. */
        if (fraction < width) {
            double angle = (crank.tooth_index + fraction) * 36000.0 / p->config.crank.slots
                + p->config.cam_revolution_phase * 36000.0;
            p->cam = (tunuino_cam_reading){now,(uint32_t)(angle + 0.5) % 72000u,1};
        }
    }
    return p->state;
}
tunuino_position_status tunuino_position_cam_read(tunuino_position *p, uint64_t now,
    tunuino_cam_reading *reading) {
    if (reading == NULL) return TUNUINO_POSITION_ARGUMENT;
    memset(reading,0,sizeof(*reading));
    tunuino_position_reading position;
    tunuino_position_status status = tunuino_position_read(p,now,&position);
    if (position.cycle_valid) *reading = p->cam;
    return status;
}
tunuino_position_status tunuino_position_read(tunuino_position *p, uint64_t now,
    tunuino_position_reading *reading) {
    if (reading == NULL) { return TUNUINO_POSITION_ARGUMENT; }
    memset(reading,0,sizeof(*reading));
    if (p == NULL || !p->configured) { return TUNUINO_POSITION_ARGUMENT; }
    tunuino_crank_reading crank;
    (void)update(p,now,0,&crank);
    if (crank.valid) {
        reading->crank_valid = 1; reading->tooth_index = crank.tooth_index; reading->rpm = crank.rpm;
        reading->cycle_valid = p->cycle_valid;
        if (p->cycle_valid) {
            reading->revolution_phase = p->config.cam_revolution_phase ^ (p->revolutions_since_cam & 1u);
        }
    }
    return p->state;
}
