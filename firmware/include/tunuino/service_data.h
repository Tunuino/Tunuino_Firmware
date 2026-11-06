/* Copyright (c) 2026 Tunuino. Dedicated ECU metadata and retained statistics. */
#ifndef TUNUINO_SERVICE_DATA_H
#define TUNUINO_SERVICE_DATA_H
#include "tunuino/device.h"
#include "tunuino/engine_features.h"
#define TUNUINO_SERVICE_SIZE 512u
#define TUNUINO_SERVICE_REVISION 278u
#define TUNUINO_SERVICE_STATS 282u
typedef bool (*tunuino_service_save)(void *user,const uint8_t bytes[TUNUINO_SERVICE_SIZE]);
typedef struct {
    uint8_t current[TUNUINO_SERVICE_SIZE],persisted[TUNUINO_SERVICE_SIZE],frozen[TUNUINO_SERVICE_SIZE];
    uint32_t token,frozen_generation,cached_generation;
    uint64_t last_sample_ms;
    bool have_sample,previous_full,frozen_dirty,uncertain;
    tunuino_feature_config features;
} tunuino_service_data;
bool tunuino_service_validate(const uint8_t bytes[TUNUINO_SERVICE_SIZE]);
void tunuino_service_init(tunuino_service_data *service,const uint8_t bytes[TUNUINO_SERVICE_SIZE]);
void tunuino_service_sample(tunuino_service_data *service,const tunuino_device *device,uint64_t now_ms,uint8_t output_fault);
bool tunuino_service_pending(const tunuino_service_data *service);
void tunuino_service_prepare(const tunuino_service_data *service,uint64_t now_ms,uint8_t next[TUNUINO_SERVICE_SIZE]);
void tunuino_service_committed(tunuino_service_data *service,const uint8_t bytes[TUNUINO_SERVICE_SIZE]);
/* 0 snapshot; 1 token(U32),offset(U16),count(U8); 2 VIN rev(U32),length(U8),VIN;
 * 3 comment rev(U32),UTF8 length(U16),text; 4 reset rev(U32),"RESET"; 5 checkpoint rev(U32).
 * Only metadata/statistics are committed; no engine configuration or outputs change. */
bool tunuino_service_reply(tunuino_service_data *service,uint32_t generation,uint64_t now_ms,bool writable,
    tunuino_service_save save,void *user,const uint8_t *request,size_t length,
    uint8_t *out,size_t capacity,size_t *written);
#endif
