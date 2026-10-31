/* Copyright (c) 2026 Tunuino. */
#ifndef TUNUINO_CONFIG_JOURNAL_H
#define TUNUINO_CONFIG_JOURNAL_H
#include <stdbool.h>
#include "tunuino/device.h"
#define TUNUINO_JOURNAL_BANK_BYTES (128u*1024u)
#define TUNUINO_JOURNAL_RECORD_BYTES (TUNUINO_IO_SETTINGS_SIZE+32u)
typedef struct {
    bool (*read)(void *user,unsigned bank,size_t offset,uint8_t *bytes,size_t length);
    bool (*program)(void *user,unsigned bank,size_t offset,const uint8_t *bytes,size_t length);
    bool (*erase)(void *user,unsigned bank);
    void *user;
} tunuino_journal_io;
typedef struct {
    tunuino_journal_io io;
    uint32_t sequence;
    unsigned bank;
    size_t offset;
    bool found;
    bool legacy;
} tunuino_config_journal;
bool tunuino_journal_init(tunuino_config_journal *journal,const tunuino_journal_io *io,uint8_t settings[TUNUINO_IO_SETTINGS_SIZE]);
bool tunuino_journal_save(tunuino_config_journal *journal,const uint8_t settings[TUNUINO_IO_SETTINGS_SIZE]);
#endif
