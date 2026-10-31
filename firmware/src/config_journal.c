/* Copyright (c) 2026 Tunuino. Two flash sectors, append records, commit marker last. */
#include "tunuino/config_journal.h"
#include <string.h>

static const uint8_t magic[8]={'T','N','U','I','O','0','0','3'};
static const uint8_t marker[4]={'D','O','N','E'};
static uint32_t get32(const uint8_t *p) {return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static void put32(uint8_t *p,uint32_t n){p[0]=(uint8_t)(n>>24);p[1]=(uint8_t)(n>>16);p[2]=(uint8_t)(n>>8);p[3]=(uint8_t)n;}
static bool erased(const uint8_t *p,size_t size){for(size_t i=0;i<size;++i)if(p[i]!=255)return false;return true;}
static bool valid(const uint8_t *record,bool legacy) {
    size_t payload=legacy?TUNUINO_IO_LEGACY_SIZE:TUNUINO_IO_SETTINGS_SIZE,size=payload+32u;
    return memcmp(record,magic,7)==0 && record[7]==(legacy?'2':'3') && get32(record+12)==payload
        && memcmp(record+size-4,marker,4)==0
        && get32(record+size-16)==tunuino_crc32(record,size-16)
        && erased(record+size-12,8)
        && tunuino_device_validate(record+16,payload);
}
bool tunuino_journal_init(tunuino_config_journal *j,const tunuino_journal_io *io,uint8_t settings[TUNUINO_IO_SETTINGS_SIZE]) {
    if(j==NULL||io==NULL||settings==NULL||io->read==NULL||io->program==NULL||io->erase==NULL)return false;
    memset(j,0,sizeof(*j));j->io=*io;uint8_t record[TUNUINO_JOURNAL_RECORD_BYTES];
    /* A bank uses one record format. Scan both grids so interrupted migration
     * can always recover the still-committed legacy bank. */
    for(unsigned format=0;format<2;++format){
        bool legacy=format==0;size_t payload=legacy?TUNUINO_IO_LEGACY_SIZE:TUNUINO_IO_SETTINGS_SIZE,size=payload+32u;
        for(unsigned bank=0;bank<2;++bank)for(size_t offset=0;offset+size<=TUNUINO_JOURNAL_BANK_BYTES;offset+=size){
            if(!io->read(io->user,bank,offset,record,size))return false;
            if(valid(record,legacy)){
                uint32_t sequence=get32(record+8),delta=sequence-j->sequence;
                if(!j->found||(delta&&delta<UINT32_C(0x80000000))||(!delta&&j->legacy&&!legacy)){
                    j->found=true;j->legacy=legacy;j->sequence=sequence;j->bank=bank;j->offset=offset;
                    memset(settings,0,TUNUINO_IO_SETTINGS_SIZE);memcpy(settings,record+16,payload);
                }
            }
        }
    }
    return j->found;
}
bool tunuino_journal_save(tunuino_config_journal *j,const uint8_t settings[TUNUINO_IO_SETTINGS_SIZE]) {
    if(j==NULL||!tunuino_device_validate(settings,TUNUINO_IO_SETTINGS_SIZE))return false;
    uint8_t record[TUNUINO_JOURNAL_RECORD_BYTES];unsigned bank=j->found?j->bank:0;size_t offset=j->found?j->offset+TUNUINO_JOURNAL_RECORD_BYTES:0;
    if(j->found&&!j->legacy) {
        if(!j->io.read(j->io.user,j->bank,j->offset,record,sizeof(record)))return false;
        if(valid(record,false)&&memcmp(record+16,settings,TUNUINO_IO_SETTINGS_SIZE)==0)return true;
    }
    if(j->found&&j->legacy){
        /* Migration commits in the other bank, retaining the original image
         * until the new-format record has been verified. */
        bank=1u-j->bank;if(!j->io.erase(j->io.user,bank))return false;offset=0;
    }
    /* Torn records consume their slot. Never program over a non-erased word. */
    for(;offset+TUNUINO_JOURNAL_RECORD_BYTES<=TUNUINO_JOURNAL_BANK_BYTES;offset+=TUNUINO_JOURNAL_RECORD_BYTES) {
        if(!j->io.read(j->io.user,bank,offset,record,sizeof(record)))return false;
        if(erased(record,sizeof(record)))break;
    }
    if(offset+TUNUINO_JOURNAL_RECORD_BYTES>TUNUINO_JOURNAL_BANK_BYTES) {
        bank=j->found?1u-j->bank:0;
        if(!j->io.erase(j->io.user,bank))return false;
        offset=0;
    }
    memset(record,255,sizeof(record));memcpy(record,magic,8);
    uint32_t sequence=j->found?j->sequence+1:1;
    put32(record+8,sequence);
    put32(record+12,TUNUINO_IO_SETTINGS_SIZE);
    memcpy(record+16,settings,TUNUINO_IO_SETTINGS_SIZE);
    put32(record+TUNUINO_JOURNAL_RECORD_BYTES-16,
        tunuino_crc32(record,TUNUINO_JOURNAL_RECORD_BYTES-16));
    if(!j->io.program(j->io.user,bank,offset,record,TUNUINO_JOURNAL_RECORD_BYTES-4))
        return false;
    if(!j->io.program(j->io.user,bank,offset+TUNUINO_JOURNAL_RECORD_BYTES-4,marker,4))
        return false;
    if(!j->io.read(j->io.user,bank,offset,record,sizeof(record))||!valid(record,false)
        ||memcmp(record+16,settings,TUNUINO_IO_SETTINGS_SIZE)!=0)return false;
    j->found=true;j->legacy=false;j->sequence=sequence;j->bank=bank;j->offset=offset;return true;
}
