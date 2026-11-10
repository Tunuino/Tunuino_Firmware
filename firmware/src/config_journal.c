/* Copyright (c) 2026 Tunuino. Atomic calibration + dedicated service data records. */
#include "tunuino/config_journal.h"
#include "tunuino/service_data.h"
#include <string.h>
static const uint8_t magic[7]={'T','N','U','I','O','0','0'},marker[4]={'D','O','N','E'};
static uint32_t get32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static void put32(uint8_t *p,uint32_t n){p[0]=(uint8_t)(n>>24);p[1]=(uint8_t)(n>>16);p[2]=(uint8_t)(n>>8);p[3]=(uint8_t)n;}
static bool erased(const uint8_t *p,size_t n){for(size_t i=0;i<n;++i)if(p[i]!=255)return false;return true;}
static size_t settings_size(unsigned f){return f==2?TUNUINO_IO_LEGACY_SIZE:f<5?TUNUINO_IO_V4_SIZE:f==5?TUNUINO_IO_V5_SIZE:f<8?TUNUINO_IO_V7_SIZE:TUNUINO_IO_SETTINGS_SIZE;}
static size_t payload(unsigned f){return settings_size(f)+(f>=4?TUNUINO_SERVICE_SIZE+4u:0u);}
static bool valid(const uint8_t *r,unsigned format){
    const size_t bytes=payload(format),size=bytes+32u,at=format>=4?20u:16u,settings=settings_size(format);
    return memcmp(r,magic,7)==0&&r[7]=='0'+format&&get32(r+12)==bytes&&memcmp(r+size-4,marker,4)==0
        &&get32(r+size-16)==tunuino_crc32(r,size-16)&&erased(r+size-12,8)
        &&tunuino_device_validate(r+at,settings)&&(format<4||(get32(r+16)<=1&&tunuino_service_validate(r+at+settings)));
}
bool tunuino_journal_init(tunuino_config_journal *j,const tunuino_journal_io *io,uint8_t settings[TUNUINO_IO_SETTINGS_SIZE]){
    if(!j||!io||!settings||!io->read||!io->program||!io->erase)return false;
    memset(j,0,sizeof(*j));j->io=*io;uint8_t record[TUNUINO_JOURNAL_RECORD_BYTES];
    /* Each bank has one format grid. Interrupted migration leaves the previous
     * committed bank intact; scan all supported grids before choosing a record. */
    for(unsigned format=2;format<=8;++format){const size_t bytes=payload(format),size=bytes+32u;
        for(unsigned bank=0;bank<2;++bank)for(size_t offset=0;offset+size<=TUNUINO_JOURNAL_BANK_BYTES;offset+=size){
            if(!io->read(io->user,bank,offset,record,size))return false;
            if(valid(record,format)){const uint32_t sequence=get32(record+8),delta=sequence-j->sequence;
                if(!j->found||(delta&&delta<UINT32_C(0x80000000))||(!delta&&format>j->format)){
                    j->found=true;j->format=format;j->legacy=format!=8;j->sequence=sequence;j->bank=bank;j->offset=offset;
                    memset(settings,0,TUNUINO_IO_SETTINGS_SIZE);memcpy(settings,record+(format>=4?20:16),settings_size(format));
                    memset(j->service,0,TUNUINO_SERVICE_SIZE);if(format>=4)memcpy(j->service,record+20+settings_size(format),TUNUINO_SERVICE_SIZE);j->calibration_persisted=format<4||get32(record+16)==1;
                }
            }
        }
    }
    j->healthy=true;return j->found;
}
bool tunuino_journal_save_combined(tunuino_config_journal *j,const uint8_t settings[TUNUINO_IO_SETTINGS_SIZE],const uint8_t service[TUNUINO_SERVICE_SIZE],bool calibration_persisted){
    if(!j||!j->healthy||!settings||!service||!tunuino_device_validate(settings,TUNUINO_IO_SETTINGS_SIZE)||!tunuino_service_validate(service))return false;
    uint8_t record[TUNUINO_JOURNAL_RECORD_BYTES];unsigned bank=j->found?j->bank:0;size_t offset=j->found?j->offset+TUNUINO_JOURNAL_RECORD_BYTES:0;
    if(j->found&&!j->legacy){
        if(!j->io.read(j->io.user,j->bank,j->offset,record,sizeof(record)))return false;
        if(valid(record,8)&&get32(record+16)==(calibration_persisted?1u:0u)&&memcmp(record+20,settings,TUNUINO_IO_SETTINGS_SIZE)==0&&memcmp(record+20+TUNUINO_IO_SETTINGS_SIZE,service,TUNUINO_SERVICE_SIZE)==0)return true;
    }
    if(j->found&&j->legacy){bank=1u-j->bank;if(!j->io.erase(j->io.user,bank))return false;offset=0;}
    for(;offset+sizeof(record)<=TUNUINO_JOURNAL_BANK_BYTES;offset+=sizeof(record)){
        if(!j->io.read(j->io.user,bank,offset,record,sizeof(record)))return false;
        if(erased(record,sizeof(record)))break;
    }
    if(offset+sizeof(record)>TUNUINO_JOURNAL_BANK_BYTES){bank=j->found?1u-j->bank:0;if(!j->io.erase(j->io.user,bank))return false;offset=0;}
    memset(record,255,sizeof(record));memcpy(record,magic,7);record[7]='8';const uint32_t sequence=j->found?j->sequence+1u:1u;
    put32(record+8,sequence);put32(record+12,TUNUINO_IO_SETTINGS_SIZE+TUNUINO_SERVICE_SIZE+4u);put32(record+16,calibration_persisted?1u:0u);memcpy(record+20,settings,TUNUINO_IO_SETTINGS_SIZE);memcpy(record+20+TUNUINO_IO_SETTINGS_SIZE,service,TUNUINO_SERVICE_SIZE);
    put32(record+sizeof(record)-16,tunuino_crc32(record,sizeof(record)-16));
    if(!j->io.program(j->io.user,bank,offset,record,sizeof(record)-4)||!j->io.program(j->io.user,bank,offset+sizeof(record)-4,marker,4))return false;
    if(!j->io.read(j->io.user,bank,offset,record,sizeof(record))||!valid(record,8)||get32(record+16)!=(calibration_persisted?1u:0u)||memcmp(record+20,settings,TUNUINO_IO_SETTINGS_SIZE)!=0||memcmp(record+20+TUNUINO_IO_SETTINGS_SIZE,service,TUNUINO_SERVICE_SIZE)!=0)return false;
    j->found=true;j->legacy=false;j->format=8;j->calibration_persisted=calibration_persisted;j->sequence=sequence;j->bank=bank;j->offset=offset;if(j->service!=service)memcpy(j->service,service,TUNUINO_SERVICE_SIZE);return true;
}
bool tunuino_journal_save_service(tunuino_config_journal *j,const uint8_t settings[TUNUINO_IO_SETTINGS_SIZE],const uint8_t service[TUNUINO_SERVICE_SIZE]){return j?tunuino_journal_save_combined(j,settings,service,j->calibration_persisted):false;}
bool tunuino_journal_save(tunuino_config_journal *j,const uint8_t settings[TUNUINO_IO_SETTINGS_SIZE]){return j?tunuino_journal_save_combined(j,settings,j->service,true):false;}
