/* Copyright (c) 2026 Tunuino. Proteus v0.7/F427 input acquisition.
 * Source: Tunuino/proteus @ 383558bf7ca1c53ad8bbd8fdc9dd3a4187013c46,
 * U1501 pads 26..29,34..37,40..47,58,59; STM32F427 LQFP144 pin table.
 * Actuator pins and timing are isolated in board_outputs.c. */
#include "ch.h"
#include "hal.h"
#include "board_io.h"
#include "board_outputs.h"
#include "board_status.h"
#include "tunuino/config_journal.h"
#include "tunuino/capture_source.h"
#include <string.h>

static tunuino_config_journal journal;
static tunuino_capture_source capture;
static uint32_t applied_generation, last_adc_start, last_adc_good;
static uint16_t raw[16];
static adcsample_t adc_samples[16*4];
static volatile bool adc_failed;
static volatile bool adc_completed;
static bool adc_good;
static uint32_t previous_counter;
static uint64_t epoch;
static tunuino_raw_capture edges[64];
static volatile unsigned edge_head,edge_tail;
static volatile bool edge_overflow;
static bool trigger_enabled;
static uint32_t trigger_errors;
static uint32_t saved_flash_acr;
static uint8_t cam_first=1,cam_last=2,cam_phase;

static void adc_error(ADCDriver *driver,adcerror_t error) {(void)driver;(void)error;adc_failed=true;}
static void adc_complete(ADCDriver *driver){(void)driver;adc_completed=true;}
static const ADCConversionGroup group={
    .circular=false,.num_channels=16,.end_cb=adc_complete,.error_cb=adc_error,
    .cr1=0,.cr2=ADC_CR2_SWSTART,
    .smpr1=UINT32_C(0x00FFFFFF),.smpr2=UINT32_C(0x3FFFFFFF),.htr=4095,.ltr=0,
    .sqr1=ADC_SQR1_NUM_CH(16)|ADC_SQR1_SQ13_N(14)|ADC_SQR1_SQ14_N(15)|ADC_SQR1_SQ15_N(8)|ADC_SQR1_SQ16_N(9),
    .sqr2=ADC_SQR2_SQ7_N(2)|ADC_SQR2_SQ8_N(3)|ADC_SQR2_SQ9_N(4)|ADC_SQR2_SQ10_N(5)|ADC_SQR2_SQ11_N(6)|ADC_SQR2_SQ12_N(7),
    .sqr3=ADC_SQR3_SQ1_N(10)|ADC_SQR3_SQ2_N(11)|ADC_SQR3_SQ3_N(12)|ADC_SQR3_SQ4_N(13)|ADC_SQR3_SQ5_N(0)|ADC_SQR3_SQ6_N(1)
};

uint64_t board_io_time_us(void) {
    uint32_t now=TIM2->CNT;
    if(now<previous_counter)epoch+=UINT64_C(1)<<32;
    previous_counter=now;return epoch|now;
}
uint64_t board_io_clock(void *user){(void)user;return board_io_time_us()/1000;}

static uintptr_t bank_address(unsigned bank){return bank==0?UINT32_C(0x08020000):UINT32_C(0x08040000);}
static bool flash_read(void *user,unsigned bank,size_t offset,uint8_t *bytes,size_t length){
    (void)user;if(bank>1||offset>TUNUINO_JOURNAL_BANK_BYTES||length>TUNUINO_JOURNAL_BANK_BYTES-offset)return false;
    memcpy(bytes,(const void *)(bank_address(bank)+offset),length);return true;
}
static bool flash_wait(void) {
    uint32_t start=TIM2->CNT;
    while(FLASH->SR&FLASH_SR_BSY)if((uint32_t)(TIM2->CNT-start)>5000000u)return false;
    return (FLASH->SR&(FLASH_SR_SOP|FLASH_SR_WRPERR|FLASH_SR_PGAERR|FLASH_SR_PGPERR|FLASH_SR_PGSERR))==0;
}
static bool flash_unlock(void) {
    uint32_t start=TIM2->CNT;
    while(FLASH->SR&FLASH_SR_BSY)if((uint32_t)(TIM2->CNT-start)>5000000u)return false;
    if(FLASH->CR&FLASH_CR_LOCK){FLASH->KEYR=UINT32_C(0x45670123);FLASH->KEYR=UINT32_C(0xCDEF89AB);}
    FLASH->SR=FLASH_SR_EOP|FLASH_SR_SOP|FLASH_SR_WRPERR|FLASH_SR_PGAERR|FLASH_SR_PGPERR|FLASH_SR_PGSERR;
    if(FLASH->CR&FLASH_CR_LOCK)return false;
    saved_flash_acr=FLASH->ACR;
    FLASH->ACR=saved_flash_acr&~(FLASH_ACR_ICEN|FLASH_ACR_DCEN);
    FLASH->ACR|=FLASH_ACR_ICRST|FLASH_ACR_DCRST;
    FLASH->ACR&=~(FLASH_ACR_ICRST|FLASH_ACR_DCRST);
    return true;
}
static void flash_finish(void) {
    FLASH->CR=FLASH_CR_LOCK;
    /* Refresh instruction/data caches after changing flash contents. */
    FLASH->ACR=saved_flash_acr&~(FLASH_ACR_ICEN|FLASH_ACR_DCEN);
    FLASH->ACR|=FLASH_ACR_ICRST|FLASH_ACR_DCRST;
    FLASH->ACR=saved_flash_acr;
}
static bool flash_program(void *user,unsigned bank,size_t offset,const uint8_t *bytes,size_t length){
    (void)user;if(bank>1||offset%4||length%4||offset>TUNUINO_JOURNAL_BANK_BYTES
        ||length>TUNUINO_JOURNAL_BANK_BYTES-offset||!flash_unlock())return false;
    bool okay=true;
    for(size_t i=0;i<length;i+=4){
        volatile uint32_t *target=(volatile uint32_t *)(bank_address(bank)+offset+i);uint32_t value;
        memcpy(&value,bytes+i,4);
        if(*target!=UINT32_MAX){okay=false;break;}
        FLASH->CR=FLASH_CR_PSIZE_1|FLASH_CR_PG;*target=value;
        if(!flash_wait()||*target!=value){okay=false;break;}
    }
    flash_finish();return okay;
}
static bool flash_erase(void *user,unsigned bank){
    (void)user;if(bank>1||!flash_unlock())return false;
    FLASH->CR=FLASH_CR_PSIZE_1|FLASH_CR_SER|((bank+5u)<<3);FLASH->CR|=FLASH_CR_STRT;
    bool okay=flash_wait();flash_finish();return okay;
}
bool board_io_save(void *user,const uint8_t *bytes,size_t length){
    (void)user;
    if(!board_outputs_idle() || length!=TUNUINO_IO_SETTINGS_SIZE)return false;
    board_status_flash(true);
    bool saved=tunuino_journal_save(&journal,bytes);
    board_status_flash(false);
    return saved;
}

static void edge_callback(void *argument){
    unsigned head=edge_head;
    if((unsigned)(head-edge_tail)>=64){edge_overflow=true;return;}
    edges[head%64]=(tunuino_raw_capture){TIM2->CNT,(uintptr_t)argument==0?TUNUINO_CAPTURE_CRANK:TUNUINO_CAPTURE_CAM};
    /* The main thread drains while holding the system lock. Both pads share
     * EXTI9_5 and therefore have one serialized interrupt producer. */
    __DMB();edge_head=head+1;
}
static void configure_capture(tunuino_device *d,uint64_t now){
    tunuino_device_position(d,now,NULL,NULL);
    palDisablePadEvent(GPIOE,7);palDisablePadEvent(GPIOE,8);
    chSysLock();edge_head=edge_tail=0;edge_overflow=false;chSysUnlock();
    trigger_enabled=d->active[2]!=0;
    const tunuino_position_config config={{d->active[3],d->active[4],30,12000,300000},cam_first,cam_last,cam_phase};
    tunuino_position_config selected=config;selected.crank.maximum_rpm=tunuino_device_u16(d->active+96);
    if(tunuino_capture_source_init(&capture,&selected,(uint32_t)now,now,1000000,100000)!=TUNUINO_SOURCE_OK)trigger_enabled=false;
    if(trigger_enabled){
        palSetPadCallback(GPIOE,7,edge_callback,(void *)(uintptr_t)0);
        palSetPadCallback(GPIOE,8,edge_callback,(void *)(uintptr_t)1);
        palEnablePadEvent(GPIOE,7,PAL_EVENT_MODE_RISING_EDGE);palEnablePadEvent(GPIOE,8,PAL_EVENT_MODE_RISING_EDGE);
    }
    applied_generation=d->generation;
}
void board_io_resync(tunuino_device *d){board_outputs_stop();configure_capture(d,board_io_time_us());}
void board_io_usb_recover(tunuino_device *d){if(!board_outputs_autonomous())board_io_resync(d);}
void board_io_phase(tunuino_device *d,uint8_t first,uint8_t last,uint8_t phase){
    cam_first=first;cam_last=last;cam_phase=phase;board_io_resync(d);
}
void board_io_start(tunuino_device *d){
    _Static_assert(STM32_TIMCLK1%1000000u==0,"TIM2 clock must support 1 us");
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIOEEN;
    RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;(void)RCC->APB1ENR;
    TIM2->CR1=0;TIM2->PSC=STM32_TIMCLK1/1000000u-1u;TIM2->ARR=UINT32_MAX;
    TIM2->EGR=TIM_EGR_UG;TIM2->CNT=0;TIM2->CR1=TIM_CR1_CEN;
    board_outputs_start(d);
    for(unsigned pin=0;pin<8;++pin)palSetPadMode(GPIOA,pin,PAL_MODE_INPUT_ANALOG);
    for(unsigned pin=0;pin<6;++pin)palSetPadMode(GPIOC,pin,PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOB,0,PAL_MODE_INPUT_ANALOG);palSetPadMode(GPIOB,1,PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOE,7,PAL_MODE_INPUT);palSetPadMode(GPIOE,8,PAL_MODE_INPUT);
    adcStart(&ADCD1,NULL);
    const tunuino_journal_io io={flash_read,flash_program,flash_erase,NULL};uint8_t saved[TUNUINO_IO_SETTINGS_SIZE];
    if(tunuino_journal_init(&journal,&io,saved))(void)tunuino_device_restore(d,saved,sizeof(saved));
    board_outputs_restore(d,true);
    configure_capture(d,board_io_time_us());
}
void board_io_update(tunuino_device *d){
    uint64_t now=board_io_time_us();uint32_t ms=(uint32_t)(now/1000);
    if(d->generation!=applied_generation)configure_capture(d,now);
    if(adc_completed){
        for(unsigned channel=0;channel<16;++channel){unsigned sum=0;for(unsigned sample=0;sample<4;++sample)sum+=adc_samples[sample*16+channel];raw[channel]=(uint16_t)((sum+2)/4);}
        adc_completed=false;adc_good=!adc_failed&&(uint32_t)(ms-last_adc_start)<=10;last_adc_good=last_adc_start;
    }
    if(adc_failed||(ADCD1.state==ADC_ACTIVE&&(uint32_t)(ms-last_adc_start)>10)){
        adcStopConversion(&ADCD1);adc_good=false;
    }
    if(ADCD1.state==ADC_READY&&(uint32_t)(ms-last_adc_start)>=10){
        adc_failed=false;adc_completed=false;last_adc_start=ms;adcStartConversion(&ADCD1,&group,adc_samples,4);
    }
    tunuino_position_reading position={0};tunuino_cam_reading cam={0};uint64_t crank_edge=0;
    if(trigger_enabled){
        /* Latch the complete pending batch before taking the current timer.
         * Do not mask IRQs while decoding, serializing or touching flash. */
        tunuino_raw_capture pending[64];size_t count=0;bool lost;uint32_t counter;
        chSysLock();
        while(edge_tail!=edge_head&&count<64){pending[count++]=edges[edge_tail%64];++edge_tail;}
        lost=edge_overflow;counter=TIM2->CNT;chSysUnlock();
        uint64_t observed;
        /* source_sample supports a capture per channel, so replay a complete
         * chronological batch against one coherent observation. */
        if(lost || tunuino_capture_source_observe(&capture,counter,&observed)!=TUNUINO_SOURCE_OK){
            (void)tunuino_capture_source_lost(&capture);++trigger_errors;trigger_enabled=false;
        }else{
            for(size_t i=0;i<count;++i)if(tunuino_capture_source_push(&capture,pending[i].counter,pending[i].kind)!=TUNUINO_SOURCE_OK){++trigger_errors;trigger_enabled=false;break;}
            size_t processed;(void)tunuino_capture_processor_drain(&capture.processor,observed,64,&processed);
            if(trigger_enabled)(void)tunuino_capture_processor_anchor(&capture.processor,observed,&position,&crank_edge);
            if(trigger_enabled)(void)tunuino_capture_processor_cam(&capture.processor,observed,&cam);
        }
    }
    tunuino_device_sample(d,now/1000,raw,adc_good&&(uint32_t)(ms-last_adc_good)<100,
        (uint32_t)(position.rpm*1000+0.5),position.crank_valid!=0,position.tooth_index,trigger_errors);
    tunuino_device_position(d,board_io_time_us(),&position,&cam);
    board_outputs_update(d,&position,(uint32_t)crank_edge,TIM2->CNT);
}
