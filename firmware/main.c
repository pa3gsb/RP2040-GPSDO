#include "main.h"
#include "counter.pio.h"
#include "pps.pio.h"

#include "si5351.h" 

//------------------------------------------------------------------------------------------------------
#define PPS_PIN 2
#define SIGNAL_PIN 15
#define INFO_RUN_PIN     PICO_DEFAULT_LED_PIN
#define INFO_PPS_PIN   21
#define INFO_LOCK_PIN  20

// these are used by the PPS interrupt routine
volatile int ppsFlag = 0;
volatile absolute_time_t ppsTime;
volatile int32_t pulseCountSnapshot = 0;

// pulseCount is the target for the DMA stream from the PPS SM
uint32_t pulseCount = 0;

// declare the IDs for the DMA channels
int dma_chan = 0;
int dma_chan2 = 0;
int dma_chan3 = 0;

// PIO globals
PIO pio = pio0;
uint sm_pps = 0;

#define REF_EXTERNAL_FREQ  25000000ULL
uint64_t ref_external_freq=(uint64_t) REF_EXTERNAL_FREQ; 

#define REF_INTERNAL_FREQ  20000000ULL 
uint64_t ref_internal_freq=(uint64_t) REF_INTERNAL_FREQ;

#define REF_EXTERNAL_10MHZ 10000000ULL
uint64_t ref_external_10=(uint64_t) REF_EXTERNAL_10MHZ; 


unsigned int count_time=0;        		// counts the seconds in a measurement cycle 
int start = 1;               			// Wait one second after first pps 
int duration = 1;             			// Initial measurement duration

int32_t correction_factor;              // correction of master clock...
int64_t target_count;           		// Target count of pulses
int64_t measured_count;         		// Actual count of pulses

int bullseye = 0;

bool blink_lock = false;
bool blink_pps = false;

int found_negative = 0;
int found_positive = 0;

void pps_callback() {
    pulseCountSnapshot = pulseCount;
    ppsTime = get_absolute_time();
    ppsFlag = 1;
    pio_interrupt_clear(pio, 0);
}

void execute_correction(int32_t _correction) {
    correction_factor = correction_factor + _correction;
    set_correction(correction_factor, SI5351_PLL_INPUT_XO);
    
    si5351_set_freq(ref_internal_freq * 100ULL, SI5351_CLK0);
    si5351_set_freq(ref_external_freq * 100ULL, SI5351_CLK1);
    si5351_set_freq(ref_external_10   * 100ULL, SI5351_CLK2);
}

bool apply_fast_correction(int32_t _deviation) {
    int32_t _correction;
    bool fast_running =  true;
    if (abs(_deviation) > 10000 ) return fast_running;
    if (_deviation < 0) {found_negative++; found_positive=0;}
    if (_deviation > 0) {found_positive++; found_negative=0;}
    if (_deviation < 0) _correction = found_negative  * _deviation;
    if (_deviation > 0) _correction = found_positive  * _deviation;
    if (_deviation == 0 ) bullseye++; else { execute_correction(_correction); bullseye = 0; }
    if (bullseye == 10) fast_running = false;
    if (!fast_running) {found_positive=0; found_negative=0;}
    return fast_running;
}

void  apply_slow_correction(int32_t _deviation) {

    int32_t _correction;
    if (_deviation < 0) {found_negative++; found_positive=0;}
    if (_deviation > 0) {found_positive++; found_negative=0;}
    if (_deviation < 0) _correction = found_negative  * _deviation;
    if (_deviation > 0) _correction = found_positive  * _deviation;
    if (_deviation == 0) {found_positive=0; found_negative=0;}

    // Only when deviation is within a certain limit we will act!
    if (abs(_deviation) < 100 && abs(_deviation) > 0) execute_correction(_correction);
 
    if(abs(_deviation) > 4) {
        // Deviation too large, increasing speed of changes
        duration = duration / 2;
        if (duration == 0) duration = 1;
    } else if(abs(_deviation) < 2) {
        // Deviation small, decreasing speed of changes
        duration = duration * 2;
        if (duration > 512) duration = 512;
    }
}

int main() {
    stdio_init_all();

    gpio_init(INFO_RUN_PIN);
    gpio_set_dir(INFO_RUN_PIN, GPIO_OUT);
    gpio_init(INFO_PPS_PIN);
    gpio_set_dir(INFO_PPS_PIN, GPIO_OUT);
    gpio_init(INFO_LOCK_PIN);
    gpio_set_dir(INFO_LOCK_PIN, GPIO_OUT);

    gpio_put(INFO_RUN_PIN, true);
   
    sleep_ms(3000);

    si5351_init(0x60, SI5351_CRYSTAL_LOAD_6PF, 25000000, 0);
    si5351_set_freq(ref_internal_freq* 100ULL, SI5351_CLK0);
    si5351_output_enable(SI5351_CLK0, 1);
    si5351_set_freq(ref_external_freq* 100ULL, SI5351_CLK1);
    si5351_output_enable(SI5351_CLK1, 1);
    si5351_set_freq(ref_external_10* 100ULL, SI5351_CLK2);
    si5351_output_enable(SI5351_CLK2, 1);
    
    bool clkset = set_sys_clock_khz(250000, false);
    if(clkset) {
        printf("Clock is set at 250Mhz\n");
    }
    
    // load programs
    uint offset = pio_add_program(pio, &counter_program);
    uint offset_pps = pio_add_program(pio, &pps_program);

    // PPS program
    sm_pps = 0;
    pps_program_init(pio, sm_pps, offset_pps, PPS_PIN);

    // pulse counter program
    uint sm = 1;
    counter_program_init(pio, sm, offset, SIGNAL_PIN);

    // setup DMA between SMs
    dma_chan = dma_claim_unused_channel(true);
    dma_chan2 = dma_claim_unused_channel(true);

    // channel 1, this starts and than hands over to the second channel when it is done
    // channel 2 then hands back to channel 1, so we get a continous DMA stream to a single target variable
    dma_channel_config dc = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_chain_to(&dc, dma_chan2);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm_pps, true));
    dma_channel_configure(dma_chan, &dc,
        &pio->txf[sm_pps],
        &pio->rxf[sm],
        0xFFFFFFFF,
        true
    );

    // channel 2 as above
    dma_channel_config dc2 = dma_channel_get_default_config(dma_chan2);
    channel_config_set_transfer_data_size(&dc2, DMA_SIZE_32);
    channel_config_set_read_increment(&dc2, false);
    channel_config_set_write_increment(&dc2, false);
    channel_config_set_dreq(&dc2, pio_get_dreq(pio, sm_pps, true));
    channel_config_set_chain_to(&dc2, dma_chan);
    dma_channel_configure(dma_chan2, &dc2,
        &pio->txf[sm_pps],
        &pio->rxf[sm],
        0xFFFFFFFF,
        false
    );

    // setup DMA from PPS SM to CPU
    dma_chan3 = dma_claim_unused_channel(true);

    dma_channel_config dc3 = dma_channel_get_default_config(dma_chan3);
    channel_config_set_transfer_data_size(&dc3, DMA_SIZE_32);
    channel_config_set_read_increment(&dc3, false);
    channel_config_set_write_increment(&dc3, false);
    //channel_config_set_chain_to(&dc, dma_chan3);
    channel_config_set_dreq(&dc3, pio_get_dreq(pio, sm_pps, false));
    dma_channel_configure(dma_chan3, &dc3,
        &pulseCount,
        &pio->rxf[sm_pps],
        0xFFFFFFFF,
        true
    );

    // setup PPS IRQ/ISR
    irq_set_exclusive_handler(PIO0_IRQ_0, pps_callback);
    irq_set_enabled(PIO0_IRQ_0, true);

    // enable state machines
    pio_sm_set_enabled(pio, sm_pps, true);
    pio_sm_set_enabled(pio, sm, true);

    uint32_t prevCount = 0;
     
    bool fast_correction_running = true;

    while(1) {
        if(ppsFlag == 1) {
            blink_pps = !blink_pps; gpio_put(INFO_PPS_PIN, blink_pps);
            uint32_t delta = prevCount - pulseCountSnapshot;
            
            measured_count += delta;	
            if (count_time == start) measured_count = 0;								
            if (count_time == (duration + start) )	{ 
                int32_t target_pulses= ref_internal_freq * duration; 
                
                int32_t deviation =  measured_count - target_pulses;
                double result = deviation /(double)target_pulses;

                if (fast_correction_running) {
                    blink_lock = !blink_lock; gpio_put(INFO_LOCK_PIN, blink_lock);
                    fast_correction_running = apply_fast_correction(deviation);
                    printf("FAST : Deviation %d\tError %.3e\tNext Duration %d \t Corr. %d \n", deviation, result, duration, correction_factor);
                } else  {
                    if (duration < 16) {blink_lock= !blink_lock; gpio_put(INFO_LOCK_PIN, blink_lock);}
                    else gpio_put(INFO_LOCK_PIN, true);
                    apply_slow_correction(deviation);
                    printf("SLOW : Deviation %d\tError %.3e\tNext Duration %d \t Corr. %d \n", deviation, result, duration, correction_factor);
                }
                count_time = 0;
            }
            count_time++;
            prevCount = pulseCountSnapshot;
            ppsFlag = 0;
        }
        tight_loop_contents();
    }
}