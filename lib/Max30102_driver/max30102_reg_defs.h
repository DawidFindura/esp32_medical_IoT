#ifndef _MAX30102_REG_DEFS_H_
#define _MAX30102_REG_DEFS_H_

#include <stdint.h>

/*Read/Write, Read, Write registers attributes*/
#define _RW_                          volatile
#define _R_                           volatile
#define _W_                           volatile

/*MAX30102 register addresses*/
#define MAX30102_BASE_REG_ADDR                     (0x00)

/*Status*/
#define MAX30102_INTERRUPT_STATUS_1_OFFSET         (0x00)
#define MAX30102_REG_ADDR_INTERRUPT_STATUS_1       (MAX30102_BASE_REG_ADDR + MAX30102_INTERRUPT_STATUS_1_OFFSET)
#define MAX30102_INTERRUPT_STATUS_2_OFFSET         (0x01)
#define MAX30102_REG_ADDR_INTERRUPT_STATUS_2       (MAX30102_BASE_REG_ADDR + MAX30102_INTERRUPT_STATUS_2_OFFSET)
#define MAX30102_INTERRUPT_ENABLE_1_OFFSET         (0x02)
#define MAX30102_REG_ADDR_INTERRUPT_ENABLE_1       (MAX30102_BASE_REG_ADDR + MAX30102_INTERRUPT_ENABLE_1_OFFSET)
#define MAX30102_INTERRUPT_ENABLE_2_OFFSET         (0x03)
#define MAX30102_REG_ADDR_INTERRUPT_ENABLE_2       (MAX30102_BASE_REG_ADDR + MAX30102_INTERRUPT_ENABLE_2_OFFSET)

/*FIFO*/
#define MAX30102_FIFO_WR_PTR_OFFSET                (0x04)
#define MAX30102_REG_ADDR_FIFO_WR_PTR              (MAX30102_BASE_REG_ADDR + MAX30102_FIFO_WR_PTR_OFFSET)
#define MAX30102_OVF_COUNTER_OFFSET                (0x05)
#define MAX30102_REG_ADDR_OVF_COUNTER              (MAX30102_BASE_REG_ADDR + MAX30102_OVF_COUNTER_OFFSET)
#define MAX30102_FIFO_RD_PTR_OFFSET                (0x06)      
#define MAX30102_REG_ADDR_FIFO_RD_PTR              (MAX30102_BASE_REG_ADDR + MAX30102_FIFO_RD_PTR_OFFSET)
#define MAX30102_FIFO_DATA_OFFSET                  (0x07)
#define MAX30102_REG_ADDR_FIFO_DATA                (MAX30102_BASE_REG_ADDR + MAX30102_FIFO_DATA_OFFSET)    

/*Configuration*/
#define MAX30102_REG_ADDR_FIFO_CONFIG              (0x08)        
#define MAX30102_REG_ADDR_MODE_CONFIG              (0x09)       
#define MAX30102_REG_ADDR_SPO2_CONFIG              (0x0A)        
#define MAX30102_REG_ADDR_LED1_PA                  (0x0C)       
#define MAX30102_REG_ADDR_LED2_PA                  (0x0D)       
#define MAX30102_REG_ADDR_PROX_MODE_LED_PA         (0x10)
#define MAX30102_REG_ADDR_MULTI_LED_MODE_CNTRL_1   (0x11)
#define MAX30102_REG_ADDR_MULTI_LED_MODE_CNTRL_2   (0x12)

/*Die temperature*/
#define MAX30102_REG_ADDR_DIE_TEMP_INT             (0x1F)
#define MAX30102_REG_ADDR_DIE_TEMP_FRAC            (0x20)        
#define MAX30102_REG_ADDR_DIE_TEMP_CONFIG          (0x21)

/*Proximity function*/
#define MAX30102_REG_ADDR_PROX_INT_THRESH          (0x30)

/*Part ID*/
#define MAX30102_REG_ADDR_REVISION_ID              (0xFE)        
#define MAX30102_REG_ADDR_PART_ID                  (0xFF)        


/* interrupts types */
#define MAX30102_INTR_TYPE_PWRRDY                  (1 << 0)
#define MAX30102_INTR_TYPE_DIETEMP_RDY             (1 << 1)
#define MAX30102_INTR_TYPE_AMBILITOVF              (1 << 5)
#define MAX30102_INTR_TYPE_NEWSAMPLE               (1 << 6)
#define MAX30102_INTR_TYPE_ALMFULL                 (1 << 7)



typedef enum {
    MAX31_MODE_STARTUP = 0,
    MAX31_MODE_HEARTRATE_RED = 0x02, /** Red led only **/
    MAX31_MODE_SPO2_RED_IR = 0x03,  /** Red and IR **/
    MAX31_MODE_MULTILED_RIR = 0x07, /** Red and IR **/
} max30102_mode_t;

typedef enum {
    MAX31_LED_PWM_69  = 0b0,
    MAX31_LED_PWM_118 = 0b01,
    MAX31_LED_PWM_215 = 0b10,
    MAX31_LED_PWM_411 = 0b11
} max30102_ledpwm_t;

typedef enum {
    MAX31_SAMPLERATE_50 = 0,
    MAX31_SAMPLERATE_100,
    MAX31_SAMPLERATE_200,
    MAX31_SAMPLERATE_400,
    MAX31_SAMPLERATE_800,
    MAX31_SAMPLERATE_1000,
    MAX31_SAMPLERATE_1600,
    MAX31_SAMPLERATE_3200,
} max30102_samplerate_t;

typedef enum {
    MAX31_ADC_RNG_2048,
    MAX31_ADC_RNG_4096,
    MAX31_ADC_RNG_8192,
    MAX31_ADC_RNG_16384,
} max30102_adcrange_t;

typedef enum {
    MAX31_SAMPLE_AVG_1 = 0,
    MAX31_SAMPLE_AVG_2 = 1,
    MAX31_SAMPLE_AVG_4 = 2,
    MAX31_SAMPLE_AVG_8 = 3,
    MAX31_SAMPLE_AVG_16 = 4,
    MAX31_SAMPLE_AVG_32 = 5,
} max30102_sampleavg_t;



/*MAX30102 registers structures*/
typedef struct {
    uint8_t reg_addr;

    union { 
        uint8_t reg_val; //0x00 
        struct {
            _R_ uint8_t pwr_rdy_bit   :1;
            _R_ uint8_t RESERVED      :3;
            _R_ uint8_t prox_intr_bit :1;
            _R_ uint8_t alc_ovf_bit   :1;
            _R_ uint8_t ppg_rdy_bit   :1;
            _R_ uint8_t a_full_bit    :1;
        };
    };
} max30102_intr_status_1_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;  //0x01
        struct {
            _R_ uint8_t RESERVED1        :1;
            _R_ uint8_t die_temp_rdy_bit :1;
            _R_ uint8_t RESERVED2        :6;               
        };
    };
} max30102_intr_status_2_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;  //0x02  INTR_ENABLE1
        struct {
            _RW_ uint8_t RESERVED         :4;
            _RW_ uint8_t prox_intr_en_bit :1;
		    _RW_ uint8_t alc_ovf_en_bit   :1;
		    _RW_ uint8_t ppg_rdy_en_bit   :1;
		    _RW_ uint8_t a_full_en_bit    :1;
        };
    };
} max30102_intr_enable_1_reg_t;


typedef	struct {
    uint8_t reg_addr;

    union {
	    uint8_t reg_val;  //0x03 REG_INTR_ENABLE_2
	    struct {
		    _RW_ uint8_t RESERVED1           :1;
		    _RW_ uint8_t die_temp_rdy_en_bit :1;
		    _RW_ uint8_t RESERVED2           :6;
	    };
    };
} max30102_intr_enable_2_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;  //0x04   REG_FIFO_WR_PTR
        struct{
            _RW_ uint8_t fifo_wr_ptr_bits :5; 
		    _RW_ uint8_t RESERVED         :3;
        };
    };
} max30102_fifo_wr_ptr_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;  //0x05     REG_OVF_COUNTER
	    struct {
		    uint8_t	ovf_counter_bits :5;
		    uint8_t RESERVED         :3;
	    };
    };
} max30102_ovf_counter_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;  //0x06    REG_FIFO_RD_PTR
	    struct {
		    uint8_t	fifo_rd_ptr_bits :5;
		    uint8_t RESERVED         :3;
	    };
    };
} max30102_fifo_rd_ptr_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;  //0x07    FIFO_DATA_REG
	    struct {
		    uint8_t	fifo_data_bits :8;
	    };
    };
} max30102_fifo_data_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
	    uint8_t reg_val;  //0x08   REG_FIFO_CONFIG
	    struct{
		    uint8_t	fifo_a_full_bits     :4;
		    uint8_t fifo_rollover_en_bit :1;
		    uint8_t smp_ave_bits         :3;
        };
    };
} max30102_fifo_config_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
	    uint8_t reg_val; //0x09   REG_MODE_CONFIG
	    struct{
		    uint8_t mode_bits :3;
		    uint8_t	RESERVED  :3;
		    uint8_t reset_bit :1;
		    uint8_t shdn_bit  :1;
	    };
    };
} max30102_mode_config_reg_t;


typedef struct {
    uint8_t reg_addr;

    union {
	    uint8_t reg_val; //0x0A   REG_SPO2_CONFIG
	    struct{
		    uint8_t led_pw_bits       :2;
		    uint8_t	spo2_sr_bits      :3;
		    uint8_t spo2_adc_rge_bits :2;
		    uint8_t RESERVED          :1;
	    };
    };
} max30102_spo2_config_reg_t;


typedef struct { 
    uint8_t reg_addr;

    union {
	    uint8_t reg_val; //0X0C   REG_LED1_PA
	    struct{
		    uint8_t led1_pa_bits :8;
	    };
    };
} max30102_led1_pa_reg_t;

typedef struct {
    uint8_t reg_addr;

    union { 
	    uint8_t reg_val; //0X0D   REG_LED2_PA
	    struct{
		    uint8_t led2_pa_bits :8;
	    };
    };
} max30102_led2_pa_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
	    uint8_t reg_val; //0X10  REG_PILOT_PA
	    struct{
		    uint8_t pilot_pa_bits :8;
	    };
    };
} max30102_prox_mode_led_reg_t;

typedef struct { 
    uint8_t reg_addr;

    union {
	    uint8_t reg_val; //0X11 REG_MULTI_LED_CTRL1
	    struct{
		    uint8_t slot1_bits :3;
		    uint8_t RESERVED2  :1;
		    uint8_t slot2_bits :3;
		    uint8_t RESERVED1  :1;
	    };
    };
} max30102_multi_led_ctrl1_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
	    uint8_t reg_val; //0X12  REG_MULTI_LED_CTRL2
	    struct{
		    uint8_t slot3_bits :3;
		    uint8_t RESERVED2  :1;
		    uint8_t slot4_bits :3;
		    uint8_t RESERVED1  :1;
	    };
    };
} max30102_multi_led_ctrl2_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;
        struct {
            uint8_t tint_bits :8;
        };
    };
} max30102_die_temp_int_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;
        struct {
            uint8_t tfrac_bits :4;
            uint8_t RESERVED   :4;
        };
    };
} max30102_die_temp_frac_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;
        struct {
            uint8_t temp_en_bit :1;
            uint8_t RESERVED    :7;
        }; 
    };  
} max30102_die_temp_config_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;
        struct {
            uint8_t prox_intr_thresh_bits :8;
        };
    };
} max30102_prox_intr_thresh_reg_t;

typedef struct {
    uint8_t reg_addr;
    
    union {
        uint8_t reg_val;
        struct {
            uint8_t rev_id_bits :8;
        };
    };
} max30102_rev_id_reg_t;

typedef struct {
    uint8_t reg_addr;

    union {
        uint8_t reg_val;
        struct {
            uint8_t part_id_bits :8;
        };
    };
} max30102_part_id_reg_t;


/*GENERIC REGISTER TYPE*/
typedef struct {
    uint8_t reg_addr;
    uint8_t reg_val;
} max30102_generic_register_t;

typedef struct {
    max30102_intr_status_1_reg_t intr_status_1_reg;
    max30102_intr_status_2_reg_t intr_status_2_reg;
    max30102_intr_enable_1_reg_t intr_enable_1_reg;
    max30102_intr_enable_2_reg_t intr_enable_2_reg;
    max30102_fifo_wr_ptr_reg_t fifo_wr_ptr_reg;
    max30102_ovf_counter_reg_t overflow_counter_reg;
    max30102_fifo_rd_ptr_reg_t fifo_rd_ptr_reg;
    max30102_fifo_data_reg_t fifo_data_reg;
    max30102_fifo_config_reg_t fifo_config_reg;
    max30102_mode_config_reg_t mode_config_reg;
    max30102_spo2_config_reg_t spo2_config_reg;
    max30102_led1_pa_reg_t led1_pa_reg;
    max30102_led2_pa_reg_t led2_pa_reg;
    max30102_prox_mode_led_reg_t prox_mode_led_reg;
    max30102_multi_led_ctrl1_reg_t multi_led_ctrl1_reg;
    max30102_multi_led_ctrl2_reg_t multi_led_ctrl2_reg;
    max30102_die_temp_int_reg_t die_temp_int_reg;
    max30102_die_temp_frac_reg_t die_temp_frac_reg;
    max30102_die_temp_config_reg_t die_temp_config_reg;
    max30102_prox_intr_thresh_reg_t prox_intr_thresh_reg;
    max30102_rev_id_reg_t revision_id_reg;
    max30102_part_id_reg_t part_id_reg;
} max30102_registers_t;


/*MAX30102 register flags definitions*/
/*Power Ready Flag*/
#define INTERRUPT_STATUS1_PWR_RDY_Pos    (0U)
#define INTERRUPT_STATUS1_PWR_RDY_Msk    (1U << INTERRUPT_STATUS1_PWR_RDY_Pos)
#define INTERRUPT_STATUS1_PWR_RDY        INTERRUPT_STATUS1_PWR_RDY_Msk

/*Proximity Threshold Triggered*/
#define INTERRUPT_STATUS1_PROX_INT_Pos   (4U)
#define INTERRUPT_STATUS1_PROX_INT_Msk   (1U << INTERRUPT_STATUS1_PROX_INT_Pos)
#define INTERRUPT_STATUS1_PROX_INT       INTERRUPT_STATUS1_PROX_INT_Msk

/*Ambient Light Cancellation Overflow*/
#define INTERRUPT_STATUS1_ALC_OVF_Pos    (5U)    
#define INTERRUPT_STATUS1_ALC_OVF_Msk    (1U << INTERRUPT_STATUS1_ALC_OVF_Pos)
#define INTERRUPT_STATUS1_ALC_OVF        INTERRUPT_STATUS1_ALC_OVF_Msk

/*New FIFO Data Ready*/
#define INTERRUPT_STATUS1_PPG_RDY_Pos    (6U)
#define INTERRUPT_STATUS1_PPG_RDY_Msk    (1U << INTERRUPT_STATUS1_PPG_RDY_Pos)  
#define INTERRUPT_STATUS1_PPG_RDY        INTERRUPT_STATUS1_PPG_RDY_Msk

/*FIFO Almost Full Flag*/
#define INTERRUPT_STATUS1_A_FULL_Pos     (7U)
#define INTERRUPT_STATUS1_A_FULL_Msk     (1U << INTERRUPT_STATUS1_A_FULL_Pos)
#define INTERRUPT_STATUS1_A_FULL         INTERRUPT_STATUS1_A_FULL_Msk    

/*Internal Temperature Ready Flag*/
#define INTERRUPT_STATUS2_DIE_TEMP_RDY_Pos      (1U)
#define INTERRUPT_STATUS2_DIE_TEMP_RDY_Msk      (1U << INTERRUPT_STATUS2_DIE_TEMP_RDY_Pos)
#define INTERRUPT_STATUS2_DIE_TEMP_RDY          INTERRUPT_STATUS2_DIE_TEMP_RDY_Msk

#define INTERRUPT_ENABLE1_PROX_INT_EN_Pos       (4U)
#define INTERRUPT_ENABLE1_PROX_INT_EN_Msk       (1U << INTERRUPT_ENABLE1_PROX_INT_EN_Pos)
#define INTERRUPT_ENABLE1_PROX_INT_EN           INTERRUPT_ENABLE1_PROX_INT_EN_Msk

#define INTERRUPT_ENABLE1_ALC_OVF_EN_Pos        (5U)
#define INTERRUPT_ENABLE1_ALC_OVF_EN_Msk        (1U << INTERRUPT_ENABLE1_ALC_OVF_EN_Pos)
#define INTERRUPT_ENABLE1_ALC_OVF_EN            INTERRUPT_ENABLE1_ALC_OVF_EN_Msk

#define INTERRUPT_ENABLE1_PPG_RDY_EN_Pos        (6U)
#define INTERRUPT_ENABLE1_PPG_RDY_EN_Msk        (1U << INTERRUPT_ENABLE1_PPG_RDY_EN_Pos)
#define INTERRUPT_ENABLE1_PPG_RDY_EN            INTERRUPT_ENABLE1_PPG_RDY_EN_Msk    

#define INTERRUPT_ENABLE1_A_FULL_EN_Pos         (7U)
#define INTERRUPT_ENABLE1_A_FULL_EN_Msk         (1U << INTERRUPT_ENABLE1_A_FULL_EN_Pos)
#define INTERRUPT_ENABLE1_A_FULL_EN             INTERRUPT_ENABLE1_A_FULL_EN_Msk

#define INTERRUPT_ENABLE2_DIE_TEMP_RDY_EN_Pos   (1U)
#define INTERRUPT_ENABLE2_DIE_TEMP_RDY_EN_Msk   (1U << INTERRUPT_ENABLE2_DIE_TEMP_RDY_EN_Pos)
#define INTERRUPT_ENABLE2_DIE_TEMP_RDY_EN       INTERRUPT_ENABLE2_DIE_TEMP_RDY_EN_Msk

/*FIFO Bit Masks*/
#define FIFO_WR_PTR_Msk           (0x1FU)
#define FIFO_WR_PTR               FIFO_WR_PTR_Msk

#define FIFO_OVF_COUNTER_Msk      (0x1FU)
#define FIFO_OVF_COUNTER          FIFO_OVF_COUNTER_Msk

#define FIFO_RD_PTR_Msk           (0x1FU)
#define FIFO_RD_PTR               FIFO_RD_PTR_Msk

#define FIFO_DATA_Msk             (0xFFU)
#define FIFO_DATA                 FIFO_DATA_Msk

/*Configuration flags*/
#define FIFO_CONFIG_FIFO_A_FULL_Pos      (0U)
#define FIFO_CONFIG_FIFO_A_FULL_Msk      (0xFU << FIFO_CONFIG_FIFO_A_FULL_Pos)
#define FIFO_CONFIG_FIFO_A_FULL          FIFO_CONFIG_FIFO_A_FULL_Msk

#define FIFO_CONFIG_ROLL_OVER_EN_Pos     (4U)
#define FIFO_CONFIG_ROLL_OVER_EN_Msk     (1U << FIFO_CONFIG_ROLL_OVER_EN_Pos)
#define FIFO_CONFIG_ROLL_OVER_EN         FIFO_CONFIG_ROLL_OVER_EN_Msk

#define FIFO_CONFIG_SMP_AVE_Pos          (5U)
#define FIFO_CONFIG_SMP_AVE_Msk          (0x7U << FIFO_CONFIG_SMP_AVE_Pos)
#define FIFO_CONFIG_SMP_AVE              FIFO_CONFIG_SMP_AVE_Msk

#define MODE_CONFIG_MODE_Pos             (0U)
#define MODE_CONFIG_MODE_Msk             (0x7U << MODE_CONFIG_MODE_Pos)             
#define MODE_CONFIG_MODE                 MODE_CONFIG_MODE_Msk

#define MODE_CONFIG_RESET_Pos            (6U)
#define MODE_CONFIG_RESET_Msk            (1U << MODE_CONFIG_RESET_Pos)
#define MODE_CONFIG_RESET                MODE_CONFIG_RESET_Msk

#define MODE_CONFIG_SHDN_Pos             (7U)
#define MODE_CONFIG_SHDN_Msk             (1U << MODE_CONFIG_SHDN_Pos)
#define MODE_CONFIG_SHDN                 MODE_CONFIG_SHDN_Msk

#define SPO2_CONFIG_LED_PW_Pos           (0U)
#define SPO2_CONFIG_LED_PW_Msk           (3U << SPO2_CONFIG_LED_PW_Pos)
#define SPO2_CONFIG_LED_PW               SPO2_CONFIG_LED_PW_Msk

#define SPO2_CONFIG_SPO2_SR_Pos          (2U)
#define SPO2_CONFIG_SPO2_SR_Msk          (0x7U << SPO2_CONFIG_SPO2_SR_Pos)
#define SPO2_CONFIG_SPO2_SR              SPO2_CONFIG_SPO2_SR_Msk

#define SPO2_CONFIG_SPO2_ADC_RGE_Pos     (5U)
#define SPO2_CONFIG_SPO2_ADC_RGE_Msk     (3U << SPO2_CONFIG_SPO2_ADC_RGE_Pos)
#define SPO2_CONFIG_SPO2_ADC_RGE         SPO2_CONFIG_SPO2_ADC_RGE_Msk

#define LED_CONFIG_LED1_PA_Msk           (0xFFU)
#define LED_CONFIG_LED1_PA               LED_CONFIG_LED1_PA_Msk

#define LED_CONFIG_LED2_PA_Msk           (0xFFU)
#define LED_CONFIG_LED2_PA               LED_CONFIG_LED2_PA_Msk

#define PROXIMITY_MODE_PILOT_PA_Msk      (0xFFU)
#define PROXIMITY_MODE_PILOT_PA          PROXIMITY_MODE_PILOT_PA_Msk

#define MULTI_LED_MODE_SLOT1_Pos         (0U)
#define MULTI_LED_MODE_SLOT1_Msk         (0x7U << MULTI_LED_MODE_SLOT1_Pos)
#define MULTI_LED_MODE_SLOT1             MULTI_LED_MODE_SLOT1_Msk

#define MULTI_LED_MODE_SLOT2_Pos         (4U)
#define MULTI_LED_MODE_SLOT2_Msk         (0x7U << MULTI_LED_MODE_SLOT2_Pos)
#define MULTI_LED_MODE_SLOT2             MULTI_LED_MODE_SLOT2_Msk

#define MULTI_LED_MODE_SLOT3_Pos         (0U)
#define MULTI_LED_MODE_SLOT3_Msk         (0x7U << MULTI_LED_MODE_SLOT3_Pos)
#define MULTI_LED_MODE_SLOT3             MULTI_LED_MODE_SLOT3_Msk

#define MULTI_LED_MODE_SLOT4_Pos         (4U)
#define MULTI_LED_MODE_SLOT4_Msk         (0x7U << MULTI_LED_MODE_SLOT4_Pos)
#define MULTI_LED_MODE_SLOT4             MULTI_LED_MODE_SLOT4_Msk

/*Die Temperature*/
#define DIE_TEMP_INT_Msk                 (0xFFU)
#define DIE_TEMP_INT                     DIE_TEMP_INT_Msk

#define DIE_TEMP_FRAC_Msk                (0xFU << 0U)
#define DIE_TEMP_FRAC                    DIE_TEMP_FRAC_Msk

#define DIE_TEMP_CONFIG_TEMP_EN_Msk      (1U << 0U)
#define DIE_TEMP_CONFIG_TEMP_EN          DIE_TEMP_CONFIG_TEMP_EN_Msk

/*Proximity interrupt threshold*/
#define PROX_INT_THRESH_Msk              (0xFFU)
#define PROX_INT_THRESH                  PROX_INT_THRESH_Msk

#endif /*_MAX30102_REG_DEFS_H_*/