#ifndef _ESP_ADC_H
#define _ESP_ADC_H

#include <cstdint>

#define LUT_ENABLED             1               //ESP configuration, all new esp32 chips should support it

/* ----------------------- Raw to Voltage Constants ------------------------- */
#define LIN_COEFF_A_SCALE               65536
#define LIN_COEFF_A_ROUND               (LIN_COEFF_A_SCALE/2)

#define LUT_VREF_LOW                    1000
#define LUT_VREF_HIGH                   1200
#define LUT_ADC_STEP_SIZE               64
#define LUT_POINTS                      20
#define LUT_LOW_THRESH                  2880
#define LUT_HIGH_THRESH                 (LUT_LOW_THRESH + LUT_ADC_STEP_SIZE)
#define ADC_12_BIT_RES                  4096

/**
 * @brief ADC unit enumeration.
 *
 * @note  For ADC digital controller (DMA mode), ESP32 doesn't support `ADC_UNIT_2`, `ADC_UNIT_BOTH`, `ADC_UNIT_ALTER`.
 */
typedef enum {
    ADC_UNIT_1 = 1,          /*!< SAR ADC 1. */
    ADC_UNIT_2 = 2,          /*!< SAR ADC 2. */
    ADC_UNIT_BOTH = 3,       /*!< SAR ADC 1 and 2. */
    ADC_UNIT_ALTER = 7,      /*!< SAR ADC 1 and 2 alternative mode. */
    ADC_UNIT_MAX,
} adc_unit_t;

/**
 * @brief ADC channels handle. See ``adc1_channel_t``, ``adc2_channel_t``.
 *
 * @note  For ESP32 ADC1, don't use `ADC_CHANNEL_8`, `ADC_CHANNEL_9`. See ``adc1_channel_t``.
 */
typedef enum {
    ADC_CHANNEL_0 = 0, /*!< ADC channel */
    ADC_CHANNEL_1,     /*!< ADC channel */
    ADC_CHANNEL_2,     /*!< ADC channel */
    ADC_CHANNEL_3,     /*!< ADC channel */
    ADC_CHANNEL_4,     /*!< ADC channel */
    ADC_CHANNEL_5,     /*!< ADC channel */
    ADC_CHANNEL_6,     /*!< ADC channel */
    ADC_CHANNEL_7,     /*!< ADC channel */
    ADC_CHANNEL_8,     /*!< ADC channel */
    ADC_CHANNEL_9,     /*!< ADC channel */
    ADC_CHANNEL_MAX,
} adc_channel_t;

/**
 * @brief ADC attenuation parameter. Different parameters determine the range of the ADC. See ``adc1_config_channel_atten``.
 */
typedef enum {
    ADC_ATTEN_DB_0   = 0,  /*!<No input attenumation, ADC can measure up to approx. 800 mV. */
    ADC_ATTEN_DB_2_5 = 1,  /*!<The input voltage of ADC will be attenuated, extending the range of measurement to up to approx. 1100 mV. */
    ADC_ATTEN_DB_6   = 2,  /*!<The input voltage of ADC will be attenuated, extending the range of measurement to up to  approx. 1350 mV. */
    ADC_ATTEN_DB_11  = 3,  /*!<The input voltage of ADC will be attenuated, extending the range of measurement to up to  approx. 2600 mV. */
    ADC_ATTEN_MAX,
} adc_atten_t;

/**
 * @brief ADC resolution setting option.
 *
 * @note  For ESP32-S2. Only 13 bit resolution is supported.
 *        For ESP32.   13 bit resolution is not supported.
 */
typedef enum {
    ADC_WIDTH_BIT_9  = 0, /*!< ADC capture width is 9Bit. Only ESP32 is supported. */
    ADC_WIDTH_BIT_10 = 1, /*!< ADC capture width is 10Bit. Only ESP32 is supported. */
    ADC_WIDTH_BIT_11 = 2, /*!< ADC capture width is 11Bit. Only ESP32 is supported. */
    ADC_WIDTH_BIT_12 = 3, /*!< ADC capture width is 12Bit. Only ESP32 is supported. */
    ADC_WIDTH_MAX,
} adc_bits_width_t;

/**
 * @brief Structure storing characteristics of an ADC
 *
 * @note Call esp_adc_cal_characterize() to initialize the structure
 */
typedef struct {
    adc_unit_t adc_num;                     /**< ADC number*/
    adc_atten_t atten;                      /**< ADC attenuation*/
    adc_bits_width_t bit_width;             /**< ADC bit width */
    uint32_t coeff_a;                       /**< Gradient of ADC-Voltage curve*/
    uint32_t coeff_b;                       /**< Offset of ADC-Voltage curve*/
    uint32_t vref;                          /**< Vref used by lookup table*/
    const uint32_t *low_curve;              /**< Pointer to low Vref curve of lookup table (NULL if unused)*/
    const uint32_t *high_curve;             /**< Pointer to high Vref curve of lookup table (NULL if unused)*/
} esp_adc_cal_characteristics_t;

//20 Point lookup tables, covering ADC readings from 2880 to 4096, step size of 64
//20 Point lookup tables, covering ADC readings from 2880 to 4096, step size of 64
static const uint32_t lut_adc1_low[LUT_POINTS] = {2240, 2297, 2352, 2405, 2457, 2512, 2564, 2616, 2664, 2709,
                                                  2754, 2795, 2832, 2868, 2903, 2937, 2969, 3000, 3030, 3060};
static const uint32_t lut_adc1_high[LUT_POINTS] = {2667, 2706, 2745, 2780, 2813, 2844, 2873, 2901, 2928, 2956,
                                                   2982, 3006, 3032, 3059, 3084, 3110, 3135, 3160, 3184, 3209};
static const uint32_t lut_adc2_low[LUT_POINTS] = {2238, 2293, 2347, 2399, 2451, 2507, 2561, 2613, 2662, 2710,
                                                  2754, 2792, 2831, 2869, 2904, 2937, 2968, 2999, 3029, 3059};
static const uint32_t lut_adc2_high[LUT_POINTS] = {2657, 2698, 2738, 2774, 2807, 2838, 2867, 2894, 2921, 2946,
                                                   2971, 2996, 3020, 3043, 3067, 3092, 3116, 3139, 3162, 3185};

uint32_t esp_adc_cal_raw_to_voltage(uint32_t adc_reading, const esp_adc_cal_characteristics_t *chars);

#endif