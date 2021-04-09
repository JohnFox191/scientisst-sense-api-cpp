#include <cstdlib>
#include <cstdio>
#include "esp_adc.h"

#define ADC_CAL_CHECK(cond, ret) ({                                         \
            if(!(cond)){                                                    \
                return ret;                                                 \
            }                                                               \
})

/* ------------------------ Characterization Constants ---------------------- */
static const uint32_t adc1_tp_atten_scale[4] = {65504, 86975, 120389, 224310};
static const uint32_t adc2_tp_atten_scale[4] = {65467, 86861, 120416, 224708};
static const uint32_t adc1_tp_atten_offset[4] = {0, 1, 27, 54};
static const uint32_t adc2_tp_atten_offset[4] = {0, 9, 26, 66};

static const uint32_t adc1_vref_atten_scale[4] = {57431, 76236, 105481, 196602};
static const uint32_t adc2_vref_atten_scale[4] = {57236, 76175, 105678, 197170};
static const uint32_t adc1_vref_atten_offset[4] = {75, 78, 107, 142};
static const uint32_t adc2_vref_atten_offset[4] = {63, 66, 89, 128};

static inline uint32_t interpolate_two_points(uint32_t y1, uint32_t y2, uint32_t x_step, uint32_t x){
    //Interpolate between two points (x1,y1) (x2,y2) between 'lower' and 'upper' separated by 'step'
    return ((y1 * x_step) + (y2 * x) - (y1 * x) + (x_step / 2)) / x_step;
}


static uint32_t calculate_voltage_linear(uint32_t adc_reading, uint32_t coeff_a, uint32_t coeff_b){
    //Where voltage = coeff_a * adc_reading + coeff_b
    return (((coeff_a * adc_reading) + LIN_COEFF_A_ROUND) / LIN_COEFF_A_SCALE) + coeff_b;
}

//Only call when ADC reading is above threshold
static uint32_t calculate_voltage_lut(uint32_t adc, uint32_t vref, const uint32_t *low_vref_curve, const uint32_t *high_vref_curve){
    //Get index of lower bound points of LUT
    uint32_t i = (adc - LUT_LOW_THRESH) / LUT_ADC_STEP_SIZE;

    //Let the X Axis be Vref, Y axis be ADC reading, and Z be voltage
    int x2dist = LUT_VREF_HIGH - vref;                 //(x2 - x)
    int x1dist = vref - LUT_VREF_LOW;                  //(x - x1)
    int y2dist = ((i + 1) * LUT_ADC_STEP_SIZE) + LUT_LOW_THRESH - adc;  //(y2 - y)
    int y1dist = adc - ((i * LUT_ADC_STEP_SIZE) + LUT_LOW_THRESH);        //(y - y1)

    //For points for bilinear interpolation
    int q11 = low_vref_curve[i];                    //Lower bound point of low_vref_curve
    int q12 = low_vref_curve[i + 1];                //Upper bound point of low_vref_curve
    int q21 = high_vref_curve[i];                   //Lower bound point of high_vref_curve
    int q22 = high_vref_curve[i + 1];               //Upper bound point of high_vref_curve

    //Bilinear interpolation
    //Where z = 1/((x2-x1)*(y2-y1)) * ( (q11*x2dist*y2dist) + (q21*x1dist*y2dist) + (q12*x2dist*y1dist) + (q22*x1dist*y1dist) )
    int voltage = (q11 * x2dist * y2dist) + (q21 * x1dist * y2dist) + (q12 * x2dist * y1dist) + (q22 * x1dist * y1dist);
    voltage += ((LUT_VREF_HIGH - LUT_VREF_LOW) * LUT_ADC_STEP_SIZE) / 2; //Integer division rounding
    voltage /= ((LUT_VREF_HIGH - LUT_VREF_LOW) * LUT_ADC_STEP_SIZE);    //Divide by ((x2-x1)*(y2-y1))
    return (uint32_t)voltage;
}


uint32_t esp_adc_cal_raw_to_voltage(uint32_t adc_reading, const esp_adc_cal_characteristics_t *chars){

    //Scale adc_rading if not 12 bits wide
    adc_reading = (adc_reading << (ADC_WIDTH_BIT_12 - chars->bit_width));
    if (adc_reading > ADC_12_BIT_RES - 1) {
        adc_reading = ADC_12_BIT_RES - 1;    //Set to 12bit res max
    }

    if (LUT_ENABLED && (chars->atten == ADC_ATTEN_DB_11) && (adc_reading >= LUT_LOW_THRESH)) {  //Check if in non-linear region
        //Use lookup table to get voltage in non linear portion of ADC_ATTEN_DB_11
        uint32_t lut_voltage = calculate_voltage_lut(adc_reading, chars->vref, chars->low_curve, chars->high_curve);
        if (adc_reading <= LUT_HIGH_THRESH) {   //If ADC is transitioning from linear region to non-linear region
            //Linearly interpolate between linear voltage and lut voltage
            uint32_t linear_voltage = calculate_voltage_linear(adc_reading, chars->coeff_a, chars->coeff_b);
            return interpolate_two_points(linear_voltage, lut_voltage, LUT_ADC_STEP_SIZE, (adc_reading - LUT_LOW_THRESH));
        } else {
            return lut_voltage;
        }
    } else {
        return calculate_voltage_linear(adc_reading, chars->coeff_a, chars->coeff_b);
    }
}