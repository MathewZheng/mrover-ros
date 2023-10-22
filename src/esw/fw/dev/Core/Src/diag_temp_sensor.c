////////////////////////////////
//      TMP6431DECR  Temperature Sensor Nucleo Hardware Interface
//      Written by:
//      Jess Wu
//      jessyw@umich.edu
////////////////////////////////

#include "diag_temp_sensor.h"


// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created current sensor object
DiagTempSensor* new_diag_temp_sensor(ADCSensor* adc_sensor, int channel) {
    DiagTempSensor* temp_sensor = (DiagTempSensor*) malloc(sizeof(DiagTempSensor));
    temp_sensor->adc_sensor = adc_sensor;
    temp_sensor->channel = channel;
    temp_sensor->temp = 0;

    return temp_sensor;
}

// REQUIRES: valid temp sensor
// MODIFIES: stored sensor value
// EFFECTS: updates the sensor value
void update_diag_temp_sensor_val(DiagTempSensor* sensor) {
	float measured_voltage = get_adc_sensor_value(sensor->adc_sensor, sensor->channel) * 3.3f / 4096.0f;
    sensor->temp = (THRM_A4 * powf(measured_voltage,4)) + (THRM_A3 * powf(measured_voltage,3)) + (THRM_A2 * powf(measured_voltage,2)) + (THRM_A1 *  measured_voltage) + THRM_A0;
}

// REQUIRES: valid temp sensor
// MODIFIES: nothing
// EFFECTS: returns the stored value for amps
float get_diag_temp_sensor_val(DiagTempSensor* sensor) {
    return sensor->temp;
}
