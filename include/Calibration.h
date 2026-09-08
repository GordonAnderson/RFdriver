#ifndef Calibration_h
#define Calibration_h

// Interactive AD5592 DAC/ADC channel calibration helpers. These prompt over
// the USB serial port for a reference measurement at two set points and
// derive the channel's linear m/b calibration pair from the result. Not
// currently wired to a host command, but available for adding one.
void CalibrateLoop(void);
int  Calibrate5592point(uint8_t SPIcs, DACchan *dacchan, ADCchan *adcchan, float *V);
void Calibrate5592(uint8_t SPIcs, DACchan *dacchan, ADCchan *adcchan, float V1, float V2);

#endif
