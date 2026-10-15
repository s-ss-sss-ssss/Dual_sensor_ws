# AK affine input is in uT (signed RAW * 1.1 * raw_scale).
# TMAG physical conversion is TODO; output units also depend on affine coefficients.
uint8 sensor_id
uint32 read_done_offset_us
# Hardware addressing is retained without inferring logical IDs.
uint8 channel
uint8 address
float64 x
float64 y
float64 z
uint8 drdy
uint8 err
uint8 dor
uint8 overflow
