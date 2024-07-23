#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#define led_green	2
#define led_blue	3
#define button      16
#define led         25

///////////////////////////////////// RECEIVER SECTION START /////////////////////////////////////

#define REC_CHANNELS    	6
#define rec_ch1          	16
#define rec_ch2          	17
#define rec_ch3          	18
#define rec_ch4          	19
#define rec_ch5          	20
#define rec_ch6          	21

#define rec_yaw				0
#define rec_pitch			1
#define rec_throttle		2
#define rec_roll			3

uint8_t tx_conn = 0;

uint8_t rec_ch_state[REC_CHANNELS] 		= {0};
volatile long rec_ch_rise[REC_CHANNELS] = {0};
int rec_ch[REC_CHANNELS] 				= {0};

void rec_capture()
{
	tx_conn = 1;

	uint32_t current_time = time_us_32();

	//////////////////////////// CH1 ////////////////////////////
	if(sio_hw->gpio_in & (1 << rec_ch1))
	{
		if(rec_ch_state[0] == 0)
		{
			rec_ch_state[0] = 1;
			rec_ch_rise[0] = current_time;
		}
	}
	else if(rec_ch_state[0] == 1)
	{
		rec_ch_state[0] = 0;
		rec_ch[0] = current_time - rec_ch_rise[0];
	}

	//////////////////////////// CH2 ////////////////////////////
	if(sio_hw->gpio_in & (1 << rec_ch2))
	{
		if(rec_ch_state[1] == 0)
		{
			rec_ch_state[1] = 1;
			rec_ch_rise[1] = current_time;
		}
	}
	else if(rec_ch_state[1] == 1)
	{
		rec_ch_state[1] = 0;
		rec_ch[1] = current_time - rec_ch_rise[1];
	}

	//////////////////////////// CH3 ////////////////////////////
	if(sio_hw->gpio_in & (1 << rec_ch3))
	{
		if(rec_ch_state[2] == 0)
		{
			rec_ch_state[2] = 1;
			rec_ch_rise[2] = current_time;
		}
	}
	else if(rec_ch_state[2] == 1)
	{
		rec_ch_state[2] = 0;
		rec_ch[2] = current_time - rec_ch_rise[2];
	}

	//////////////////////////// CH4 ////////////////////////////
	if(sio_hw->gpio_in & (1 << rec_ch4))
	{
		if(rec_ch_state[3] == 0)
		{
			rec_ch_state[3] = 1;
			rec_ch_rise[3] = current_time;
		}
	}
	else if(rec_ch_state[3] == 1)
	{
		rec_ch_state[3] = 0;
		rec_ch[3] = current_time - rec_ch_rise[3];
	}

	//////////////////////////// CH5 ////////////////////////////
	if(sio_hw->gpio_in & (1 << rec_ch5))
	{
		if(rec_ch_state[4] == 0)
		{
			rec_ch_state[4] = 1;
			rec_ch_rise[4] = current_time;
		}
	}
	else if(rec_ch_state[4] == 1)
	{
		rec_ch_state[4] = 0;
		rec_ch[4] = current_time - rec_ch_rise[4];
	}

	//////////////////////////// CH6 ////////////////////////////
	if(sio_hw->gpio_in & (1 << rec_ch6))
	{
		if(rec_ch_state[5] == 0)
		{
			rec_ch_state[5] = 1;
			rec_ch_rise[5] = current_time;
		}
	}
	else if(rec_ch_state[5] == 1)
	{
		rec_ch_state[5] = 0;
		rec_ch[5] = current_time - rec_ch_rise[5];
	}
}

void print_rec_channels(void)
{
    printf("RECEIVER: CH1: %d  CH2: %d  CH3: %d  CH4: %d  CH5: %d  CH6: %d\n", rec_ch[0], rec_ch[1], rec_ch[2], rec_ch[3], rec_ch[4], rec_ch[5]);
}

void rec_init(void)
{
	gpio_set_irq_enabled_with_callback(rec_ch1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &rec_capture);
    gpio_set_irq_enabled_with_callback(rec_ch2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &rec_capture);
    gpio_set_irq_enabled_with_callback(rec_ch3, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &rec_capture);
    gpio_set_irq_enabled_with_callback(rec_ch4, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &rec_capture);
    gpio_set_irq_enabled_with_callback(rec_ch5, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &rec_capture);
    gpio_set_irq_enabled_with_callback(rec_ch6, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &rec_capture);
}

///////////////////////////////////// RECEIVER SECTION END /////////////////////////////////////

///////////////////////////////////// IMU SECTION BEGIN /////////////////////////////////////

/////////////////// IMU REGISTER MAP START ///////////////////

#define MPU_SELF_TEST_X_GYRO    0x00
#define MPU_SELF_TEST_Y_GYRO    0x01
#define MPU_SELF_TEST_Z_GYRO    0x02
#define MPU_SELF_TEST_X_ACCEL   0x0D
#define MPU_SELF_TEST_Y_ACCEL   0x0E
#define MPU_SELF_TEST_Z_ACCEL   0x0F
#define MPU_XG_OFFSET_H         0x13
#define MPU_XG_OFFSET_L         0x14
#define MPU_YG_OFFSET_H         0x15
#define MPU_YG_OFFSET_L         0x16
#define MPU_ZG_OFFSET_H         0x17
#define MPU_ZG_OFFSET_L         0x18
#define MPU_SMPLRT_DIV          0x19
#define MPU_CONFIG              0x1A
#define MPU_GYRO_CONFIG         0x1B
#define MPU_ACCEL_CONFIG        0x1C
#define MPU_ACCEL_CONFIG2       0x1D
#define MPU_LP_ACCEL_ODR        0x1E
#define MPU_WOM_THR             0x1F
#define MPU_FIFO_EN             0x23
#define MPU_I2C_MST_CTRL        0x24
#define MPU_I2C_SLV0_ADDR       0x25
#define MPU_I2C_SLV0_REG        0x26
#define MPU_I2C_SLV0_CTRL       0x27
#define MPU_I2C_SLV1_ADDR       0x28
#define MPU_I2C_SLV1_REG        0x29
#define MPU_I2C_SLV1_CTRL       0x2A
#define MPU_I2C_SLV2_ADDR       0x2B
#define MPU_I2C_SLV2_REG        0x2C
#define MPU_I2C_SLV2_CTRL       0x2D
#define MPU_I2C_SLV3_ADDR       0x2E
#define MPU_I2C_SLV3_REG        0x2F
#define MPU_I2C_SLV3_CTRL       0x30
#define MPU_I2C_SLV4_ADDR       0x31
#define MPU_I2C_SLV4_REG        0x32
#define MPU_I2C_SLV4_DO         0x33
#define MPU_I2C_SLV4_CTRL       0x34
#define MPU_I2C_SLV4_DI         0x35
#define MPU_I2C_MST_STATUS      0x36
#define MPU_INT_PIN_CFG         0x37
#define MPU_INT_ENABLE          0x38
#define MPU_INT_STATUS          0x3A
#define MPU_ACCEL_XOUT_H        0x3B
#define MPU_ACCEL_XOUT_L        0x3C
#define MPU_ACCEL_YOUT_H        0x3D
#define MPU_ACCEL_YOUT_L        0x3E
#define MPU_ACCEL_ZOUT_H        0x3F
#define MPU_ACCEL_ZOUT_L        0x40
#define MPU_TEMP_OUT_H          0x41
#define MPU_TEMP_OUT_L          0x42
#define MPU_GYRO_XOUT_H         0x43
#define MPU_GYRO_XOUT_L         0x44
#define MPU_GYRO_YOUT_H         0x45
#define MPU_GYRO_YOUT_L         0x46
#define MPU_GYRO_ZOUT_H         0x47
#define MPU_GYRO_ZOUT_L         0x48
#define MPU_EXT_SENS_DATA_00    0x49
#define MPU_EXT_SENS_DATA_01    0x4A
#define MPU_EXT_SENS_DATA_02    0x4B
#define MPU_EXT_SENS_DATA_03    0x4C
#define MPU_EXT_SENS_DATA_04    0x4D
#define MPU_EXT_SENS_DATA_05    0x4E
#define MPU_EXT_SENS_DATA_06    0x4F
#define MPU_EXT_SENS_DATA_07    0x50
#define MPU_EXT_SENS_DATA_08    0x51
#define MPU_EXT_SENS_DATA_09    0x52
#define MPU_EXT_SENS_DATA_10    0x53
#define MPU_EXT_SENS_DATA_11    0x54
#define MPU_EXT_SENS_DATA_12    0x55
#define MPU_EXT_SENS_DATA_13    0x56
#define MPU_EXT_SENS_DATA_14    0x57
#define MPU_EXT_SENS_DATA_15    0x58
#define MPU_EXT_SENS_DATA_16    0x59
#define MPU_EXT_SENS_DATA_17    0x5A
#define MPU_EXT_SENS_DATA_18    0x5B
#define MPU_EXT_SENS_DATA_19    0x5C
#define MPU_EXT_SENS_DATA_20    0x5D
#define MPU_EXT_SENS_DATA_21    0x5E
#define MPU_EXT_SENS_DATA_22    0x5F
#define MPU_EXT_SENS_DATA_23    0x60
#define MPU_I2C_SLV0_DO         0x63
#define MPU_I2C_SLV1_DO         0x64
#define MPU_I2C_SLV2_DO         0x65
#define MPU_I2C_SLV3_DO         0x66
#define MPU_I2C_MST_DELAY_CTRL  0x67
#define MPU_SIGNAL_PATH_RESET   0x68
#define MPU_ACCEL_INTEL_CTRL    0x69
#define MPU_USER_CTRL           0x6A
#define MPU_PWR_MGMT_1          0x6B
#define MPU_PWR_MGMT_2          0x6C
#define MPU_FIFO_COUNT_H        0x72
#define MPU_FIFO_COUNT_L        0x73
#define MPU_FIFO_R_W            0x74
#define MPU_WHO_AM_I            0x75
#define MPU_XA_OFFSET_H         0x77
#define MPU_XA_OFFSET_L         0x78
#define MPU_YA_OFFSET_H         0x7A
#define MPU_YA_OFFSET_L         0x7B
#define MPU_ZA_OFFSET_H         0x7D
#define MPU_ZA_OFFSET_L         0x7E

/////////////////// IMU REGISTER MAP END ///////////////////

/////////////////// IMU SPI CONFIG START ///////////////////

#define imu_spi     spi1

#define imu_spi_sck     10
#define imu_spi_mosi    11
#define imu_spi_miso    12
#define imu_spi_cs      13

/////////////////// IMU SPI CONFIG END ///////////////////

#define MPU_ID			0x70

#define imu_cs_low()    gpio_put(imu_spi_cs, 0)
#define imu_cs_high()   gpio_put(imu_spi_cs, 1)

#define GYRO_SENS_SCALE_FACTOR	16.4

#define ACCEL_GRAVITY			9.81f

#define ACCEL_SENS_SCALE_FACTOR	16384.0f

#define ACCEL_ERROR_X			0.02f
#define ACCEL_ERROR_Y			-0.01f
#define ACCEL_ERROR_Z			-0.04f

typedef struct
{
	int16_t accel_raw_x;
	int16_t accel_raw_y;
	int16_t accel_raw_z;
	int16_t temp_raw;
	int16_t gyro_raw_x;
	int16_t gyro_raw_y;
	int16_t gyro_raw_z;
}imu_t;

imu_t imu;

float gyro_offset_x = 0;
float gyro_offset_y = 0;
float gyro_offset_z = 0;

void imu_write_reg(uint8_t _reg, uint8_t _data)
{
    imu_cs_low();
    spi_write_blocking(imu_spi, &_reg, 1);
    spi_write_blocking(imu_spi, &_data, 1);
    imu_cs_high();
}

uint8_t imu_read_reg(uint8_t _reg)
{
    uint8_t reg_data = 0x00, dummy = 0xFF;

	uint8_t cmd = _reg | 0x80;

    imu_cs_low();
    spi_write_blocking(imu_spi, &cmd, 1);
    spi_read_blocking(imu_spi, dummy, &reg_data, 1);
    imu_cs_high();

    return reg_data;
}

uint8_t imu_getID(void)
{
	uint8_t id = 0x00;

	imu_cs_low();
	id = imu_read_reg(MPU_WHO_AM_I);
	imu_cs_high();

	return id;
}

void imu_init(void)
{
	gpio_init(imu_spi_cs);
    gpio_set_dir(imu_spi_cs, GPIO_OUT);
    gpio_put(imu_spi_cs, 1);

    spi_init(imu_spi, 1000 * 1000);

    spi_set_format(imu_spi, 8, 0, 0, SPI_MSB_FIRST);

	gpio_set_function(imu_spi_sck, GPIO_FUNC_SPI);
    gpio_set_function(imu_spi_mosi, GPIO_FUNC_SPI);
    gpio_set_function(imu_spi_miso, GPIO_FUNC_SPI);

	sleep_ms(100);

	imu_cs_low();
	imu_write_reg(MPU_PWR_MGMT_1, 0x80);			//Soft Reset
	sleep_ms(100);
	imu_write_reg(MPU_SIGNAL_PATH_RESET, 0x07);		//Gyro Reset, Accel Reset, Temp Reset
	sleep_ms(100);
	imu_write_reg(MPU_PWR_MGMT_1, 0x06);			//CLK: Internal 20MHz
	sleep_ms(50);
	imu_write_reg(MPU_USER_CTRL, 0x10);				//Disable I2C Mode
	sleep_ms(50);
	imu_cs_high();

	imu_cs_low();
	imu_write_reg(MPU_ACCEL_CONFIG, 0x00);			//Accel Full Scale Select: +-2g
	imu_cs_high();

	imu_cs_low();
	imu_write_reg(MPU_SIGNAL_PATH_RESET, 0x07);		//Reset Signal Paths
	imu_cs_high();

	imu_cs_low();
	imu_write_reg(MPU_GYRO_CONFIG, 0x18);			//Gyro Full Scale Select: +=2000dps
	imu_cs_high();

	sleep_ms(50);
}

void imu_get_data_blocking(void)
{
    uint8_t dummy = 0xFF;
	uint8_t cmd = MPU_ACCEL_XOUT_H | 0x80;
	uint8_t buffer[14] = {0};

    imu_cs_low();
	spi_write_blocking(imu_spi, &cmd, 1);
	spi_read_blocking(imu_spi, dummy, &buffer[0], 14);
	imu_cs_high();

	imu.accel_raw_x = buffer[0] << 8  | buffer[1];
	imu.accel_raw_y = buffer[2] << 8  | buffer[3];
	imu.accel_raw_z = buffer[4] << 8  | buffer[5];
	imu.temp_raw	= buffer[6] << 8  | buffer[7];
	imu.gyro_raw_x  = buffer[8] << 8  | buffer[9];
	imu.gyro_raw_y  = buffer[10] << 8 | buffer[11];
	imu.gyro_raw_z  = buffer[12] << 8 | buffer[13];
}

void imu_calibrate_gyro(void)
{
	int samples = 2000;

	float sum_x = 0, sum_y = 0, sum_z = 0;
	float avg_x = 0, avg_y = 0, avg_z = 0;

	for(int i=0; i<samples; i++)
	{
		imu_get_data_blocking();

		float curr_reading_x = (float)imu.gyro_raw_x / (float)GYRO_SENS_SCALE_FACTOR;
		float curr_reading_y = (float)imu.gyro_raw_y / (float)GYRO_SENS_SCALE_FACTOR;
		float curr_reading_z = (float)imu.gyro_raw_z / (float)GYRO_SENS_SCALE_FACTOR;

		sum_x += curr_reading_x;
		sum_y += curr_reading_y;
		sum_z += curr_reading_z;
	}
	avg_x = (float)sum_x / (float)samples;
	avg_y = (float)sum_y / (float)samples;
	avg_z = (float)sum_z / (float)samples;

	avg_x *= -1;
	avg_y *= -1;
	avg_z *= -1;

	gyro_offset_x = avg_x;
	gyro_offset_y = avg_y;
	gyro_offset_z = avg_z;
}

///////////////////////////////////// IMU SECTION END /////////////////////////////////////

///////////////////////////////////// MOTOR SECTION END /////////////////////////////////////

#define motor1			6		//CW
#define motor2			7		//CCW
#define motor3			8		//CW
#define motor4			9		//CCW

#define throttleIdle	1000

int motor1_signal = 0, motor2_signal = 0, motor3_signal = 0, motor4_signal = 0;

uint slice_num_motor1 = 0, slice_num_motor2 = 0, slice_num_motor3 = 0, slice_num_motor4 = 0;

float desired_rate_pitch = 0, desired_rate_roll = 0, desired_rate_yaw = 0;
int inputThrottle = 0, inputPitch = 0, inputRoll = 0, inputYaw = 0;

void reset_pid();

void calibrate_esc(void)
{
	motor1_signal = rec_ch[rec_throttle];
	motor2_signal = rec_ch[rec_throttle];
	motor3_signal = rec_ch[rec_throttle];
	motor4_signal = rec_ch[rec_throttle];

	pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A, 0.4895 * motor1_signal);
    pwm_set_chan_level(slice_num_motor2, PWM_CHAN_B, 0.4895 * motor2_signal);
    pwm_set_chan_level(slice_num_motor3, PWM_CHAN_A, 0.4895 * motor3_signal);
    pwm_set_chan_level(slice_num_motor4, PWM_CHAN_B, 0.4895 * motor4_signal);
}

void motors_init(void)
{
	gpio_set_function(motor1, GPIO_FUNC_PWM);
	gpio_set_function(motor2, GPIO_FUNC_PWM);
	gpio_set_function(motor3, GPIO_FUNC_PWM);
	gpio_set_function(motor4, GPIO_FUNC_PWM);

	slice_num_motor1 = pwm_gpio_to_slice_num(motor1);
	slice_num_motor2 = pwm_gpio_to_slice_num(motor2);
	slice_num_motor3 = pwm_gpio_to_slice_num(motor3);
	slice_num_motor4 = pwm_gpio_to_slice_num(motor4);

    pwm_set_clkdiv(slice_num_motor1, 256);
    pwm_set_clkdiv(slice_num_motor2, 256);
    pwm_set_clkdiv(slice_num_motor3, 256);
    pwm_set_clkdiv(slice_num_motor4, 256);

    pwm_set_enabled(slice_num_motor1, true);
    pwm_set_enabled(slice_num_motor2, true);
    pwm_set_enabled(slice_num_motor3, true);
    pwm_set_enabled(slice_num_motor4, true);

    pwm_set_wrap(slice_num_motor1, 1224);
    pwm_set_wrap(slice_num_motor2, 1224);
    pwm_set_wrap(slice_num_motor3, 1224);
    pwm_set_wrap(slice_num_motor4, 1224);

    pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A, 489);
    pwm_set_chan_level(slice_num_motor2, PWM_CHAN_B, 489);
    pwm_set_chan_level(slice_num_motor3, PWM_CHAN_A, 489);
    pwm_set_chan_level(slice_num_motor4, PWM_CHAN_B, 489);
}

static inline void control_motors(void)
{
	motor1_signal = inputThrottle + inputRoll - inputPitch - inputYaw;
	motor2_signal = inputThrottle - inputRoll - inputPitch + inputYaw;
	motor3_signal = inputThrottle - inputRoll + inputPitch - inputYaw;
	motor4_signal = inputThrottle + inputRoll + inputPitch + inputYaw;

	if(motor1_signal < 1000) motor1_signal = throttleIdle;
	if(motor2_signal < 1000) motor2_signal = throttleIdle;
	if(motor3_signal < 1000) motor3_signal = throttleIdle;
	if(motor4_signal < 1000) motor4_signal = throttleIdle;

	if(motor1_signal > 2000) motor1_signal = 1999;
	if(motor2_signal > 2000) motor2_signal = 1999;
	if(motor3_signal > 2000) motor3_signal = 1999;
	if(motor4_signal > 2000) motor4_signal = 1999;

	if(inputThrottle < 1120)
	{
		motor1_signal = throttleIdle;
		motor2_signal = throttleIdle;
		motor3_signal = throttleIdle;
		motor4_signal = throttleIdle;
		
		reset_pid();
	}
	pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A, 0.4895 * motor1_signal);
    pwm_set_chan_level(slice_num_motor2, PWM_CHAN_B, 0.4895 * motor2_signal);
    pwm_set_chan_level(slice_num_motor3, PWM_CHAN_A, 0.4895 * motor3_signal);
    pwm_set_chan_level(slice_num_motor4, PWM_CHAN_B, 0.4895 * motor4_signal);
}

///////////////////////////////////// MOTOR SECTION END /////////////////////////////////////

///////////////////////////////////// BATTERY SECTION END /////////////////////////////////////

#define battery_pin				26
#define adc_in0					0

#define battery_voltage_max		12.6
#define battery_voltage_min		11.1

float battery_voltage_current  = 0;
float battery_percentage 	   = 0;

void battery_init(void)
{
	adc_init();

	adc_gpio_init(battery_pin);

	adc_select_input(adc_in0);
}

float get_battery_voltage(void)
{
	float battery_voltage = 0;

	uint16_t adc_val = adc_read();

	battery_voltage = (adc_val * 3.3 / 4096) * 3.86;

	return battery_voltage;
}

///////////////////////////////////// BATTERY SECTION END /////////////////////////////////////

///////////////////////////////////// PID SECTION START /////////////////////////////////////

#define P_Pitch			2.5f
#define I_Pitch			0.3f
#define D_Pitch			0.7f

#define P_Roll			P_Pitch
#define I_Roll			I_Pitch
#define D_Roll			D_Pitch

#define P_Yaw			4.5f
#define I_Yaw			1.2f
#define D_Yaw			2.5f

#define I_term_limit	200

#define refresh_rate	400.0f
#define time_period		1.0f / refresh_rate

float error_rate_pitch = 0, 	  error_rate_roll = 0, 	     error_rate_yaw = 0;
float error_rate_pitch_prev = 0,  error_rate_roll_prev = 0,  error_rate_yaw_prev = 0;
float error_iterm_pitch_prev = 0, error_iterm_roll_prev = 0, error_iterm_yaw_prev = 0;

uint32_t loopTimer = 0;

float PID_Return[2] = {0, 0};			//Prev_Error, I_Term

int pid_equation(float error, float P, float I, float D, float previous_error, float previous_i)
{
	float P_term = P * error;

	float I_term = previous_i + (I * (((error + previous_error) * time_period) / 2.0f));
	if(I_term > I_term_limit) I_term = I_term_limit;
	else if(I_term < -I_term_limit) I_term = -I_term_limit;

	float D_term = D * ((error - previous_error) / time_period);

	int result = P_term + I_term + D_term;

	PID_Return[0] = error;
	PID_Return[1] = I_term;

	return result;
}

void reset_pid(void)
{
	error_rate_pitch_prev = 0;
	error_rate_roll_prev = 0;
	error_rate_yaw_prev = 0;
	error_iterm_pitch_prev = 0;
	error_iterm_roll_prev = 0;
	error_iterm_yaw_prev = 0;
}

///////////////////////////////////// PID SECTION END /////////////////////////////////////

///////////////////////////////////// FLIGHT SECTION START /////////////////////////////////////

uint8_t  armed = 0;
uint32_t arm_time = 0;

uint32_t tx_disconnection_time = 0;

///////////////////////////////////// FLIGHT SECTION START /////////////////////////////////////

int main(void)
{
	stdio_init_all();

	gpio_init(button);
	gpio_set_dir(button, GPIO_IN);

    gpio_init(led);
    gpio_set_dir(led, GPIO_OUT);

	gpio_init(led_blue);
	gpio_init(led_green);

	gpio_set_dir(led_blue, GPIO_OUT);
	gpio_set_dir(led_green, GPIO_OUT);

	sleep_ms(2000);

	gpio_put(led_blue, 1);

	battery_init();

	rec_init();

	motors_init();

	imu_init();

	imu_calibrate_gyro();

 	gpio_put(led_blue, 0);
	
	gpio_put(led_green, 1);
	sleep_ms(250);
	gpio_put(led_green, 0);
	sleep_ms(250);
	gpio_put(led_green, 1);
	sleep_ms(250);
	gpio_put(led_green, 0);
	sleep_ms(250);

	gpio_put(led_green, 1);

	//Check for Transmitter Connection at startup
	while(tx_conn == 0)						//Transmitter Not Connected Warning
	{
		gpio_put(led_blue, 1);
		sleep_ms(100);
		gpio_put(led_blue, 0);
		sleep_ms(100);
	}

	//Check for Battery Voltage at startup
	battery_voltage_current = get_battery_voltage();

	if(battery_voltage_current <= 11.2)
	{
		while(1)							//Low Battery Warning
		{
			gpio_put(led_blue, 1);
			sleep_ms(1000);
			gpio_put(led_blue, 0);
			sleep_ms(1000);
		}
	}
	gpio_put(led_blue, 0);

	//Check for Throttle Stick at startup
	while(rec_ch[rec_throttle] > 1140)		//Throttle Stick Warning
	{
		gpio_put(led_blue, 1);
		sleep_ms(250);
		gpio_put(led_blue, 0);
		sleep_ms(250);
	}
	gpio_put(led_blue, 0);
    
	while(1)
    {
		if(armed == 0)						//This section is used to Arm the Drone
		{
			if(rec_ch[rec_throttle] <= 1140 && rec_ch[rec_yaw] <= 1140) arm_time += 2500;
			else arm_time = 0;
			
			if(arm_time >= 1000000)
			{
				arm_time = 0;

				armed = 1;

				gpio_put(led_green, 0);
				
				while(!(rec_ch[rec_yaw] >= 1450 && rec_ch[rec_yaw] <= 1550));
			}
		}

		if(armed == 1)						//The PID Control Loop
		{
			battery_voltage_current = get_battery_voltage();

			if(battery_voltage_current <= 11.2) gpio_put(led_blue, 1);

			imu_get_data_blocking();

			if(rec_ch[rec_throttle] <= 1140) reset_pid();

			//Read and Convert Receiver Values
			desired_rate_pitch = 0.1 * (rec_ch[rec_pitch] - 1500);
			desired_rate_roll  = 0.1 * (rec_ch[rec_roll]  - 1500);
			inputThrottle = rec_ch[rec_throttle];
			desired_rate_yaw   = 0.1 * (rec_ch[rec_yaw]   - 1500);

			//NOTE: The Values are Inverted to match the Physical Orientation of the Flight Controller on the Drone, Adjust Accordingly

			//Calculate the Current Pitch, Roll and Yaw Rates from Gyro
			float rate_pitch = ((float)imu.gyro_raw_y / GYRO_SENS_SCALE_FACTOR) + gyro_offset_y;
			float rate_roll  = ((float)imu.gyro_raw_x / GYRO_SENS_SCALE_FACTOR) + gyro_offset_x;
			float rate_yaw   = (((float)imu.gyro_raw_z / GYRO_SENS_SCALE_FACTOR) + gyro_offset_z) * -1;

			//Calculate the errors
			error_rate_pitch = desired_rate_pitch - rate_pitch;
			error_rate_roll  = desired_rate_roll  - rate_roll;
			error_rate_yaw   = desired_rate_yaw   - rate_yaw;

			//PID Calculations for Pitch, Roll and Yaw
			inputPitch = pid_equation(error_rate_pitch, P_Pitch, I_Pitch, D_Pitch, error_rate_pitch_prev, error_iterm_pitch_prev);
			error_rate_pitch_prev = PID_Return[0];
			error_iterm_pitch_prev = PID_Return[1];

			inputRoll = pid_equation(error_rate_roll, P_Roll, I_Roll, D_Roll, error_rate_roll_prev, error_iterm_roll_prev);
			error_rate_roll_prev = PID_Return[0];
			error_iterm_roll_prev = PID_Return[1];
			
			inputYaw = pid_equation(error_rate_yaw, P_Yaw, I_Yaw, D_Yaw, error_rate_yaw_prev, error_iterm_yaw_prev);
			error_rate_yaw_prev = PID_Return[0];
			error_iterm_yaw_prev = PID_Return[1];

			control_motors();

			//This section is used to Disarm the Drone
			if(rec_ch[rec_throttle] <= 1145 && rec_ch[rec_yaw] >= 1925) arm_time += 2500;
			else arm_time = 0;
			
			if(arm_time >= 1000000)
			{
				arm_time = 0;
				
				armed = 0;

				gpio_put(led_green, 1);
			}
		}
		else			//If the Drone is Disarmed
		{
			motor1_signal = throttleIdle;
			motor2_signal = throttleIdle;
			motor3_signal = throttleIdle;
			motor4_signal = throttleIdle;

			pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A, 0.4895 * motor1_signal);
			pwm_set_chan_level(slice_num_motor2, PWM_CHAN_B, 0.4895 * motor2_signal);
			pwm_set_chan_level(slice_num_motor3, PWM_CHAN_A, 0.4895 * motor3_signal);
			pwm_set_chan_level(slice_num_motor4, PWM_CHAN_B, 0.4895 * motor4_signal);

			reset_pid();

			if(rec_ch[rec_throttle] >= 1140)			//Throttle Stick Warning
			{
				gpio_put(led_blue, 1);
				sleep_ms(250);
				gpio_put(led_blue, 0);
				sleep_ms(250);
			}
		}
		
		//This Section is used to Check for Connection with the Transmitter
		if(tx_conn == 1 && (sio_hw->gpio_in & 0x003F0000) == 0) tx_disconnection_time += 2500;
		else tx_disconnection_time = 0;

		if(tx_disconnection_time == 250000)				//If Tx Disconnects for 250ms
		{
			tx_conn = 0;
			tx_disconnection_time = 0;

			rec_ch[0] = 0;
			rec_ch[1] = 0;
			rec_ch[2] = 0;
			rec_ch[3] = 0;
			rec_ch[4] = 0;
			rec_ch[5] = 0;
		}

		while((time_us_32() - loopTimer) < 2500);		//Wait till 2500ms have passed

		loopTimer = time_us_32();
    }
}