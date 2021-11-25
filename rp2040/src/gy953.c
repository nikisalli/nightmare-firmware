#include "gy953.h"

static volatile bool _attention = false;
//static uint8_t _gy953reg_a = 0x08;
//static uint8_t _gy953reg_b = 0x11;
static uint8_t _gyBuffer[41] = {0};

void gy953_init(gy953* s)
{
	memset(s->_rpy, 0, sizeof(s->_rpy[0]) * 3);
	memset(s->_raw, 0, sizeof(s->_rpy[0]) * 4);

	// pinMode(_CS,OUTPUT);
	gpio_init(s->cs_pin);
	gpio_set_dir(s->cs_pin, GPIO_OUT);

	// SPI.begin();
	spi_inst_t *spi = spi0;
	stdio_init_all();

	// SPI.setBitOrder(MSBFIRST);
	// SPI.setClockDivider(SPI_CLOCK_DIV64);
	// SPI.setDataMode(SPI_MODE3);

	spi_init(spi, 400 * 1000);  	// 400KHz
	spi_set_format(spi0,   		// SPI instance
                   8,      		// Number of bits per transfer
                   SPI_CPOL_1,     // Polarity (CPOL)
                   SPI_CPHA_1,     // Phase (CPHA)
                   SPI_MSB_FIRST);

	gpio_set_function(s->sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(s->mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(s->miso_pin, GPIO_FUNC_SPI);

	//_disableCS();
	// digitalWrite(_CS, HIGH);
	gpio_put(s->cs_pin, 1);
	sleep_ms(10);
	//TODO: Check this
	gy953_setRefreshRate(s, 100);
	//setMode(1);
	gy953_enableInt(s);
	while (!_attention)
		sleep_ms(1);
	//setRefreshRate(100);
	//setMode(1);
}

void gy953_disableCS(gy953* s)
{
	// digitalWrite(_CS, HIGH);
	gpio_put(s->cs_pin, 1);
}

void gy953_enableCS(gy953* s)
{
	// digitalWrite(_CS, LOW);
	gpio_put(s->cs_pin, 0);
}

void gy953_writeRegister(gy953* s, uint8_t reg, uint8_t *data, int len)
{
	uint8_t i = 0;
	gy953_enableCS(s);

	//SPI.transfer(reg); //Send register location
	spi_write_blocking(spi0, &reg, 1);

	while (i < len){
		// SPI.transfer(data[i++]);
		spi_write_blocking(spi0, &data[i++], 1);
	}
	gy953_disableCS(s);
}

void gy953_readRegister(gy953* s, uint8_t reg, uint8_t *data, int len)
{
	uint8_t i = 0;
	gy953_enableCS(s);

	// SPI.transfer(reg);
	spi_write_blocking(spi0, &reg, 1);

	while (i < len){
		// data[i++] = SPI.transfer(0);
		spi_read_blocking(spi0, 0, &data[i++], 1);
	}
	gy953_disableCS(s);
}

void gy953_interrupt_callback(uint gpio, uint32_t events)
{
	if (!_attention)
        _attention = true;
}

void gy953_enableInt(gy953* s)
{
	// pinMode(_INTp,INPUT);

	_attention = false;

	// uint8_t theINT = digitalPinToInterrupt(_INTp);

	// SPI.usingInterrupt(theINT);

	// attachInterrupt(theINT, isr, RISING);

	gpio_set_irq_enabled_with_callback(s->int_pin, GPIO_IRQ_EDGE_RISE, true, gy953_interrupt_callback);
}

bool gy953_update(gy953* s, uint8_t mode)
{
	uint8_t sum = 0;
	if (_attention){
		//TODO:disable other INT here
		uint32_t interrupt_status = save_and_disable_interrupts();
		gy953_readRegister(s, _GY953_INTREG, _gyBuffer, 41);//fill buffer
		gy953_setMode(s, mode);
		if (_gyBuffer[34] == 0x0D){
			if (s->_stateReg & _GY953_MAGRPY){
				for (uint8_t i = 0; i < 6; i++) sum += _gyBuffer[20 + i];
				if (sum == _gyBuffer[39]){
					s->_rpy[0] = (_gyBuffer[20] << 8) | _gyBuffer[21];//(float)_roll/100
					s->_rpy[1] = (_gyBuffer[22] << 8) | _gyBuffer[23];//(float)_pitch/100
					s->_rpy[2] = (_gyBuffer[24] << 8) | _gyBuffer[25];//(float)_yaw/100
				}
			}//end _GY953_MAGRPY
			if (s->_stateReg & _GY953_MAGACC){
				s->_raw[0] = (_gyBuffer[2] << 8) | _gyBuffer[3];//X
				s->_raw[1] = (_gyBuffer[4] << 8) | _gyBuffer[5];//Y
				s->_raw[2] = (_gyBuffer[6] << 8) | _gyBuffer[7];//Z
				s->_raw[3] = 0;//na
			}
			if (s->_stateReg & _GY953_MAGGYR){
				s->_raw[0] = (_gyBuffer[8] << 8) | _gyBuffer[9];//X
				s->_raw[1] = (_gyBuffer[10] << 8) | _gyBuffer[11];//Y
				s->_raw[2] = (_gyBuffer[12] << 8) | _gyBuffer[13];//Z
				s->_raw[3] = 0;//na
			}
			if (s->_stateReg & _GY953_MAGMAG){
				s->_raw[0] = (_gyBuffer[14] << 8) | _gyBuffer[15];//X
				s->_raw[1] = (_gyBuffer[16] << 8) | _gyBuffer[17];//Y
				s->_raw[2] = (_gyBuffer[18] << 8) | _gyBuffer[19];//Z
				s->_raw[3] = 0;//na
			}
			if (s->_stateReg & _GY953_MAG_Q4){
				s->_raw[0] = (_gyBuffer[26] << 8) | _gyBuffer[27];
				s->_raw[1] = (_gyBuffer[28] << 8) | _gyBuffer[29];
				s->_raw[2] = (_gyBuffer[30] << 8) | _gyBuffer[31];
				s->_raw[3] = (_gyBuffer[32] << 8) | _gyBuffer[33];
			}
		}
		_attention = false;
		restore_interrupts(interrupt_status);
		//TODO:enable other INT here
		return 1;
	} else {
		return 0;
	}
}

void gy953_sensorEnabled(gy953* s, uint8_t mode)
{
	uint8_t tempdata;
	switch(mode){
		case 0://all
			tempdata = 0x73 | 0x08;
			gy953_writeRegister(s, _GY953_SETREG, &tempdata, 1);
		break;
		case 1://close acc
			tempdata = 0x63 | 0x08;
			gy953_writeRegister(s, _GY953_SETREG, &tempdata, 1);
		break;
		case 2://close gyro
			tempdata = 0x53 | 0x08;
			gy953_writeRegister(s, _GY953_SETREG, &tempdata, 1);
		break;
		case 3://close magn
			tempdata = 0x33 | 0x08;
			gy953_writeRegister(s, _GY953_SETREG, &tempdata, 1);
		break;
		default:
			return;
		break;
	}
}

void gy953_setRefreshRate(gy953* s, uint8_t freq)
{
	uint8_t tempdata;
	if (freq <= 50){
		tempdata = _GY953_UPFREQ_50 | 0x08;//_gy953reg_a;
		gy953_writeRegister(s, _GY953_SETREG, &tempdata, 1);
		s->_refreshRate = 0;
	} else if (freq > 50 && freq <= 100){
		tempdata = _GY953_UPFREQ_100 | 0x08;//_gy953reg_a;
		gy953_writeRegister(s, _GY953_SETREG, &tempdata, 1);
		s->_refreshRate = 1;
	} else {
		tempdata = _GY953_UPFREQ_200 | 0x08;//_gy953reg_a;
		gy953_writeRegister(s, _GY953_SETREG, &tempdata, 1);
		s->_refreshRate = 2;
	}
}

void gy953_calibration(gy953* s, uint8_t mode)
{
	//uint8_t tempdata;
	if (mode < 1){			//0:Clear Internal Saved Calibration Data (CAUTION)
		/*
		tempdata = _GY953_CAL_CLR | _gy953reg_b;
		writeRegister(_GY953_CALREG, &tempdata, 1);
		*/
	} else if (mode > 1){	//2:Calibrate Compass
		/*
		tempdata = _GY953_CAL_MAG | _gy953reg_b;
		writeRegister(_GY953_CALREG, &tempdata, 1);
		*/
	} else {				//1:Calibrate Accellerometer/Gyroscope
		/*
		tempdata = _GY953_CAL_ACC | _gy953reg_b;
		writeRegister(_GY953_CALREG, &tempdata, 1);
		*/
	}
}

void gy953_setMode(gy953* s, uint8_t mode)
{
	if (mode != s->_currentMode){
        //uint8_t data;
        switch(mode){
            case 0:
                //do nothing
            break;
            case 1://enable/disable R-P-Y out
                s->_stateReg ^= _GY953_MAGRPY;
            break;
            case 2://enable/disable ACC Raw data
                s->_stateReg ^= _GY953_MAGACC;
            break;
            case 3://enable/disable Gyroscope Raw data
                s->_stateReg ^= _GY953_MAGGYR;
            break;
            case 4://enable/disable Magnetometer Raw data
                s->_stateReg ^= _GY953_MAGMAG;
            break;
            case 5://enable/disable Q Raw data
                s->_stateReg ^= _GY953_MAG_Q4;
            break;
            default:
                mode = 0;
            break;
        }
	    s->_currentMode = mode;
	}
}



void gy953_getRPY(gy953* s, int *data)
{
	//should we set ON RPY?
	if (s->_currentMode != 1) gy953_setMode(s, 1);
	memcpy(data,s->_rpy,sizeof(s->_rpy));
}

void gy953_getACC(gy953* s, int *data)
{
	if (s->_currentMode != 2) gy953_setMode(s, 2);
	memcpy(data,s->_raw,sizeof(s->_raw));
}

void gy953_getGYR(gy953* s, int *data)
{
	if (s->_currentMode != 3) gy953_setMode(s, 3);
	memcpy(data,s->_raw,sizeof(s->_raw));
}

void gy953_getMAG(gy953* s, int *data)
{
	if (s->_currentMode != 4) gy953_setMode(s, 4);
	memcpy(data,s->_raw,sizeof(s->_raw));
}

void gy953_getQ(gy953* s, int *data)
{
	if (s->_currentMode != 5) gy953_setMode(s, 5);
	memcpy(data,s->_raw,sizeof(s->_raw));
}

void gy953_readAccuracy(uint8_t *data)
{
	data[0] = ((_gyBuffer[35] >> 4) & 0x03);//MAG_ACC Accuracy
	data[1] = ((_gyBuffer[35] >> 2) & 0x03);//MAG_GYR Accuracy
	data[2] = (_gyBuffer[35] & 0x03);//MAG_MAGN Accuracy
	data[3] = (_gyBuffer[0] & 0x07);//output frequency. If 0:not working!
	//_stateReg = 0x00;
}

void gy953_readRange(uint8_t *data)
{
	data[0] = ((_gyBuffer[34] >> 4) & 0x03);//MAG_ACC Range
	data[1] = ((_gyBuffer[34] >> 2) & 0x03);//MAG_GYR Range
	data[2] = (_gyBuffer[34] & 0x03);//MAG_MAGN Range
	//_stateReg = 0x00;
}