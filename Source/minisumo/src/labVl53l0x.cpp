#include "labVl53l0x.h"

#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api_core.h"

static constexpr const char *TAG = "VL53L0X";
static constexpr uint8_t VL53L0X_I2C_ADDRESS_DEFAULT = 0x29;

bool labVL53L0X::Init(VL53L0X_DeviceModes devMode)
{
    if (gpio_xshut != GPIO_NUM_MAX)
    {
        gpio_set_direction(gpio_xshut, GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_xshut, 1);
    }

    /* device init */
    vl53l0x_dev.i2c_port_num = i2c_port;
    vl53l0x_dev.i2c_address = VL53L0X_I2C_ADDRESS_DEFAULT;
    VL53L0X_ResetDevice(&vl53l0x_dev);

    if (init_vl53l0x(devMode) != VL53L0X_ERROR_NONE)
        return false;

    if (VL53L0X_ERROR_NONE !=
        VL53L0X_SetGpioConfig(&vl53l0x_dev, 0,
                              devMode,
                              VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
                              VL53L0X_INTERRUPTPOLARITY_LOW))
    {
        return false;
    }


    
    if (!setTimingBudget(20000))
        return false;

    VL53L0X_StartMeasurement(&vl53l0x_dev);

    return true;
}

bool labVL53L0X::setThresholds(uint32_t rate, uint32_t threshold)
{
    //VL53L0X_SetReferenceSpads(&vl53l0x_dev, 6, 1);
    if (VL53L0X_ERROR_NONE != VL53L0X_SetLimitCheckEnable(&vl53l0x_dev, 
                                VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                1))
    {
        return false;
    }

    if (VL53L0X_ERROR_NONE != VL53L0X_SetLimitCheckValue(&vl53l0x_dev, 
                                VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                (FixPoint1616_t)(rate * 65536 / 100)))
    {
        return false;
    }

    if (VL53L0X_ERROR_NONE != VL53L0X_SetLimitCheckEnable(&vl53l0x_dev, 
                                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                1))
    {
        return false;
    }

    if (VL53L0X_ERROR_NONE != VL53L0X_SetLimitCheckValue(&vl53l0x_dev, 
                                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                (FixPoint1616_t)(threshold * 65536 / 100)))
    {
        return false;
    }
}

bool labVL53L0X::readSingleWithPolling(uint16_t *pRangeMilliMeter)
{
    VL53L0X_RangingMeasurementData_t MeasurementData;

    VL53L0X_Error status =
        VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev, &MeasurementData);

    if (status != VL53L0X_ERROR_NONE)
    {
        return false;
    }

    *pRangeMilliMeter = MeasurementData.RangeMilliMeter;

    if (MeasurementData.RangeStatus != 0)
        return false;

    return true;
}

void labVL53L0X::i2cMasterInit(gpio_num_t pin_sda, gpio_num_t pin_scl, uint32_t freq)
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = pin_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = pin_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));
}

VL53L0X_Error labVL53L0X::init_vl53l0x(VL53L0X_DeviceModes devMode)
{
    VL53L0X_Error status;
    uint8_t isApertureSpads;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t VhvSettings;
    // Device Initialization (~40ms)
    status = VL53L0X_DataInit(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE)
        return print_pal_error(status, "VL53L0X_DataInit");
    status = VL53L0X_StaticInit(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE)
        return print_pal_error(status, "VL53L0X_StaticInit");
    // SPADs calibration (~10ms)
    status = VL53L0X_PerformRefSpadManagement(&vl53l0x_dev, &refSpadCount,
                                              &isApertureSpads);
    ESP_LOGI(TAG, "refSpadCount = %" PRIu32 ", isApertureSpads = %" PRIu8 "\n", refSpadCount,
             isApertureSpads);
    if (status != VL53L0X_ERROR_NONE)
        return print_pal_error(status, "VL53L0X_PerformRefSpadManagement");
    // Temperature calibration (~40ms)
    status = VL53L0X_PerformRefCalibration(&vl53l0x_dev, &VhvSettings, &PhaseCal);
    if (status != VL53L0X_ERROR_NONE)
        return print_pal_error(status, "VL53L0X_PerformRefCalibration");
    // Setup in single ranging mode
    status = VL53L0X_SetDeviceMode(&vl53l0x_dev, devMode);
    if (status != VL53L0X_ERROR_NONE)
        return print_pal_error(status, "VL53L0X_SetDeviceMode");
    // end
    return status;
}

VL53L0X_Error labVL53L0X::print_pal_error(VL53L0X_Error status,
                                          const char *method)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(status, buf);
    ESP_LOGE(TAG, "%s API status: %i : %s\n", method, status, buf);
    return status;
}

bool labVL53L0X::setTimingBudget(uint32_t TimingBudgetMicroSeconds)
{
    VL53L0X_Error status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
        &vl53l0x_dev, TimingBudgetMicroSeconds);
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status, "VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
        return false;
    }
    this->TimingBudgetMicroSeconds = TimingBudgetMicroSeconds;
    return true;
}

bool labVL53L0X::readContiniousLastData(uint16_t *pRangeMilliMeter)
{
    int Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t p_data;

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_measurement_poll_for_completion(&vl53l0x_dev);
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &p_data);

        // Clear the interrupt
        // VL53L0X_ClearInterruptMask(&vl53l0x_dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    }

    *pRangeMilliMeter = p_data.RangeMilliMeter;
    return Status;
}