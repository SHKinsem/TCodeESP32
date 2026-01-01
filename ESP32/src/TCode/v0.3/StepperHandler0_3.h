/* MIT License

Copyright (c) 2024 Jason C. Fain

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#pragma once

#include <Wire.h>
#include <AS5600.h>
#include <HardwareSerial.h>
#include <driver/gpio.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <algorithm>
#include <cmath>
#include <memory>

#include "TCode0_3.h"
#include "SettingsHandler.h"
#include "Global.h"
#include "MotorHandler0_3.h"
#include "TagHandler.h"
#include "settingsFactory.h"
#include "pinMap.h"
#include "StepperAxis.h"

// Simple step/dir stepper handler for OSR using TCode v0.3
class StepperHandler0_3 : public MotorHandler0_3
{
public:
    StepperHandler0_3() : MotorHandler0_3(new TCode0_3()) {}

    void setup() override
    {
        LogHandler::debug(_TAG, "Setting up stepper handler v3");
        m_settingsFactory = SettingsFactory::getInstance();
        s_instance = this;

        DeviceType deviceType = DeviceType::OSR;
        m_settingsFactory->getValue(DEVICE_TYPE, deviceType);
        if (deviceType != DeviceType::OSR_STEPPER)
        {
            LogHandler::error(_TAG, "Stepper handler requires device type OSR_STEPPER");
            m_initFailed = true;
            return;
        }

        m_pinMap = PinMapOSRStepper::getInstance();
        if (!m_pinMap)
        {
            LogHandler::error(_TAG, "Pin map unavailable for stepper");
            m_initFailed = true;
            return;
        }

        m_engine.init();

        // Configure primary axes
        m_tcode->RegisterAxis("L0", "Stroke");
        m_tcode->RegisterAxis("R1", "Roll");
        m_tcode->RegisterAxis("R2", "Pitch");

        stepper::StepperAxis::Config strokeCfg;
        strokeCfg.name = "Stroke";
        strokeCfg.stepPin = m_pinMap->strokeStep();
        strokeCfg.dirPin = m_pinMap->strokeDir();
        strokeCfg.rangeSteps = resolveRange(STEPPER_STROKE_RANGE, STEPPER_STROKE_RANGE_DEFAULT);
        strokeCfg.maxHz = clampHz(resolveInt(STEPPER_STROKE_MAX_HZ, STEPPER_STROKE_MAX_HZ_DEFAULT));
        strokeCfg.accel = clampAccel(resolveInt(STEPPER_STROKE_ACCEL, STEPPER_STROKE_ACCEL_DEFAULT));
        strokeCfg.invertDir = resolveBool(STEPPER_STROKE_INVERT);
        strokeCfg.sensorPollIntervalMs = kSensorPollIntervalMs;
        strokeCfg.sensorDeadbandSteps = kSensorDeadbandSteps;
        strokeCfg.sensorMaxStepJump = kSensorMaxStepJump;
        strokeCfg.sensorAlpha = kSensorAlpha;
        strokeCfg.posSyncThresholdSteps = kPosSyncThresholdSteps;
        if (!m_strokeAxis.configure(strokeCfg, m_engine))
            m_initFailed = true;

        stepper::StepperAxis::Config rollCfg;
        rollCfg.name = "Roll";
        rollCfg.stepPin = m_pinMap->rollStep();
        rollCfg.dirPin = m_pinMap->rollDir();
        rollCfg.rangeSteps = resolveRange(STEPPER_ROLL_RANGE, STEPPER_ROLL_RANGE_DEFAULT);
        rollCfg.maxHz = clampHz(resolveInt(STEPPER_ROLL_MAX_HZ, STEPPER_ROLL_MAX_HZ_DEFAULT));
        rollCfg.accel = clampAccel(resolveInt(STEPPER_ROLL_ACCEL, STEPPER_ROLL_ACCEL_DEFAULT));
        rollCfg.invertDir = resolveBool(STEPPER_ROLL_INVERT);
        if (!m_rollAxis.configure(rollCfg, m_engine))
            m_initFailed = true;

        stepper::StepperAxis::Config pitchCfg;
        pitchCfg.name = "Pitch";
        pitchCfg.stepPin = m_pinMap->pitchStep();
        pitchCfg.dirPin = m_pinMap->pitchDir();
        pitchCfg.rangeSteps = resolveRange(STEPPER_PITCH_RANGE, STEPPER_PITCH_RANGE_DEFAULT);
        pitchCfg.maxHz = clampHz(resolveInt(STEPPER_PITCH_MAX_HZ, STEPPER_PITCH_MAX_HZ_DEFAULT));
        pitchCfg.accel = clampAccel(resolveInt(STEPPER_PITCH_ACCEL, STEPPER_PITCH_ACCEL_DEFAULT));
        pitchCfg.invertDir = resolveBool(STEPPER_PITCH_INVERT);
        if (!m_pitchAxis.configure(pitchCfg, m_engine))
            m_initFailed = true;

        setupTmcDrivers();

        // AS5600 feedback is optional and only used on stroke for simple closed loop
        m_settingsFactory->getValue(AS5600_MODE, m_as5600Mode);
        m_settingsFactory->getValue(AS5600_I2C_ADDR, m_as5600I2CAddr);
        m_settingsFactory->getValue(AS5600_MIN_RAW, m_as5600MinRaw);
        m_settingsFactory->getValue(AS5600_MAX_RAW, m_as5600MaxRaw);
        m_settingsFactory->getValue(AS5600_STEPS_PER_REV, m_as5600StepsPerRev);
        m_settingsFactory->getValue(AS5600_OFFSET_STEPS, m_as5600OffsetSteps);
        if (m_as5600Mode == 1 && m_as5600I2CAddr != AS5600_DEFAULT_ADDRESS)
        {
            LogHandler::warning(_TAG, "AS5600 custom I2C address unsupported; using 0x36");
            m_as5600I2CAddr = AS5600_DEFAULT_ADDRESS;
        }
        m_as5600PwmPin = m_pinMap->as5600Pwm();
        if (m_as5600Mode != 0)
        {
            stepper::As5600Config encCfg;
            encCfg.mode = m_as5600Mode;
            encCfg.i2cAddr = m_as5600I2CAddr;
            encCfg.minRaw = m_as5600MinRaw;
            encCfg.maxRaw = m_as5600MaxRaw;
            encCfg.stepsPerRev = m_as5600StepsPerRev;
            encCfg.offsetSteps = m_as5600OffsetSteps;
            encCfg.pwmPin = m_as5600PwmPin;
            encCfg.sdaPin = m_pinMap->i2cSda();
            encCfg.sclPin = m_pinMap->i2cScl();
            auto encoder = std::make_unique<stepper::As5600Encoder>(encCfg);
            m_strokeAxis.setEncoder(std::move(encoder), strokeCfg);
        }

        setupCommon();

        if (m_initFailed)
        {
            m_tcode->sendMessage("Init stepper error!");
        }
        else
        {
            m_tcode->sendMessage("Ready!");
        }
    }

    void setMessageCallback(TCODE_FUNCTION_PTR_T function) override
    {
        m_tcode->setMessageCallback(function);
    }

    void read(const String &input) override { m_tcode->read(input); }

    void read(const char *input, size_t len) override
    {
        for (size_t i = 0; i < len; i++)
        {
            read(input[i]);
        }
    }

    void read(byte input) override { m_tcode->read(input); }

    static long getAs5600StepsPerRev()
    {
        if (!s_instance)
        {
            return AS5600_STEPS_PER_REV_DEFAULT;
        }
        const long stepsPerRev = s_instance->m_strokeAxis.encoderStepsPerRev();
        return stepsPerRev > 0 ? stepsPerRev : s_instance->m_as5600StepsPerRev;
    }

    static bool getStrokeSensorSnapshot(int &raw, long &steps, long &filtered, long &offset, long &planner)
    {
        if (!s_instance)
        {
            return false;
        }
        bool ok = s_instance->m_strokeAxis.getSensorSnapshot(raw, steps, filtered, planner);
        offset = s_instance->m_as5600OffsetSteps;
        return ok;
    }

    static bool zeroStrokeFromSensor()
    {
        if (!s_instance)
        {
            return false;
        }
        return s_instance->zeroStrokeFromSensorInternal();
    }

private:
    bool zeroStrokeFromSensorInternal()
    {
        if (!m_strokeAxis.hasEncoder())
        {
            return false;
        }
        if (!m_strokeAxis.zeroFromEncoder())
        {
            return false;
        }
        m_as5600OffsetSteps = m_strokeAxis.encoderOffsetSteps();
        m_settingsFactory->setValue(AS5600_OFFSET_STEPS, static_cast<int>(m_as5600OffsetSteps));
        m_settingsFactory->saveCommon();
        return true;
    }

    void execute() override
    {
        if (m_initFailed)
        {
            return;
        }

        const uint32_t nowMs = millis();
        const int strokeTcode = channelRead("L0");
        const int rollTcode = channelRead("R1");
        const int pitchTcode = channelRead("R2");

        const long strokeTarget = mapLong(strokeTcode, TCODE_MIN, TCODE_MAX, -m_strokeAxis.rangeHalf(), m_strokeAxis.rangeHalf());
        const long rollTarget = mapLong(rollTcode, TCODE_MIN, TCODE_MAX, -m_rollAxis.rangeHalf(), m_rollAxis.rangeHalf());
        const long pitchTarget = mapLong(pitchTcode, TCODE_MIN, TCODE_MAX, -m_pitchAxis.rangeHalf(), m_pitchAxis.rangeHalf());

        m_strokeAxis.update(strokeTarget, nowMs);
        m_rollAxis.update(rollTarget, nowMs);
        m_pitchAxis.update(pitchTarget, nowMs);

        pollTmcStatus(nowMs);

        executeCommon(strokeTcode);
    }

private:
    struct TmcAxisConfig
    {
        uint8_t address = 0;
        int runCurrentmA = 1400;
        int holdCurrentmA = 400;
        int microsteps = 16;
        bool stealthChop = false;
        bool spreadCycle = true;
    };

    // Limits to keep stepping within a safe, realistic range
    static constexpr int kMaxStepperHz = 100000;     // library clamp
    static constexpr int kMinStepperHz = 1;
    static constexpr int kDefaultAccel = 20000;   // steps/s^2
    static constexpr int kMinAccel = 100;         // prevent zero/negative accel
    static constexpr int kMaxAccel = 1000000;      // reasonable upper bound
    static constexpr int kSensorDeadbandSteps = 3;      // ignore tiny noise
    static constexpr int kSensorMaxStepJump = 200;      // limit sudden jumps per refresh
    static constexpr float kSensorAlpha = 0.2f;         // low-pass factor for sensor smoothing
    static constexpr uint32_t kSensorPollIntervalMs = 25; // throttle AS5600 reads
    static constexpr int kPosSyncThresholdSteps = 100;    // only resync planner when error exceeds this many steps
    static constexpr uint32_t kTmcStatusIntervalMs = 1000;

    // TMC2209 defaults now target UART2 (TX2=GPIO17, RX2=GPIO16)
    static constexpr bool kTmcEnabledDefault = true;
    static constexpr int kTmcUartTxPinDefault = 17;
    static constexpr int kTmcUartRxPinDefault = 16;
    static constexpr uint32_t kTmcUartBaudDefault = 115200;
    static constexpr float kTmcRsenseDefault = 0.110f; // ohms (calculator: VSENSE=0, Rsense≈0.108)
    static constexpr TmcAxisConfig kTmcStrokeConfigDefault{0, 1500, 750, 16, false, true};
    static constexpr TmcAxisConfig kTmcRollConfigDefault{1, 700, 350, 16, false, true};
    static constexpr TmcAxisConfig kTmcPitchConfigDefault{2, 700, 350, 16, false, true};

    const char * _TAG = TagHandler::MotorHandler;
    SettingsFactory *m_settingsFactory = nullptr;
    PinMapOSRStepper *m_pinMap = nullptr;
    bool m_initFailed = false;

    stepper::StepperAxis m_strokeAxis{"Stroke"};
    stepper::StepperAxis m_rollAxis{"Roll"};
    stepper::StepperAxis m_pitchAxis{"Pitch"};
    FastAccelStepperEngine m_engine;

    bool m_tmcEnabled = kTmcEnabledDefault;
    int m_tmcUartTxPin = kTmcUartTxPinDefault;
    int m_tmcUartRxPin = kTmcUartRxPinDefault;
    uint32_t m_tmcUartBaud = kTmcUartBaudDefault;
    float m_tmcRsense = kTmcRsenseDefault;
    TmcAxisConfig m_tmcStrokeConfig = kTmcStrokeConfigDefault;
    TmcAxisConfig m_tmcRollConfig = kTmcRollConfigDefault;
    TmcAxisConfig m_tmcPitchConfig = kTmcPitchConfigDefault;
    HardwareSerial m_tmcSerial = HardwareSerial(2);
    uint32_t m_lastTmcStatusMs = 0;

    int m_as5600Mode = 0; // 0=off, 1=I2C, 2=PWM
    int m_as5600I2CAddr = AS5600_I2C_ADDR_DEFAULT;
    int m_as5600MinRaw = AS5600_MIN_RAW_DEFAULT;
    int m_as5600MaxRaw = AS5600_MAX_RAW_DEFAULT;
    long m_as5600StepsPerRev = AS5600_STEPS_PER_REV_DEFAULT;
    long m_as5600OffsetSteps = 0;
    int8_t m_as5600PwmPin = -1;

    inline static StepperHandler0_3 *s_instance = nullptr;

    long mapLong(long x, long inMin, long inMax, long outMin, long outMax)
    {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    int clampHz(int maxHz)
    {
        if (maxHz <= 0)
        {
            maxHz = kMinStepperHz;
        }
        return std::max(kMinStepperHz, std::min(kMaxStepperHz, maxHz));
    }

    int clampAccel(int accel)
    {
        if (accel <= 0)
        {
            accel = kDefaultAccel;
        }
        return std::max(kMinAccel, std::min(kMaxAccel, accel));
    }

    int resolveRange(const char *key, int defaultVal)
    {
        int value = defaultVal;
        m_settingsFactory->getValue(key, value);
        if (value <= 0)
        {
            value = defaultVal;
        }
        return value;
    }

    int resolveInt(const char *key, int defaultVal)
    {
        int value = defaultVal;
        m_settingsFactory->getValue(key, value);
        return value;
    }

    bool resolveBool(const char *key)
    {
        bool value = false;
        m_settingsFactory->getValue(key, value);
        return value;
    }

    void setupTmcDrivers()
    {
        if (!kTmcEnabledDefault)
        {
            return;
        }
        if (m_tmcUartTxPin < 0 || m_tmcUartRxPin < 0)
        {
            LogHandler::warning(_TAG, "TMC2209 UART disabled (no TX/RX pins configured)");
            return;
        }

        m_tmcEnabled = true;
        LogHandler::info(_TAG, "TMC2209 UART enabled on TX%d/RX%d @%lu", m_tmcUartTxPin, m_tmcUartRxPin, static_cast<unsigned long>(m_tmcUartBaud));
        m_tmcSerial.begin(m_tmcUartBaud, SERIAL_8N1, m_tmcUartRxPin, m_tmcUartTxPin);

        configureTmcAxis(m_strokeAxis, m_tmcStrokeConfig);
        configureTmcAxis(m_rollAxis, m_tmcRollConfig);
        configureTmcAxis(m_pitchAxis, m_tmcPitchConfig);
    }

    void configureTmcAxis(stepper::StepperAxis &axis, const TmcAxisConfig &cfg)
    {
        if (!m_tmcEnabled)
        {
            return;
        }
        if (!axis.stepper())
        {
            return;
        }
        if (axis.tmcDriver())
        {
            return;
        }

        const uint8_t address = cfg.address & 0x03;
        auto *driver = new TMC2209Stepper(&m_tmcSerial, m_tmcRsense, address);
        if (!driver)
        {
            LogHandler::warning(_TAG, "TMC alloc failed for %s", axis.name());
            return;
        }

        axis.attachTmcDriver(driver);
        driver->begin();
        driver->pdn_disable(true);
        driver->mstep_reg_select(true);
        driver->I_scale_analog(false);
        driver->rms_current(cfg.runCurrentmA, cfg.holdCurrentmA > 0 ? static_cast<float>(cfg.holdCurrentmA) / static_cast<float>(cfg.runCurrentmA) : 0.5f);
        driver->microsteps(cfg.microsteps);
        driver->en_spreadCycle(cfg.spreadCycle);
        driver->pwm_autoscale(true);
        driver->pwm_autograd(true);
        driver->toff(4);
        driver->blank_time(24);       // TBL=1 -> 24 tCLK per calculator
        // driver->hysteresis_start(6);  // HSTRT from calculator
        // driver->hysteresis_end(3);    // HEND from calculator

        const uint8_t result = driver->test_connection();
        if (result != 0)
        {
            LogHandler::warning(_TAG, "TMC2209 %s UART test failed (code %u)", axis.name(), result);
        }
        else
        {
            LogHandler::info(_TAG, "TMC2209 %s ready (addr %u)", axis.name(), address);
        }
    }

    void pollTmcStatus(uint32_t nowMs)
    {
        if (!m_tmcEnabled)
        {
            return;
        }
        if (nowMs - m_lastTmcStatusMs < kTmcStatusIntervalMs)
        {
            return;
        }
        m_lastTmcStatusMs = nowMs;

        auto checkAxis = [&](stepper::StepperAxis &axis) {
            auto *driver = axis.tmcDriver();
            if (!driver)
            {
                return;
            }
            const bool hadError = driver->drv_err();
            const bool hadUv = driver->uv_cp();
            if (hadError || hadUv)
            {
                LogHandler::warning(_TAG, "TMC2209 %s error: drv_err=%d uv_cp=%d", axis.name(), static_cast<int>(hadError), static_cast<int>(hadUv));
                driver->GSTAT(0b111);
            }
        };

        checkAxis(m_strokeAxis);
        checkAxis(m_rollAxis);
        checkAxis(m_pitchAxis);
    }
};