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
#include <driver/gpio.h>
#include <FastAccelStepper.h>
#include <algorithm>
#include <cmath>

#include "TCode0_3.h"
#include "SettingsHandler.h"
#include "Global.h"
#include "MotorHandler0_3.h"
#include "TagHandler.h"
#include "settingsFactory.h"
#include "pinMap.h"

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

        if (!configureAxis(m_strokeAxis, m_pinMap->strokeStep(), m_pinMap->strokeDir(), STEPPER_STROKE_RANGE, STEPPER_STROKE_MAX_HZ, STEPPER_STROKE_INVERT, STEPPER_STROKE_ACCEL, STEPPER_STROKE_RANGE_DEFAULT, STEPPER_STROKE_MAX_HZ_DEFAULT, STEPPER_STROKE_ACCEL_DEFAULT))
            m_initFailed = true;
        if (!configureAxis(m_rollAxis, m_pinMap->rollStep(), m_pinMap->rollDir(), STEPPER_ROLL_RANGE, STEPPER_ROLL_MAX_HZ, STEPPER_ROLL_INVERT, STEPPER_ROLL_ACCEL, STEPPER_ROLL_RANGE_DEFAULT, STEPPER_ROLL_MAX_HZ_DEFAULT, STEPPER_ROLL_ACCEL_DEFAULT))
            m_initFailed = true;
        if (!configureAxis(m_pitchAxis, m_pinMap->pitchStep(), m_pinMap->pitchDir(), STEPPER_PITCH_RANGE, STEPPER_PITCH_MAX_HZ, STEPPER_PITCH_INVERT, STEPPER_PITCH_ACCEL, STEPPER_PITCH_RANGE_DEFAULT, STEPPER_PITCH_MAX_HZ_DEFAULT, STEPPER_PITCH_ACCEL_DEFAULT))
            m_initFailed = true;

        // AS5600 feedback is optional and only used on stroke for simple closed loop
        m_settingsFactory->getValue(AS5600_MODE, m_as5600Mode);
        m_settingsFactory->getValue(AS5600_I2C_ADDR, m_as5600I2CAddr);
        m_settingsFactory->getValue(AS5600_MIN_RAW, m_as5600MinRaw);
        m_settingsFactory->getValue(AS5600_MAX_RAW, m_as5600MaxRaw);
        m_settingsFactory->getValue(AS5600_STEPS_PER_REV, m_as5600StepsPerRev);
        m_settingsFactory->getValue(AS5600_OFFSET_STEPS, m_as5600OffsetSteps);
        s_lastOffsetSteps = m_as5600OffsetSteps;
        if (m_as5600Mode == 1 && m_as5600I2CAddr != AS5600_DEFAULT_ADDRESS)
        {
            LogHandler::warning(_TAG, "AS5600 custom I2C address unsupported; using 0x36");
            m_as5600I2CAddr = AS5600_DEFAULT_ADDRESS;
        }
        m_as5600PwmPin = m_pinMap->as5600Pwm();
        if (m_as5600Mode == 1)
        {
            Wire.begin(m_pinMap->i2cSda(), m_pinMap->i2cScl());
            Wire.setClock(100000); // be conservative to reduce boot errors
            m_as5600FailCount = 0;
            m_as5600BackoffUntilMs = 0;
            m_as5600Ok = m_as5600.begin();
            if (!m_as5600Ok)
            {
                LogHandler::warning(_TAG, "AS5600 not detected at boot; disabling encoder");
                m_as5600Mode = 0;
            }
        }
        else if (m_as5600Mode == 2 && m_as5600PwmPin >= 0)
        {
            pinMode(m_as5600PwmPin, INPUT);
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
        return s_instance->m_as5600StepsPerRev;
    }

    static bool getStrokeSensorSnapshot(int &raw, long &steps, long &filtered, long &offset, long &planner)
    {
        if (!s_instance)
        {
            return false;
        }
        raw = s_lastSensorRaw;
        steps = s_lastSensorSteps;
        filtered = s_lastFilteredSteps;
        offset = s_lastOffsetSteps;
        planner = s_lastPlannerSteps;
        return true;
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
        if (m_as5600Mode == 0)
        {
            return false;
        }
        const int raw = readAS5600();
        if (raw < 0)
        {
            return false;
        }
        const long sensorSteps = mapLong(raw, m_as5600MinRaw, m_as5600MaxRaw, 0, m_as5600StepsPerRev);
        m_as5600OffsetSteps = sensorSteps;
        s_lastOffsetSteps = m_as5600OffsetSteps;
        m_settingsFactory->setValue(AS5600_OFFSET_STEPS, static_cast<int>(m_as5600OffsetSteps));
        m_settingsFactory->saveCommon();

        m_filteredSensorSteps = 0;
        m_hasSensor = false;
        s_lastSensorRaw = raw;
        s_lastSensorSteps = 0;
        s_lastFilteredSteps = 0;
        if (m_strokeAxis.stepper)
        {
            m_strokeAxis.stepper->setCurrentPosition(0);
        }
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

        const long strokeTarget = mapLong(strokeTcode, TCODE_MIN, TCODE_MAX, -m_strokeAxis.rangeHalf, m_strokeAxis.rangeHalf);
        const long rollTarget = mapLong(rollTcode, TCODE_MIN, TCODE_MAX, -m_rollAxis.rangeHalf, m_rollAxis.rangeHalf);
        const long pitchTarget = mapLong(pitchTcode, TCODE_MIN, TCODE_MAX, -m_pitchAxis.rangeHalf, m_pitchAxis.rangeHalf);

        int strokeSensor = -1;
        if (m_as5600Mode != 0)
        {
            if (nowMs - m_as5600LastPollMs >= kSensorPollIntervalMs)
            {
                strokeSensor = readAS5600();
                m_as5600LastPollMs = nowMs;
                m_as5600LastRaw = strokeSensor;
            }
            else
            {
                strokeSensor = m_as5600LastRaw;
            }
        }

        updateAxis(m_strokeAxis, strokeTarget, strokeSensor);
        updateAxis(m_rollAxis, rollTarget, -1);
        updateAxis(m_pitchAxis, pitchTarget, -1);

        executeCommon(strokeTcode);
    }

private:
    struct StepperAxis
    {
        int8_t stepPin = -1;
        int8_t dirPin = -1;
        FastAccelStepper *stepper = nullptr;
        long rangeHalf = 0;
        int maxHz = 1000;
        int accel = 20000;
        bool invertDir = false;
        long lastTarget = 0;
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

    const char * _TAG = TagHandler::MotorHandler;
    SettingsFactory *m_settingsFactory = nullptr;
    PinMapOSRStepper *m_pinMap = nullptr;
    bool m_initFailed = false;

    StepperAxis m_strokeAxis;
    StepperAxis m_rollAxis;
    StepperAxis m_pitchAxis;
    FastAccelStepperEngine m_engine;

    int m_as5600Mode = 0; // 0=off, 1=I2C, 2=PWM
    int m_as5600I2CAddr = AS5600_I2C_ADDR_DEFAULT;
    int m_as5600MinRaw = AS5600_MIN_RAW_DEFAULT;
    int m_as5600MaxRaw = AS5600_MAX_RAW_DEFAULT;
    int8_t m_as5600PwmPin = -1;
    AS5600 m_as5600;
    bool m_as5600Ok = false;
    long m_as5600StepsPerRev = AS5600_STEPS_PER_REV_DEFAULT;
    long m_as5600OffsetSteps = 0;
    long m_filteredSensorSteps = 0;
    bool m_hasSensor = false;
    int m_as5600FailCount = 0;
    uint32_t m_as5600BackoffUntilMs = 0;
    uint32_t m_as5600LastPollMs = 0;
    int m_as5600LastRaw = -1;

    inline static StepperHandler0_3 *s_instance = nullptr;
    inline static volatile int s_lastSensorRaw = -1;
    inline static volatile long s_lastSensorSteps = 0;
    inline static volatile long s_lastFilteredSteps = 0;
    inline static volatile long s_lastOffsetSteps = 0;
    inline static volatile long s_lastPlannerSteps = 0;

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

    bool applyAxisConfig(StepperAxis &axis, const char *rangeKey, const char *maxHzKey, const char *invertKey, const char *accelKey, int rangeDefault, int maxHzDefault, int accelDefault)
    {
        int rangeSteps = rangeDefault;
        m_settingsFactory->getValue(rangeKey, rangeSteps);
        if (rangeSteps <= 0)
        {
            rangeSteps = rangeDefault;
        }

        int maxHz = maxHzDefault;
        m_settingsFactory->getValue(maxHzKey, maxHz);
        maxHz = clampHz(maxHz);

        int accel = accelDefault;
        m_settingsFactory->getValue(accelKey, accel);
        accel = clampAccel(accel);

        bool invertDir = false;
        m_settingsFactory->getValue(invertKey, invertDir);

        const bool prevInvert = axis.invertDir;
        axis.rangeHalf = std::max(1, rangeSteps / 2);
        axis.maxHz = maxHz;
        axis.accel = accel;
        axis.invertDir = invertDir;

        if (axis.stepper)
        {
            axis.stepper->setSpeedInHz(axis.maxHz);
            axis.stepper->setAcceleration(axis.accel);
            if (axis.invertDir != prevInvert)
            {
                // Only touch DIR polarity if the setting changed to avoid mid-move flips
                axis.stepper->setDirectionPin(axis.dirPin, !axis.invertDir);
            }
        }

        return true;
    }

    bool attachStepper(StepperAxis &axis)
    {
        if (axis.stepPin < 0 || axis.dirPin < 0)
        {
            return false;
        }

        axis.stepper = m_engine.stepperConnectToPin(axis.stepPin);
        if (!axis.stepper)
        {
            LogHandler::error(_TAG, "Failed to attach stepper to pin %d", axis.stepPin);
            return false;
        }

        // true means DIR high = count up; invert flips that
        axis.stepper->setDirectionPin(axis.dirPin, !axis.invertDir);
        axis.stepper->setSpeedInHz(axis.maxHz);
        axis.stepper->setAcceleration(axis.accel);
        axis.stepper->setCurrentPosition(0);
        axis.lastTarget = 0;
        return true;
    }

    bool configureAxis(StepperAxis &axis, int8_t stepPin, int8_t dirPin, const char *rangeKey, const char *maxHzKey, const char *invertKey, const char *accelKey, int rangeDefault, int maxHzDefault, int accelDefault)
    {
        if (stepPin < 0 || dirPin < 0)
        {
            LogHandler::error(_TAG, "Invalid step/dir pins for axis (%d, %d)", stepPin, dirPin);
            return false;
        }

        axis.stepPin = stepPin;
        axis.dirPin = dirPin;

        gpio_reset_pin(static_cast<gpio_num_t>(axis.stepPin));
        gpio_reset_pin(static_cast<gpio_num_t>(axis.dirPin));
        gpio_set_direction(static_cast<gpio_num_t>(axis.stepPin), GPIO_MODE_OUTPUT);
        gpio_set_direction(static_cast<gpio_num_t>(axis.dirPin), GPIO_MODE_OUTPUT);
        gpio_set_level(static_cast<gpio_num_t>(axis.stepPin), 0);
        gpio_set_level(static_cast<gpio_num_t>(axis.dirPin), 0);

        applyAxisConfig(axis, rangeKey, maxHzKey, invertKey, accelKey, rangeDefault, maxHzDefault, accelDefault);

        return attachStepper(axis);
    }

    void updateAxis(StepperAxis &axis, long target, int sensorValue)
    {
        if (!axis.stepper)
        {
            return;
        }

        // Track planner position for telemetry only on stroke axis
        if (&axis == &m_strokeAxis)
        {
            s_lastPlannerSteps = axis.stepper->getCurrentPosition();
        }

        if (sensorValue >= 0 && axis.rangeHalf > 0)
        {
            int minRaw = m_as5600MinRaw;
            int maxRaw = m_as5600MaxRaw;
            if (maxRaw - minRaw < 10)
            {
                minRaw = AS5600_MIN_RAW_DEFAULT;
                maxRaw = AS5600_MAX_RAW_DEFAULT;
            }
            const int clampedRaw = std::min(maxRaw, std::max(minRaw, sensorValue));
            // Map encoder raw directly into configured encoder steps-per-rev
            long sensorSteps = mapLong(clampedRaw, minRaw, maxRaw, 0, m_as5600StepsPerRev);
            sensorSteps -= m_as5600OffsetSteps;   // apply zero offset

            // Wrap into +/- half rev range in encoder steps
            const long halfRev = m_as5600StepsPerRev / 2;
            while (sensorSteps > halfRev) sensorSteps -= m_as5600StepsPerRev;
            while (sensorSteps < -halfRev) sensorSteps += m_as5600StepsPerRev;

            // Use encoder-steps directly so telemetry matches planner units when scaled correctly
            s_lastSensorRaw = sensorValue;
            s_lastSensorSteps = sensorSteps;

            if (!m_hasSensor)
            {
                m_filteredSensorSteps = sensorSteps;
                m_hasSensor = true;
            }
            else
            {
                long delta = sensorSteps - m_filteredSensorSteps;
                if (std::abs(delta) < kSensorDeadbandSteps)
                {
                    delta = 0;
                }
                else
                {
                    if (delta > kSensorMaxStepJump) delta = kSensorMaxStepJump;
                    if (delta < -kSensorMaxStepJump) delta = -kSensorMaxStepJump;
                    m_filteredSensorSteps += static_cast<long>(delta * kSensorAlpha);
                }
            }

            // Telemetry + correction: if idle and error is large, resync planner to filtered sensor steps
            if (&axis == &m_strokeAxis)
            {
                long plannerPos = axis.stepper->getCurrentPosition();
                const long error = m_filteredSensorSteps - plannerPos;
                if (std::abs(error) > kPosSyncThresholdSteps && !axis.stepper->isRunning())
                {
                    axis.stepper->setCurrentPosition(m_filteredSensorSteps);
                    plannerPos = m_filteredSensorSteps;
                }
                s_lastPlannerSteps = plannerPos;
                s_lastFilteredSteps = m_filteredSensorSteps;
            }
        }

        // Issue motion only when target changes to avoid re-planning every sensor poll
        if (target != axis.lastTarget)
        {
            axis.stepper->moveTo(target);
            axis.lastTarget = target;
        }
    }

    int readAS5600()
    {
        if (m_as5600Mode == 1)
        {
            return readAS5600I2C();
        }
        if (m_as5600Mode == 2)
        {
            return readAS5600Pwm();
        }
        return -1;
    }

    int readAS5600I2C()
    {
        const uint32_t now = millis();
        if (m_as5600BackoffUntilMs && now < m_as5600BackoffUntilMs)
        {
            return -1;
        }
        if (!m_as5600Ok)
        {
            return -1;
        }

        const uint16_t angle = m_as5600.rawAngle();
        const int err = m_as5600.lastError();
        if (err != AS5600_OK)
        {
            handleAS5600Failure();
            return -1;
        }

        m_as5600FailCount = 0;
        m_as5600BackoffUntilMs = 0;
        return static_cast<int>(angle & 0x0FFF);
    }

    void handleAS5600Failure()
    {
        m_as5600FailCount++;
        if (m_as5600FailCount >= 2)
        {
            LogHandler::warning(_TAG, "AS5600 disabled after repeated I2C errors");
            m_as5600Mode = 0;
            return;
        }
        if (m_as5600FailCount % 5 == 0)
        {
            m_as5600BackoffUntilMs = millis() + 1000;
            LogHandler::warning(_TAG, "AS5600 I2C error, backing off");
        }
    }

    int readAS5600Pwm()
    {
        if (m_as5600PwmPin < 0)
        {
            return -1;
        }

        const unsigned long highTime = pulseIn(m_as5600PwmPin, HIGH, 2500);
        const unsigned long lowTime = pulseIn(m_as5600PwmPin, LOW, 2500);
        const unsigned long period = highTime + lowTime;
        if (period == 0)
        {
            return -1;
        }
        return static_cast<int>((highTime * 4095UL) / period);
    }
};