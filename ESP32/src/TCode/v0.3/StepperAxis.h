#pragma once

#include <Arduino.h>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <Wire.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <driver/gpio.h>

namespace stepper
{
static inline long mapLong(long x, long inMin, long inMax, long outMin, long outMax)
{
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

class EncoderBase
{
public:
    virtual ~EncoderBase() = default;
    virtual bool begin() = 0;
    virtual bool readSteps(long &steps, int &raw) = 0;
    virtual long stepsPerRev() const = 0;
    virtual long offsetSteps() const { return 0; }
    virtual bool setOffsetFromRaw(int /*raw*/) { return false; }
    virtual void setOffsetSteps(long /*steps*/) {}
};

struct As5600Config
{
    int mode = 0;           // 0=off, 1=I2C, 2=PWM
    int i2cAddr = AS5600_I2C_ADDR_DEFAULT;
    int minRaw = AS5600_MIN_RAW_DEFAULT;
    int maxRaw = AS5600_MAX_RAW_DEFAULT;
    long stepsPerRev = AS5600_STEPS_PER_REV_DEFAULT;
    long offsetSteps = 0;
    int8_t pwmPin = -1;
    int8_t sdaPin = -1;
    int8_t sclPin = -1;
};

class As5600Encoder : public EncoderBase
{
public:
    explicit As5600Encoder(const As5600Config &cfg) : m_cfg(cfg) {}

    bool begin() override
    {
        if (m_cfg.mode == 0)
        {
            return false;
        }

        if (m_cfg.mode == 1)
        {
            Wire.begin(m_cfg.sdaPin, m_cfg.sclPin);
            Wire.setClock(100000);
            m_ok = m_as5600.begin();
            if (!m_ok)
            {
                m_cfg.mode = 0;
                return false;
            }
        }
        else if (m_cfg.mode == 2)
        {
            if (m_cfg.pwmPin < 0)
            {
                return false;
            }
            pinMode(m_cfg.pwmPin, INPUT);
            m_ok = true;
        }
        return m_ok;
    }

    bool readSteps(long &steps, int &raw) override
    {
        if (m_cfg.mode == 0 || !m_ok)
        {
            return false;
        }

        bool ok = false;
        if (m_cfg.mode == 1)
        {
            const uint32_t now = millis();
            if (m_backoffUntilMs && now < m_backoffUntilMs)
            {
                return false;
            }
            const uint16_t angle = m_as5600.rawAngle();
            const int err = m_as5600.lastError();
            if (err != AS5600_OK)
            {
                handleFailure();
                return false;
            }
            raw = static_cast<int>(angle & 0x0FFF);
            m_failCount = 0;
            m_backoffUntilMs = 0;
            ok = true;
        }
        else if (m_cfg.mode == 2)
        {
            const unsigned long highTime = pulseIn(m_cfg.pwmPin, HIGH, 2500);
            const unsigned long lowTime = pulseIn(m_cfg.pwmPin, LOW, 2500);
            const unsigned long period = highTime + lowTime;
            if (period == 0)
            {
                return false;
            }
            raw = static_cast<int>((highTime * 4095UL) / period);
            ok = true;
        }

        if (!ok)
        {
            return false;
        }

        const int clampedRaw = std::min(m_cfg.maxRaw, std::max(m_cfg.minRaw, raw));
        long sensorSteps = mapLong(clampedRaw, m_cfg.minRaw, m_cfg.maxRaw, 0, m_cfg.stepsPerRev);
        sensorSteps -= m_cfg.offsetSteps;
        const long halfRev = m_cfg.stepsPerRev / 2;
        while (sensorSteps > halfRev)
            sensorSteps -= m_cfg.stepsPerRev;
        while (sensorSteps < -halfRev)
            sensorSteps += m_cfg.stepsPerRev;

        m_lastRaw = clampedRaw;
        steps = sensorSteps;
        return true;
    }

    long stepsPerRev() const override { return m_cfg.stepsPerRev; }
    long offsetSteps() const override { return m_cfg.offsetSteps; }

    bool setOffsetFromRaw(int raw) override
    {
        const int clampedRaw = std::min(m_cfg.maxRaw, std::max(m_cfg.minRaw, raw));
        m_cfg.offsetSteps = mapLong(clampedRaw, m_cfg.minRaw, m_cfg.maxRaw, 0, m_cfg.stepsPerRev);
        return true;
    }

    void setOffsetSteps(long steps) override { m_cfg.offsetSteps = steps; }
    int lastRaw() const { return m_lastRaw; }

private:
    void handleFailure()
    {
        m_failCount++;
        if (m_failCount >= 2)
        {
            m_cfg.mode = 0;
            m_ok = false;
            return;
        }
        if (m_failCount % 5 == 0)
        {
            m_backoffUntilMs = millis() + 1000;
        }
    }

    As5600Config m_cfg;
    AS5600 m_as5600;
    bool m_ok = false;
    int m_failCount = 0;
    uint32_t m_backoffUntilMs = 0;
    int m_lastRaw = -1;
};

class StepperAxis
{
public:
    struct Config
    {
        const char *name = "";
        int8_t stepPin = -1;
        int8_t dirPin = -1;
        int rangeSteps = 0;
        int maxHz = 1000;
        int accel = 20000;
        bool invertDir = false;
        uint32_t sensorPollIntervalMs = 25;
        int sensorDeadbandSteps = 3;
        int sensorMaxStepJump = 200;
        float sensorAlpha = 0.2f;
        int posSyncThresholdSteps = 100;
        uint32_t sensorIdleSyncMs = 150; // require recent idle time before planner resync
    };

    StepperAxis() = default;
    explicit StepperAxis(const char *name) : m_name(name) {}

    bool configure(const Config &cfg, FastAccelStepperEngine &engine)
    {
        m_cfg = cfg;
        m_name = cfg.name;
        if (m_cfg.stepPin < 0 || m_cfg.dirPin < 0 || m_cfg.rangeSteps <= 0)
        {
            return false;
        }

        gpio_reset_pin(static_cast<gpio_num_t>(m_cfg.stepPin));
        gpio_reset_pin(static_cast<gpio_num_t>(m_cfg.dirPin));
        gpio_set_direction(static_cast<gpio_num_t>(m_cfg.stepPin), GPIO_MODE_OUTPUT);
        gpio_set_direction(static_cast<gpio_num_t>(m_cfg.dirPin), GPIO_MODE_OUTPUT);
        gpio_set_level(static_cast<gpio_num_t>(m_cfg.stepPin), 0);
        gpio_set_level(static_cast<gpio_num_t>(m_cfg.dirPin), 0);

        m_rangeHalf = std::max(1, m_cfg.rangeSteps / 2);

        m_stepper = engine.stepperConnectToPin(m_cfg.stepPin);
        if (!m_stepper)
        {
            return false;
        }
        m_stepper->setDirectionPin(m_cfg.dirPin, !m_cfg.invertDir);
        m_stepper->setSpeedInHz(m_cfg.maxHz);
        m_stepper->setAcceleration(m_cfg.accel);
        m_stepper->setCurrentPosition(0);
        m_lastTarget = 0;
        return true;
    }

    void setEncoder(std::unique_ptr<EncoderBase> encoder, const Config &cfg)
    {
        m_encoder = std::move(encoder);
        if (m_encoder)
        {
            m_encoder->begin();
            m_cfg.sensorPollIntervalMs = cfg.sensorPollIntervalMs;
            m_cfg.sensorDeadbandSteps = cfg.sensorDeadbandSteps;
            m_cfg.sensorMaxStepJump = cfg.sensorMaxStepJump;
            m_cfg.sensorAlpha = cfg.sensorAlpha;
            m_cfg.posSyncThresholdSteps = cfg.posSyncThresholdSteps;
        }
    }

    void attachTmcDriver(TMC2209Stepper *driver) { m_tmcDriver.reset(driver); }

    void update(long target, uint32_t nowMs)
    {
        if (!m_stepper)
        {
            return;
        }

        const bool running = m_stepper->isRunning();
        if (running)
        {
            m_lastMotionMs = nowMs;
        }

        if (m_encoder && nowMs - m_lastSensorPollMs >= m_cfg.sensorPollIntervalMs)
        {
            long sensorSteps = 0;
            int raw = -1;
            if (m_encoder->readSteps(sensorSteps, raw))
            {
                m_lastSensorRaw = raw;
                m_lastSensorSteps = sensorSteps;
                if (!m_hasSensor)
                {
                    m_filteredSensorSteps = sensorSteps;
                    m_hasSensor = true;
                }
                else
                {
                    long delta = sensorSteps - m_filteredSensorSteps;
                    if (std::abs(delta) < m_cfg.sensorDeadbandSteps)
                    {
                        delta = 0;
                    }
                    else
                    {
                        if (delta > m_cfg.sensorMaxStepJump)
                            delta = m_cfg.sensorMaxStepJump;
                        if (delta < -m_cfg.sensorMaxStepJump)
                            delta = -m_cfg.sensorMaxStepJump;
                        m_filteredSensorSteps += static_cast<long>(delta * m_cfg.sensorAlpha);
                    }
                }

                long plannerPos = m_stepper->getCurrentPosition();
                const long error = m_filteredSensorSteps - plannerPos;
                const bool idleLongEnough = (nowMs - m_lastMotionMs) >= m_cfg.sensorIdleSyncMs;
                if (std::abs(error) > m_cfg.posSyncThresholdSteps && !running && idleLongEnough)
                {
                    m_stepper->setCurrentPosition(m_filteredSensorSteps);
                    plannerPos = m_filteredSensorSteps;
                    if (m_lastTarget != m_filteredSensorSteps)
                    {
                        // Reassert target after resync so planner keeps chasing the commanded point
                        m_stepper->moveTo(m_lastTarget);
                    }
                }
                m_lastPlannerSteps = plannerPos;
            }
            m_lastSensorPollMs = nowMs;
        }

        if (target != m_lastTarget)
        {
            m_stepper->moveTo(target);
            m_lastTarget = target;
            m_lastMotionMs = nowMs;
        }
        else
        {
            m_lastPlannerSteps = m_stepper->getCurrentPosition();
        }
    }

    bool zeroFromEncoder()
    {
        if (!m_encoder)
        {
            return false;
        }
        long steps = 0;
        int raw = -1;
        if (!m_encoder->readSteps(steps, raw))
        {
            return false;
        }
        if (!m_encoder->setOffsetFromRaw(raw))
        {
            return false;
        }
        resetFilter();
        if (m_stepper)
        {
            m_stepper->setCurrentPosition(0);
        }
        return true;
    }

    const char *name() const { return m_name; }
    FastAccelStepper *stepper() const { return m_stepper; }
    TMC2209Stepper *tmcDriver() const { return m_tmcDriver.get(); }
    bool hasEncoder() const { return static_cast<bool>(m_encoder); }
    long encoderOffsetSteps() const { return m_encoder ? m_encoder->offsetSteps() : 0; }
    void setEncoderOffsetSteps(long steps)
    {
        if (m_encoder)
        {
            m_encoder->setOffsetSteps(steps);
            resetFilter();
        }
    }
    bool getSensorSnapshot(int &raw, long &steps, long &filtered, long &planner) const
    {
        if (!m_encoder)
        {
            return false;
        }
        raw = m_lastSensorRaw;
        steps = m_lastSensorSteps;
        filtered = m_filteredSensorSteps;
        planner = m_lastPlannerSteps;
        return true;
    }

    int rangeHalf() const { return m_rangeHalf; }
    long encoderStepsPerRev() const { return m_encoder ? m_encoder->stepsPerRev() : 0; }

private:
    void resetFilter()
    {
        m_filteredSensorSteps = 0;
        m_lastSensorSteps = 0;
        m_lastSensorRaw = -1;
        m_hasSensor = false;
    }

    const char *m_name = "";
    Config m_cfg;
    int m_rangeHalf = 0;
    FastAccelStepper *m_stepper = nullptr;
    std::unique_ptr<TMC2209Stepper> m_tmcDriver;
    std::unique_ptr<EncoderBase> m_encoder;

    long m_lastTarget = 0;
    bool m_hasSensor = false;
    long m_filteredSensorSteps = 0;
    long m_lastSensorSteps = 0;
    int m_lastSensorRaw = -1;
    long m_lastPlannerSteps = 0;
    uint32_t m_lastSensorPollMs = 0;
    uint32_t m_lastMotionMs = 0;
};

} // namespace stepper
