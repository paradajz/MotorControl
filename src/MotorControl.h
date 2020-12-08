#pragma once

#include <cinttypes>
#include <functional>

class MotorControl
{
    public:
    enum class direction_t : uint8_t
    {
        home,
        end
    };

    ///
    /// \brief List of all possible motor speeds.
    ///
    enum class speed_t : uint8_t
    {
        precise,
        slow,
        fast
    };

    ///
    /// \brief Structure holding all necessary information needed to start motor movement.
    ///
    struct motorMovement_t
    {
        direction_t direction        = direction_t::home;
        speed_t     speed            = speed_t::precise;
        uint32_t    pulses           = 0;
        bool        absoluteMovement = false;
    };

    ///
    /// Structure containing values representing time in microseconds after which pulse is sent to motor for every speed.
    ///
    struct motorSpeed_t
    {
        uint32_t stepsPerRev      = 0;
        uint32_t accelerationRPS2 = 0;
        uint32_t decelerationRPS2 = 0;
        uint32_t precise          = 0;
        uint32_t slow             = 0;
        uint32_t fast             = 0;
    };

    class Timer
    {
        public:
        Timer() = default;

        virtual void init(std::function<void()> handler) = 0;
        virtual void start()                             = 0;
        virtual void stop()                              = 0;
        virtual void pause()                             = 0;
        virtual void resume()                            = 0;
        virtual void setPeriod(uint32_t period)          = 0;
        virtual bool isRunning()                         = 0;
    };

    class IO
    {
        public:
        IO() = default;

        virtual void pulse()                                           = 0;
        virtual void setDirection(MotorControl::direction_t direction) = 0;
    };

    using initCallback_t = std::function<bool(motorMovement_t& movement)>;
    using stopCallback_t = std::function<void()>;

    MotorControl(Timer& timer,
                 IO&    io);

    bool     init(const motorSpeed_t& motorSpeed);
    void     checkPendingCallbacks();
    void     registerInitCallback(initCallback_t initCallback);
    void     registerStopCallback(stopCallback_t stopCallback);
    bool     isMovementActive(direction_t& direction);
    bool     isMovementActive();
    bool     setupMovement(motorMovement_t& info);
    bool     stopMovement(bool force = false);
    bool     accelerateToMaxSpeed();
    uint32_t currentPosition();
    void     resetPosition();
    void     setTotalLength(uint32_t value);
    void     setPosition(uint32_t value);
    uint32_t totalLength();
    bool     currentSpeed(speed_t& _speed);

    private:
    void     writeDirection(direction_t direction);
    uint32_t speed(speed_t enumSpeed);
    bool     isMotorValid();
    void     sendPulse();
    void     resetMovement(bool callHandler);

    enum class runState_t : uint8_t
    {
        stopped,
        accelerating,
        running,
        decelerating
    };

    volatile runState_t _runState                    = runState_t::stopped;
    volatile int32_t    _timerInterval               = 0;
    volatile int32_t    _timerIntervalCalcRemainder  = 0;
    volatile int32_t    _accelerationCounter         = 0;
    int32_t             _decelerationRate            = 0;
    uint32_t            _decelerationPulseCountStart = 0;
    bool                _motorSpeedValid             = false;
    volatile uint32_t   _movementPulsesSent          = 0;
    volatile uint32_t   _currentPosition             = 0;
    volatile uint32_t   _movementPulses              = 0;
    int32_t             _startSpeed                  = 0;
    int32_t             _targetSpeed                 = 0;
    direction_t         _movementDirection           = direction_t::home;
    volatile bool       _pendingStopCallback         = false;
    initCallback_t      _initCallback                = nullptr;
    stopCallback_t      _stopCallback                = nullptr;
    const motorSpeed_t* _motorSpeed                  = nullptr;
    uint32_t            _totalMovementLength         = 0;
    bool                _stopRequested               = false;

    //calculations are based on Atmel application note AVR446 (integer math)
    static constexpr uint32_t startTimerPeriod = 5;    //us
    static constexpr uint32_t timerFreq        = 1 / (startTimerPeriod / 1000000.0);
    static constexpr uint32_t timerFreqOffset  = (timerFreq * 0.676) / 100;
    float                     alpha            = 0.0;
    float                     alphaSQ          = 0;
    float                     alphaxTimerx100;
    float                     alphax20000;

    Timer& _timer;

    public:
    IO& io;
};