#include <cmath>
#include "MotorControl.h"

///
/// \brief Default constructor.
/// @param [in] timer       Reference to timer class used to control motor movement.
/// @param [in] io          Reference to IO class used to send pulses to the motor and to write direction.
///
MotorControl::MotorControl(Timer& timer,
                           IO&    io)
    : _timer(timer)
    , io(io)
{
}

///
/// \brief Performs initialization of motor control.
/// @param [in] motorSpeed  Pointer to motor speed definitions used.
/// \returns True if initialization has succedded, false otherwise.
///
bool MotorControl::init(const motorSpeed_t& motorSpeed)
{
    //sanity checks for motor definition

    if (motorSpeed.slow < motorSpeed.precise)
    {
        _motorSpeedValid = false;
    }
    else if (motorSpeed.fast < motorSpeed.slow)
    {
        _motorSpeedValid = false;
    }
    else if (!motorSpeed.precise || !motorSpeed.slow || !motorSpeed.fast)
    {
        _motorSpeedValid = false;
    }
    else
    {
        _motorSpeedValid = true;
    }

    if (_motorSpeedValid)
    {
        _movementPulsesSent = 0;
        _currentPosition    = 0;
        _movementPulses     = 0;
        _targetSpeed        = 0;
        _movementDirection  = direction_t::home;
        _initCallback       = nullptr;
        _stopCallback       = nullptr;

        alpha           = (2.0 * 3.14159) / motorSpeed.stepsPerRev;
        alphaSQ         = alpha * 2.0 * 10000000000.0;
        alphaxTimerx100 = alpha * timerFreq * 100.0;
        alphax20000     = alpha * 20000.0;

        _motorSpeed = &motorSpeed;

        _timer.init([this]() {
            this->sendPulse();
        });
    }
    else
    {
        _motorSpeed = nullptr;
    }

    return _motorSpeedValid;
}

///
/// \brief Verifies if there are any pending callbacks to be called.
/// This is used to prevent any callbacks being called from interrupt handlers.
///
void MotorControl::checkPendingCallbacks()
{
    if (_pendingStopCallback)
    {
        _pendingStopCallback = false;

        if (_stopCallback != nullptr)
            _stopCallback();
    }
}

void MotorControl::registerInitCallback(MotorControl::initCallback_t initCallback)
{
    _initCallback = std::move(initCallback);
}

void MotorControl::registerStopCallback(MotorControl::stopCallback_t stopCallback)
{
    _stopCallback = std::move(stopCallback);
}

///
/// \brief Verifies if movement is currently active.
/// @param[in, out] direction Current movement direction. Set only if the movement is active.
/// \returns True or false.
///
bool MotorControl::isMovementActive(direction_t& direction)
{
    if (isMovementActive())
    {
        direction = _movementDirection;
        return true;
    }

    return false;
}

///
/// \brief Verifies if movement is currently active.
/// \returns True or false.
///
bool MotorControl::isMovementActive()
{
    return _timer.isRunning();
}

///
/// \brief Used to start a motor movement event.
/// Only starts movement if no other movements for provided movement type are active and if user-specified checks pass.
/// @param [in] info    A structure containing motor direction, speed, pulse count and acceleration.
/// \returns            True on success.
///
bool MotorControl::setupMovement(motorMovement_t& info)
{
    if (!isMotorValid())
        return false;

    if (isMovementActive())
        return false;

    if (info.absoluteMovement)
    {
        if (_totalMovementLength)
        {
            if (info.pulses > _totalMovementLength)
                info.pulses = _totalMovementLength;
        }

        if (info.pulses < _currentPosition)
        {
            _movementPulses = _currentPosition - info.pulses;
            info.pulses     = _movementPulses;
            info.direction  = MotorControl::direction_t::home;
        }
        else if (info.pulses > _currentPosition)
        {
            _movementPulses = info.pulses - _currentPosition;
            info.pulses     = _movementPulses;
            info.direction  = MotorControl::direction_t::end;
        }
        else
        {
            //motor already at specified position
            return false;
        }
    }
    else
    {
        if (!info.pulses)
        {
            if (_totalMovementLength)
            {
                if (info.direction == direction_t::end)
                    info.pulses = _totalMovementLength - _currentPosition;
                else
                    info.pulses = _currentPosition;
            }
        }
        else
        {
            //this is considered infinite movement
        }

        _movementPulses = info.pulses;
    }

    if (_initCallback != nullptr)
    {
        if (!_initCallback(info))
            return false;
    }

    uint32_t stepsBeforeMaxSpeed     = 0;
    uint32_t stepsBeforeDeceleration = 0;

    _targetSpeed   = alphaxTimerx100 / speed(info.speed);
    _timerInterval = (timerFreqOffset * sqrt(alphaSQ / _motorSpeed->accelerationRPS2)) / 100;
    _startSpeed    = _timerInterval;

    stepsBeforeMaxSpeed = speed(info.speed) * speed(info.speed) / ((static_cast<uint32_t>(alphax20000) * _motorSpeed->accelerationRPS2) / 100);

    if (!stepsBeforeMaxSpeed)
        stepsBeforeMaxSpeed = 1;

    //don't calculate deceleration for infinite pulse amount
    if (info.pulses)
    {
        stepsBeforeDeceleration = (info.pulses * _motorSpeed->decelerationRPS2) / (_motorSpeed->accelerationRPS2 + _motorSpeed->decelerationRPS2);

        if (!stepsBeforeDeceleration)
            stepsBeforeDeceleration = 1;

        if (stepsBeforeDeceleration <= stepsBeforeMaxSpeed)
        {
            _decelerationRate = stepsBeforeDeceleration - info.pulses;
        }
        else
        {
            _decelerationRate = (stepsBeforeMaxSpeed * _motorSpeed->accelerationRPS2) / _motorSpeed->decelerationRPS2;
            _decelerationRate *= -1;
        }

        if (!_decelerationRate)
            _decelerationRate = -1;

        _decelerationPulseCountStart = info.pulses + _decelerationRate;
    }
    else
    {
        //in case of infinite amount of pulses, just calculate deceleration rate so that it can be used in case stopping is requested
        _decelerationRate = (stepsBeforeMaxSpeed * _motorSpeed->accelerationRPS2) / _motorSpeed->decelerationRPS2;
        _decelerationRate *= -1;
    }

    //if the speed is too small don't use acceleration at all
    if (_timerInterval <= _targetSpeed)
    {
        _timerInterval = _targetSpeed;
        _runState      = runState_t::running;
    }
    else
    {
        _runState = runState_t::accelerating;
    }

    writeDirection(info.direction);

    _timer.setPeriod(_timerInterval);
    _timer.start();

    return true;
}

bool MotorControl::accelerateToMaxSpeed()
{
    if (!isMovementActive())
        return false;

    int32_t maxSpeed = alphaxTimerx100 / speed(speed_t::fast);

    if (_targetSpeed == maxSpeed)
        return false;    //fast speed already set

    uint32_t stepsBeforeMaxSpeed = speed(speed_t::fast) * speed(speed_t::fast) / ((static_cast<uint32_t>(alphax20000) * _motorSpeed->accelerationRPS2) / 100);

    _timer.pause();
    _decelerationRate = (stepsBeforeMaxSpeed * _motorSpeed->accelerationRPS2) / _motorSpeed->decelerationRPS2;
    _decelerationRate *= -1;
    _timerIntervalCalcRemainder = 0;
    _targetSpeed                = maxSpeed;
    _runState                   = runState_t::accelerating;
    _timer.resume();

    return true;
}

///
/// \brief Used for motor movement.
///
void MotorControl::sendPulse()
{
    if (_movementDirection == direction_t::home)
    {
        if (_currentPosition > 0)
            _currentPosition--;
    }
    else
    {
        if (_totalMovementLength)
        {
            //make sure current position doesn't exceed specified length
            if (_currentPosition < _totalMovementLength)
                _currentPosition++;
        }
        else
        {
            _currentPosition++;
        }
    }

    if (_movementPulses)
    {
        if (_movementPulsesSent >= _movementPulses)
            _runState = runState_t::stopped;
    }

    if ((_runState == runState_t::decelerating) && _stopRequested)
    {
        //once the deceleration reaches minimum value, stop the movement
        //do this only if stopping is requested
        if (_accelerationCounter >= -1)
            _runState = runState_t::stopped;
    }

    int32_t newTimerInterval = _timerInterval;

    switch (_runState)
    {
    case runState_t::stopped:
    {
        resetMovement(false);
    }
    break;

    case runState_t::accelerating:
    {
        io.pulse();
        _movementPulsesSent++;
        _accelerationCounter++;
        newTimerInterval            = _timerInterval - (((2 * _timerInterval + _timerIntervalCalcRemainder)) / (4 * _accelerationCounter + 1));
        _timerIntervalCalcRemainder = ((2 * _timerInterval) + _timerIntervalCalcRemainder) % (4 * _accelerationCounter + 1);

        //check if max speed is achieved
        if (newTimerInterval <= _targetSpeed)
        {
            newTimerInterval            = _targetSpeed;
            _runState                   = runState_t::running;
            _timerIntervalCalcRemainder = 0;
        }
    }
    break;

    case runState_t::running:
    {
        io.pulse();
        _movementPulsesSent++;

        if (_movementPulses)
        {
            if (_movementPulsesSent >= _decelerationPulseCountStart)
            {
                //start decelerating
                _accelerationCounter = _decelerationRate;
                _runState            = runState_t::decelerating;
            }
        }
    }
    break;

    case runState_t::decelerating:
    {
        io.pulse();
        _movementPulsesSent++;

        if (_accelerationCounter < 0)
        {
            if (newTimerInterval < _startSpeed)
            {
                _accelerationCounter++;
                newTimerInterval            = _timerInterval - (((2 * _timerInterval + _timerIntervalCalcRemainder)) / (4 * _accelerationCounter + 1));
                _timerIntervalCalcRemainder = ((2 * _timerInterval) + _timerIntervalCalcRemainder) % (4 * _accelerationCounter + 1);
            }

            if ((newTimerInterval > _startSpeed) || (newTimerInterval < 0))
            {
                _accelerationCounter = 0;
                newTimerInterval     = _startSpeed;
            }
        }
    }
    break;

    default:
        break;
    }

    if (newTimerInterval != _timerInterval)
    {
        _timerInterval = newTimerInterval;
        _timer.setPeriod(static_cast<uint32_t>(_timerInterval));
        _timer.start();
    }
}

///
/// \brief Writes direction to motor driver.
/// @param [in] direction   Motor direction.
///
void MotorControl::writeDirection(direction_t direction)
{
    io.setDirection(direction);
    _movementDirection = direction;
}

///
/// \brief Retrieves actual motor speed based on provided enumerated type.
/// @param [in] speed       Motor speed (enumerated type). See MotorControl::speed_t enumeration.

uint32_t MotorControl::speed(speed_t enumSpeed)
{
    switch (enumSpeed)
    {
    case speed_t::precise:
        return _motorSpeed->precise;

    case speed_t::slow:
        return _motorSpeed->slow;

    case speed_t::fast:
        return _motorSpeed->fast;

    default:
        return 0;
    }
}

///
/// \brief Stops active movement.
/// @param [in] force   If set to true, movement will be stopped immediately without deceleration.
///
bool MotorControl::stopMovement(bool force)
{
    if (!isMotorValid())
        return false;

    if (!isMovementActive())
        return false;

    if (force)
    {
        resetMovement(true);
    }
    else
    {
        if (!_stopRequested)
        {
            _timer.pause();

            _stopRequested = true;

            switch (_runState)
            {
            case runState_t::accelerating:
                _timerIntervalCalcRemainder = 0;
                _accelerationCounter        = -_accelerationCounter;
                _runState                   = runState_t::decelerating;
                break;

            case runState_t::running:
                _timerIntervalCalcRemainder = 0;
                _accelerationCounter        = _decelerationRate;
                _runState                   = runState_t::decelerating;
                break;

            default:
                break;
            }

            _timer.resume();
        }
        else
        {
            //stopping already requested
            return false;
        }
    }

    return true;
}

void MotorControl::resetMovement(bool callHandler)
{
    _movementPulsesSent         = 0;
    _timerIntervalCalcRemainder = 0;
    _accelerationCounter        = 0;
    _stopRequested              = false;

    _timer.stop();

    if (callHandler)
    {
        if (_stopCallback != nullptr)
            _stopCallback();
    }
    else
    {
        _pendingStopCallback = true;
    }
}

///
/// \brief Returns current position (relative) in pulses.
///
uint32_t MotorControl::currentPosition()
{
    return _currentPosition;
}

///
/// \brief Resets current motor position to 0.
///
void MotorControl::resetPosition()
{
    _currentPosition = 0;
}

///
/// \brief Sets new motor movement length.
/// @param [in] value   New length.
///
void MotorControl::setTotalLength(uint32_t value)
{
    _totalMovementLength = value;
}

void MotorControl::setPosition(uint32_t value)
{
    if (_totalMovementLength)
    {
        if (value > _totalMovementLength)
            _currentPosition = _totalMovementLength;
        else
            _currentPosition = value;
    }
    else
    {
        _currentPosition = value;
    }
}

uint32_t MotorControl::totalLength()
{
    return _totalMovementLength;
}

bool MotorControl::isMotorValid()
{
    return _motorSpeedValid;
}

bool MotorControl::currentSpeed(speed_t& _speed)
{
    if (!isMovementActive())
        return false;

    int32_t fastSpeed    = (alphaxTimerx100 / speed(speed_t::fast));
    int32_t slowSpeed    = (alphaxTimerx100 / speed(speed_t::slow));
    int32_t preciseSpeed = (alphaxTimerx100 / speed(speed_t::precise));

    if (_targetSpeed == fastSpeed)
    {
        _speed = speed_t::fast;
        return true;
    }
    else if (_targetSpeed == slowSpeed)
    {
        _speed = speed_t::slow;
        return true;
    }
    else if (_targetSpeed == preciseSpeed)
    {
        _speed = speed_t::precise;
        return true;
    }

    return false;
}