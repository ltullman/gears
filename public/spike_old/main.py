import simPython, time

# Needed to prevent loops from locking up the javascript thread
SENSOR_DELAY = 0.001

class PrimeHub:
  def __init__(self, address=None):
    self._buttons = simPython.HubButtons()
    self.on_up = None
    self.on_down = None
    self.on_left = None
    self.on_right = None
    self.on_enter = None
    self.on_backspace = None
    self.on_change = None

  @property
  def buttons_pressed(self):
    time.sleep(SENSOR_DELAY)
    return self._buttons.ev3dev_buttons_pressed()

  def any(self):
    time.sleep(SENSOR_DELAY)
    return self._buttons.ev3dev_any()



class Motor:
  COMMAND_RESET = 'reset'
  COMMAND_RUN_DIRECT = 'run-direct'
  COMMAND_RUN_FOREVER = 'run-forever'
  COMMAND_RUN_TIMED = 'run-timed'
  COMMAND_RUN_TO_ABS_POS = 'run-to-abs-pos'
  COMMAND_RUN_TO_REL_POS = 'run-to-rel-pos'
  COMMAND_STOP = 'stop'
  ENCODER_POLARITY_INVERSED = 'inversed'
  ENCODER_POLARITY_NORMAL = 'normal'
  POLARITY_INVERSED = 'inversed'
  POLARITY_NORMAL = 'normal'
  STATE_HOLDING = 'holding'
  STATE_OVERLOADED = 'overloaded'
  STATE_RAMPING = 'ramping'
  STATE_RUNNING = 'running'
  STATE_STALLED = 'stalled'
  STOP_ACTION_BRAKE = 'brake'
  STOP_ACTION_COAST = 'coast'
  STOP_ACTION_HOLD = 'hold'

  _DRIVER_NAME = None

  def __init__(self, address=None):
    self.wheel = simPython.Motor(address)

    self.max_speed = 1050
    self.count_per_rot = 360
    self.max_rps = float(self.max_speed/self.count_per_rot)
    self.max_rpm = self.max_rps * 60
    self.max_dps = self.max_rps * 360
    self.max_dpm = self.max_rpm * 360

  @property
  def command(self):
    raise Exception("command is a write-only property!")

  @command.setter
  def command(self, value):
    self.wheel.command(value)
    return 0

  @property
  def duty_cycle(self):
    return self.wheel.speed()

  @property
  def duty_cycle_sp(self):
    return self._duty_cycle_sp

  @duty_cycle_sp.setter
  def duty_cycle_sp(self, value):
    self._duty_cycle_sp = value
    self.wheel.speed_sp(int(value))
    return 0

  @property
  def is_holding(self):
    return self.STATE_HOLDING in self.wheel.state()

  @property
  def is_overloaded(self):
    return self.STATE_OVERLOADED in self.wheel.state()

  @property
  def is_ramping(self):
    return self.STATE_RAMPING in self.wheel.state()

  @property
  def is_running(self):
    return self.STATE_RUNNING in self.wheel.state()

  @property
  def is_stalled(self):
    return self.STATE_STALLED in self.wheel.state()

  @property
  def polarity(self):
    return self.wheel.polarity()

  @polarity.setter
  def polarity(self, value):
    self.wheel.polarity(value)
    return 0

  @property
  def position(self):
    time.sleep(SENSOR_DELAY)
    return int(self.wheel.position())

  @position.setter
  def position(self, value):
    self.wheel.position(int(value))
    return 0

  @property
  def position_d(self):
    return 1

  @position_d.setter
  def position_d(self, value):
    return 0

  @property
  def position_i(self):
    return 1

  @position_i.setter
  def position_i(self, value):
    return 0

  @property
  def position_p(self):
    return 1

  @position_p.setter
  def position_p(self, value):
    return 0

  @property
  def position_sp(self):
    return self.wheel.position_sp()

  @position_sp.setter
  def position_sp(self, value):
    self.wheel.position_sp(int(value))
    return 0

  @property
  def ramp_down_sp(self):
    return 1

  @ramp_down_sp.setter
  def ramp_down_sp(self, value):
    return 0

  @property
  def ramp_up_sp(self):
    return 1

  @ramp_up_sp.setter
  def ramp_up_sp(self, value):
    return 0

  @property
  def speed(self):
    time.sleep(SENSOR_DELAY)
    return int(self.wheel.speed())

  @property
  def speed_d(self):
    return 1

  @speed_d.setter
  def speed_d(self, value):
    return 0

  @property
  def speed_i(self):
    return 1

  @speed_i.setter
  def speed_i(self, value):
    return 0

  @property
  def speed_p(self):
    return 1

  @speed_p.setter
  def speed_p(self, value):
    return 0

  @property
  def speed_sp(self):
    return self.wheel.speed_sp()

  @speed_sp.setter
  def speed_sp(self, value):
    self.wheel.speed_sp(int(value))
    return 0

  @property
  def state(self):
    return self.wheel.state()

  @property
  def stop_action(self):
    return self.wheel.stop_action()

  @stop_action.setter
  def stop_action(self, value):
    self.wheel.stop_action(value)
    return 0

  @property
  def time_sp(self):
    return self.wheel.speed_sp()

  @time_sp.setter
  def time_sp(self, value):
    self.wheel.time_sp(value)
    return 0

  def reset(self, **kwargs):
    """
    Resets the motor the default value. It will also stop the motor.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    self.command = 'reset'

  def run_direct(self, **kwargs):
    """
    Run the motor at the duty cycle specified by duty_cycle_sp.
    Unlike other run commands, changing duty_cycle_sp
    while running will take effect immediately.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])

  def run_forever(self, **kwargs):
    """
    Run the motor until another command is sent.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    self.command = 'run-forever'

  def run_timed(self, **kwargs):
    """
    Run for the amount of time specified in time_sp.
    Then, stop the motor as specified by stop_action.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    self.command = 'run-timed'

  def run_to_abs_pos(self, **kwargs):
    """
    Run to the absolute position as specified by position_sp.
    Then, stop the motor as specified by stop_action.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    self.command = 'run-to-abs-pos'

  def run_to_rel_pos(self, **kwargs):
    """
    Run to the relative position as specified by position_sp.
    New position will be current position + position_sp
    When the new position is reached, the motor will stop, as specified
    by stop_action.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    self.command = 'run-to-rel-pos'

  def stop(self, **kwargs):
    """
    Stop any of the run commands before they are complete using the
    action specified by stop_action.
    """
    for k in kwargs:
      setattr(self, k, kwargs[k])
    self.command = ''

  def wait(self, cond, timeout=None):
    """
    Blocks until ``cond(self.state)`` is ``True``.  The condition is
    checked when there is an I/O event related to the ``state`` attribute.
    Exits early when ``timeout`` (in milliseconds) is reached.

    Returns ``True`` if the condition is met, and ``False`` if the timeout
    is reached.

    Valid flags for state attribute: running, ramping, holding,
    overloaded and stalled
    """
    if timeout != None:
      timeout = time.clock() + timeout / 1000.0
    while True:
      time.sleep(0.01)
      if cond(str(self.wheel.state())):
        return True
      if timeout != None and time.clock() >= timeout:
        return False

  def wait_until(self, s, timeout=None):
    """
    Blocks until ``s`` is in ``self.state``.  The condition is checked when
    there is an I/O event related to the ``state`` attribute.  Exits early
    when ``timeout`` (in milliseconds) is reached.

    Returns ``True`` if the condition is met, and ``False`` if the timeout
    is reached.

    Example::
        m.wait_until('stalled')
    """
    return self.wait(lambda state: s in state, timeout)

  def wait_until_not_moving(self, timeout=None):
    """
    Blocks until one of the following conditions are met:
    - ``running`` is not in ``self.state``
    - ``stalled`` is in ``self.state``
    - ``holding`` is in ``self.state``
    The condition is checked when there is an I/O event related to
    the ``state`` attribute.  Exits early when ``timeout`` (in
    milliseconds) is reached.

    Returns ``True`` if the condition is met, and ``False`` if the timeout
    is reached.

    Example::

        m.wait_until_not_moving()
    """
    return self.wait(lambda state: (self.STATE_RUNNING not in state and self.STATE_RAMPING not in state) or self.STATE_STALLED in state, timeout)

  def wait_while(self, s, timeout=None):
    """
    Blocks until ``s`` is not in ``self.state``.  The condition is checked
    when there is an I/O event related to the ``state`` attribute.  Exits
    early when ``timeout`` (in milliseconds) is reached.

    Returns ``True`` if the condition is met, and ``False`` if the timeout
    is reached.

    Example::

        m.wait_while('running')
    """
    return self.wait(lambda state: s not in state, timeout)

  def _set_rel_position_degrees_and_speed_sp(self, degrees, speed):
    degrees = degrees if speed >= 0 else -degrees
    speed = abs(speed)

    position_delta = int(round((degrees * self.count_per_rot)/360))
    speed_sp = int(round(speed))

    self.position_sp = position_delta
    self.speed_sp = speed_sp

  def on_for_rotations(self, speed, rotations, brake=True, block=True):
    """
    Rotate the motor at ``speed`` for ``rotations``

    ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
    object, enabling use of other units.
    """
    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed = SpeedPercent(speed)
        speed_sp = speed.to_native_units(self)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_sp = int(round(speed.to_native_units(self)))

    self._set_rel_position_degrees_and_speed_sp(rotations*360, speed_sp)

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.run_to_rel_pos()

    if block:
      self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
      self.wait_until_not_moving()

  def on_for_degrees(self, speed, degrees, brake=True, block=True):
    """
    Rotate the motor at ``speed`` for ``degrees``

    ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
    object, enabling use of other units.
    """
    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed = SpeedPercent(speed)
        speed_sp = speed.to_native_units(self)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_sp = int(round(speed.to_native_units(self)))

    self._set_rel_position_degrees_and_speed_sp(degrees, speed_sp)

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.run_to_rel_pos()

    if block:
      self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
      self.wait_until_not_moving()

  def on_to_position(self, speed, position, brake=True, block=True):
    """
    Rotate the motor at ``speed`` to ``position``

    ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
    object, enabling use of other units.
    """
    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed = SpeedPercent(speed)
        speed_sp = speed.to_native_units(self)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_sp = int(round(speed.to_native_units(self)))

    self.speed_sp = int(round(speed_sp))
    self.position_sp = position

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.run_to_abs_pos()

    if block:
      self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
      self.wait_until_not_moving()

  def on_for_seconds(self, speed, seconds, brake=True, block=True):
    """
    Rotate the motor at ``speed`` for ``seconds``

    ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
    object, enabling use of other units.
    """
    if seconds < 0:
      raise ValueError("Seconds is negative.")

    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed = SpeedPercent(speed)
        speed_sp = speed.to_native_units(self)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_sp = int(round(speed.to_native_units(self)))

    self.speed_sp = int(round(speed_sp))
    self.time_sp = seconds

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.run_timed()

    if block:
      self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
      self.wait_until_not_moving()

  def on(self, speed, brake=True, block=False):
    if not isinstance(speed, SpeedValue):
      if -100 <= speed <= 100:
        speed = SpeedPercent(speed)
        speed_sp = speed.to_native_units(self)
      else:
        raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
    else:
      speed_sp = int(round(speed.to_native_units(self)))

    self.speed_sp = int(round(speed_sp))

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.run_forever()

    if block:
      self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
      self.wait_until_not_moving()

  def off(self, brake=True):

    if brake:
      self.stop_action = self.STOP_ACTION_HOLD
    else:
      self.stop_action = self.STOP_ACTION_COAST

    self.stop()




class LightMatrix:
  def __init__(self, address=None):
    self._buttons = simPython.HubButtons()
    self.on_up = None
    self.on_down = None
    self.on_left = None
    self.on_right = None
    self.on_enter = None
    self.on_backspace = None
    self.on_change = None

  @property
  def buttons_pressed(self):
    time.sleep(SENSOR_DELAY)
    return self._buttons.ev3dev_buttons_pressed()

  def any(self):
    time.sleep(SENSOR_DELAY)
    return self._buttons.ev3dev_any()