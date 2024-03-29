# encoding: utf-8

import datetime
import imp
import inspect
import logger
import os
import traceback

import eventloop




class StateMachine(object):

    def __init__(self, event_loop, name, **kwargs):
        self.state_stack = []
        """:type: self.state_stack : list of State"""
        self.pending_states = []
        self.event_loop = event_loop
        self.name = name
        self.timer = eventloop.Timer(event_loop, None, self.on_timeout)
        for k, v in kwargs.items():
            setattr(self, k, v)
        main_state = self.instantiate_state_machine(self.name)
        if main_state is not None:
            self.event_loop.fsms.append(self)
            self.process(self.push_state, main_state)


    def instantiate_state_machine(self, state_machine_name):
        state_machines_dir = os.path.join(os.path.dirname(__file__), "statemachines")
        state_machine_file = os.path.join(state_machines_dir, state_machine_name + ".py")
        state_machine_module = imp.load_source(state_machine_name, state_machine_file)
        main_state = None
        for (item_name, item_type) in inspect.getmembers(state_machine_module):
            if inspect.isclass(item_type) and issubclass(item_type, State):
                if item_name == "Main":
                    main_state = item_type()
                    self.log("Successfully instatiated state '{}' from file '{}'".format(item_name, state_machine_file))
                    break
        if main_state is None:
            self.log("Error: no 'Main' state found in '{}'".format(state_machine_file))
        return main_state


    def preempt_with_state(self, state):
        self.state_stack = []
        self.process(self.push_state, state)


    def init_state(self, s):
        s.fsm_current_method = None
        s.fsm = self
        s.event_loop = self.event_loop
        s.robot = self.event_loop.robot


    @property
    def current_state(self):
        """
        :return: Current state
        :rtype: State
        """
        return self.state_stack[-1] if self.state_stack else None


    @property
    def current_state_name(self):
        state = self.current_state
        if state is not None:
            return state.name
        else:
            return "(None)"


    def log(self, msg):
        logger.log(self.name + ": " + str(msg))


    def log_exception(self, exc):
        self.log("")
        for l in traceback.format_exception(type(exc), exc, None):
            for ll in l.splitlines():
                logger.log(self.name + ": " + ll, "ARM", True)
        self.log("")


    def log_error(self, msg):
        logger.error(msg)


    def dbg(self, msg):
        logger.dbg(self.name + ": " + msg)


    def on_packet(self, packet):
        if self.current_state is not None:
            self.process(packet.dispatch_generator, self.current_state)


    def push_state(self, state):
        self.log("Switching to state {new} ({old} -> {new})".format(old = self.current_state_name, new = state.name))
        self.init_state(state)
        self.state_stack.append(state)
        if isinstance(state, Timer):
            state.restart()
        return state.on_enter()


    def pop_state(self):
        previous_state = self.current_state
        self.state_stack.pop()
        self.log("Exiting state {old} ({old} -> {new})".format(old = previous_state.name, new = self.current_state_name))
        current_state = self.current_state
        if isinstance(previous_state, Timer):
            self.log("Stopping timer")
            self.timer.stop()
        if current_state is not None:
            return previous_state, self.current_state.fsm_current_method
        else:
            return previous_state, None


    def handle_timer(self):
        cs = self.current_state
        if isinstance(cs, Timer):
            if cs._timeout_date is not None:
                if cs._timeout_date != self.timer.timeout_date:
                    self.timer.restart(cs._timeout_date)


    def process(self, method, *args):
        generator = None
        previous_state = None
        previous_exception = None

        try:
            generator = method(*args)
        except BaseException as e:
            previous_exception = e
            if not hasattr(previous_exception, "current_state_name"):
                previous_exception.current_state_name = self.current_state_name
            previous_state, generator = self.pop_state()

        while generator:
            try:
                try:
                    new_state = None
                    if previous_exception is not None:
                        exception = previous_exception
                        previous_exception = None
                        new_state = generator.throw(exception)
                    else:
                        new_state = generator.send(previous_state)
                except BaseException as e:
                    if not isinstance(e, StopIteration):
                        previous_exception = e
                        if not hasattr(previous_exception, "current_state_name"):
                            previous_exception.current_state_name = self.current_state_name
                    else:
                        raise e
                if isinstance(new_state, State):
                    previous_state = None
                    # on_enter can yield a generator
                    self.current_state.fsm_current_method = generator
                    try:
                        generator = self.push_state(new_state)
                    except BaseException as e:
                        previous_exception = e
                        if not hasattr(previous_exception, "current_state_name"):
                            previous_exception.current_state_name = self.current_state_name
                        previous_state, generator = self.pop_state()
                elif new_state is None:
                    # yield None or exception means exit current State
                    previous_state, generator = self.pop_state()
            except StopIteration:
                generator = None
        if previous_exception is not None:
            # The raised exception hasn't been handled by any state in the stack
            self.log("An exception occured while in state '{}':".format(previous_exception.current_state_name))
            self.log_exception(previous_exception)
        if self.current_state is None and len(self.pending_states) == 0:
            logger.log("State machine '{}' has no current state or pending states, exiting".format(self.name))
            self.event_loop.fsms.remove(self)
        else:
            self.handle_timer()


    def on_timeout(self):
        cs = self.current_state
        if isinstance(cs, Timer):
            cs.stop()
            self.process(cs.on_timeout)
        else:
            self.log("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.log("!! on_timeout called wheras the current state isn't a Timer !!")
            self.log("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")




class Delayed:

    def __init__(self, event_loop, timeout_ms, fsm, state):
        eventloop.Timer(event_loop, timeout_ms, self.on_timeout).start()
        self.fsm = fsm
        self.state = state
        self.fsm.pending_states.append(self)


    def on_timeout(self):
        self.fsm.pending_states.remove(self)
        self.fsm.preempt_with_state(self.state)




class State:

    @property
    def name(self):
        return self.__class__.__name__


    def log(self, msg):
        self.fsm.log(msg)


    def log_exception(self, exc):
        self.fsm.log_exception(exc)

    def log_error(self, msg):
        self.fsm.log_error(msg)

    def dbg(self, msg):
        self.fsm.dbg(msg)


    def send_packet(self, packet):
        self.event_loop.send_packet(packet, self.fsm)


    def yield_at(self, timeout_ms, state):
        Delayed(self.event_loop, timeout_ms, self.fsm, state)


    def on_enter(self):
        pass




class Timer(State):

    def __init__(self, timeout_ms, single_shot = True):
        super().__init__()
        self._timeout_ms = timeout_ms
        self._single_shot = single_shot
        self._timeout_date = None


    def has_timed_out(self):
        if self._timeout_date is None:
            return False
        else:
            return datetime.datetime.now() >= self._timeout_date


    def restart(self, timeout_ms = None):
        if timeout_ms is not None:
            self._timeout_ms = timeout_ms
        self._timeout_date = datetime.datetime.now() + datetime.timedelta(milliseconds = self._timeout_ms)


    def stop(self):
        self._timeout_date = None


    def on_timeout(self):
        yield None
