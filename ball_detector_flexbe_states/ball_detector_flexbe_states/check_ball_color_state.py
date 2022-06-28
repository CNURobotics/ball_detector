#!/usr/bin/env python
from flexbe_core import EventState, Logger


class CheckBallColorState(EventState):
    '''
    Checks the ball color

    ># ball_label	   string           The color of the ball

    <= red_ball                         Red ball detected
    <= green_ball                       Green ball detected
    <= blue_ball                        Blue ball detected
    '''

    def __init__(self):
        super(CheckBallColorState, self).__init__(outcomes=['red_ball', 'green_ball', "blue_ball", "failed"],
                                                  input_keys=['ball_label'])
        self._outcome = ''

    def execute(self, userdata):
        return self._outcome

    def on_enter(self, userdata):
        try:
            if userdata.ball_label in self.outcomes:
                self._outcome = userdata.ball_label
                Logger.loginfo('Ball Color Detected %s' % userdata.ball_label)
            else:
                Logger.logwarn(f'Invalid ball label <{userdata.ball_label}>')
                self._outcome = 'failed'
        except Exception as exc:
            Logger.logerr(f'Exception handling ball label {exc} in {self.name}')
            self._outcome = 'failed'
