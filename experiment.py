import numpy as np



class Experiment:
    # This class basically holds a record of all sorts of
    # relevant information pertaining to this experiment, the
    # schedule, etc.

    def __init__(self,name):
        self.experiment = name

    def schedule_done(self):
        # Returns True if the schedule is done
        return self.current_schedule>=len(self.schedule)


    def tr_waiting_for(self):
        # Returns the TR we are currently waiting for
        if self.schedule_done():
            return np.nan
        else:
            return self.schedule[ self.current_schedule ]["TR"]


    def t_offset_waiting_for(self):
        # Returns the offset time we are waiting for
        if self.schedule_done():
            return np.nan
        else:
            return self.schedule[ self.current_schedule ]["t.offset"]
        

    def tr_time(self,n):
        # Returns the time of a particular TR signal number n
        # Note that tr_time(1) will return the time of the first trigger,
        # i.e. the 0th item in the trigger_times list.
        if n>len(self.trigger_times):
            return np.nan
        else:
            return self.trigger_times[n-1]



    def first_trigger_t(self):
        # Returns the time of the first trigger,
        # or nan if there has not yet been a first trigger.
        if len(self.trigger_times)==0:
            return np.nan
        else:
            return self.trigger_times[0]

