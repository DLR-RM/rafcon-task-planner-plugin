# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 05.07.2019
import threading

#A lock to synchronize planning thread map accesses.
interruptable_threads_lock = threading.Lock()
#a dictionary of all interruptable threads.
interruptable_threads = {}

def current_thread():
    '''
    :return: the current interruptable thread, or None if current thread is not interruptable.
    '''
    current_thread = threading.current_thread().ident
    with interruptable_threads_lock:
        if current_thread in interruptable_threads:
            return interruptable_threads[current_thread]
    return None


class InterruptableThread(threading.Thread):
    '''
    Its a usual Thread, but its interruptable.
    In order to achieve this, it adds itself to the args of a given function.
    '''



    def __init__(self,group=None, target=None, name=None, args=(), kwargs={}):
        '''
        A thread that is interruptable.
        '''
        threading.Thread.__init__(self,group,target,name,args,kwargs)
        self.__interrupted_flag = threading.Event()


    def __del__(self):

            with interruptable_threads_lock:
                interruptable_threads.pop(threading.get_ident())

    def run(self):
        # add new thread to list of all interruptable threads
        # can't be done in init, because during init phase no ident is present.
        with interruptable_threads_lock:
            interruptable_threads[self.ident] = self

        super(InterruptableThread, self).run()
        




    def interrupt(self):
        '''
        Sets the Threads interrupted flag.
        '''
        self.__interrupted_flag.set()

    def is_interrupted(self):
        '''
        :return: True if the Interrupted Flag is set. False Otherwise.
        '''
        return self.__interrupted_flag.is_set()




