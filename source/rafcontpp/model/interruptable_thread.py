# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 07.03.2019
import threading


class InterruptableThread(threading.Thread):
    '''
    Its a usual Thread, but its interruptable.
    In order to achieve this, it adds itself to the args of a given function.
    '''



    def __init__(self,group=None, target=None, name=None, args=(), kwargs={}):
        '''
        Important: Interruptable thread addes itself to the beginning of the args, to allow the function to use
        the interrupt mechanism. e.g. args=(arg1,arg2) ==> args=(interruptableThreadObject,arg1,arg2)
        For further Documentation see threading.Thread
        '''
        threading.Thread.__init__(self,group,target,name,(self,)+args,kwargs)
        self.__interrupted_flag = threading.Event()


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




