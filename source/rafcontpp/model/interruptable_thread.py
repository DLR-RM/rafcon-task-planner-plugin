import threading


class InterruptableThread(threading.Thread):



    def __init__(self,group=None, target=None, name=None, args=(), kwargs={}):
        '''
        Important: Interruptable thread addes itself to the beginning of the args, to allow the function to use
        the interrupt mechanism. e.g. args=(arg1,arg2) ==> args=(interruptableThreadObject,arg1,arg2)
        :param group:
        :param target:
        :param name:
        :param args:
        :param kwargs:
        '''
        threading.Thread.__init__(self,group,target,name,(self,)+args,kwargs)
        self.__interrupt_flag = threading.Event()


    def interrupt(self):
        self.__interrupt_flag.set()

    def is_interrupted(self):
        return self.__interrupt_flag.is_set()



