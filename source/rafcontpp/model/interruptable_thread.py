# Copyright (C) 2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the 3-Clause BSD License which accompanies this
# distribution, and is available at
# https://opensource.org/licenses/BSD-3-Clause
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

# Don't connect with the Copyright comment above!
# Version 12.07.2019


import threading

# A lock to synchronize planning thread map accesses.
interruptable_threads_lock = threading.Lock()
# a dictionary of all running interruptable threads.
interruptable_threads = {}


def current_thread():
    """
    current thread returns the current interruptable thread, it was called from.

    :return: InterruptableThread: The current interruptable thread, or None if current thread is not interruptable.
    """
    current_thread_id = threading.current_thread().ident
    current_thread = None
    with interruptable_threads_lock:
        if current_thread_id in interruptable_threads:
            current_thread = interruptable_threads[current_thread_id]
    return current_thread


class InterruptableThread(threading.Thread):
    """
    It's a usual thread, but its interruptable.
    """

    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        """
        A thread that is interruptable.
        """
        threading.Thread.__init__(self, group, target, name, args, kwargs)
        self.__interrupted_flag = threading.Event()

    def run(self):
        # add new thread to list of all interruptable threads
        with interruptable_threads_lock:
            interruptable_threads[self.ident] = self
        #execute threads run method
        super(InterruptableThread, self).run()

        #remove thread from list of all interruptable threads
        with interruptable_threads_lock:
            elem = interruptable_threads.pop(self.ident, None)

    def interrupt(self):
        """
        Sets the Threads interrupted flag.
        """
        self.__interrupted_flag.set()

    def is_interrupted(self):
        """
        :return: Boolean: True if the Interrupted Flag is set. False otherwise.
        """
        return self.__interrupted_flag.is_set()
