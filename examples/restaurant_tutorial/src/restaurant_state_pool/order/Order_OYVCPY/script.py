import time
def execute(self, inputs, outputs, gvm):
    self.logger.info(inputs['interlocutor2']+' is ordering '+inputs['food']+' from '+inputs['interlocutor1'])
    time.sleep(0.5)
    return 0
