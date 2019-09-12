import time
def execute(self, inputs, outputs, gvm):
    self.logger.info(inputs['interlocutor1']+' gives '+inputs['food']+' to '+inputs['interlocutor2'])
    time.sleep(0.5)
    return 0
