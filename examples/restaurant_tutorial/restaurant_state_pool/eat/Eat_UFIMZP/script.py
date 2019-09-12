import time
def execute(self, inputs, outputs, gvm):
    self.logger.info(inputs['person']+' is eating '+inputs['food'])
    time.sleep(0.5)
    return 0
