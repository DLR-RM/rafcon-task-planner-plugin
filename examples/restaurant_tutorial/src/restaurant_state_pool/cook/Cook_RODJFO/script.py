import time
def execute(self, inputs, outputs, gvm):
    self.logger.info(inputs['chef']+' is cooking '+inputs['food'])
    time.sleep(1.5)
    return 0
