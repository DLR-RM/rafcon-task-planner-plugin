import time
def execute(self, inputs, outputs, gvm):
    self.logger.info(inputs['name']+' is moving to '+inputs['destination'])
    time.sleep(0.5)
    return 0
