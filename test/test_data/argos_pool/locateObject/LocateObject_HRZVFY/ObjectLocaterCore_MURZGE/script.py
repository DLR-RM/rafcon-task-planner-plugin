import time
def execute(self, inputs, outputs, gvm):
    object = inputs['objToLocate']
    self.logger.info("locating...")
    time.sleep(2)
    self.logger.info("Object \""+object+"\" is now located.")
    return 0

