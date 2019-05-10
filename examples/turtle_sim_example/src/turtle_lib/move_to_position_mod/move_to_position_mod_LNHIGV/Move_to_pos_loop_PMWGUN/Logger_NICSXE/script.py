
def execute(self, inputs, outputs, gvm):
    pddl_map = gvm.get_variable('rtpp_data')
    turtle = pddl_map[inputs['turtle']]
    turtle_name = turtle["name"]
    self.logger.info('{} is moving to {}'.format(turtle_name, inputs['destination']))
    return 0
