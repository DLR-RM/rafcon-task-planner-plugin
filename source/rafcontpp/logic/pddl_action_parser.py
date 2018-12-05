#
#
#
#
#
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version: 26.11.2018
import re
from rafcontpp.model.pddl_action_representation import PddlActionRepresentation
from rafcon.utils import log

logger = log.get_logger(__name__)


class PddlActionParser:
    ''' PddlActionParser
    PddlActionParser is able to parse an pddl action string e.g. a usual pddl action into a PddlActionRepresentation
    by extracting predicates, types, variables and the action name.

    '''

    def __init__(self, action_string):
        '''

        :param action_string: a pddl action string
        '''
        #matches variables with types e.g ?obj - Physobj
        self.__type_var_pattern = re.compile('((\?[^\s]+\s+)+-\s+[^\s|^\)]+)')
        #matches only type strings result  f.e. Location
        self.__type_pattern = re.compile('-\s+([^\s|^\)|^?|^-]+)')
        #matches only varialbes f.e. ?myVar
        self.__var_pattern = re.compile('\?[^\s|^\)]+')
        #matches predicates
        self.__predicate_pattern = re.compile('\({1}\s*[^\?|^\s][^\)|^\(]*\){1}')
        #matches the action name, ignores cases
        self.__action_name_pattern = re.compile('\(:action\s*([^\s|^:]+)', re.IGNORECASE)
        #a dictionary, which contains all variables and their types.
        self.__var_type_dict = {}
        #the pddl action as string
        self.__action_string = action_string

    def parse_action(self):
        '''parse_action
        parse_action takes the given action string, and parses it into a PddlActionRepresenation,
        raises ValueError, if action is none or empty.
        :return: a PddlActionRepresentation of the pddl action
        '''

        if self.__action_string is None or len(self.__action_string) == 0:
            raise ValueError('Can not parse action from None or Empty String!')

        self.__create_var_type_dict()
        name = self.parse_action_name()
        predicates = self.__parse_and_generalize_predicates()
        types = list(set(self.__var_type_dict.values()))
        parameters = self.parse_parameters()

        return PddlActionRepresentation(name, self.__action_string, predicates, types, [], parameters)


    def parse_action_name(self):
        '''parse_action_name
        parse_action_name reads the action form the given action and returns it.

        :return: The name of the action as string
        '''
        if self.__action_string is None or len(self.__action_string) == 0:
            logger.error('Can not parse action name from None or Empty String!')
            raise ValueError('Can not parse action name from None or Empty String!')
        parsed = re.findall(self.__action_name_pattern,self.__action_string)
        if len(parsed) == 0:
            logger.error("Couldn't find action name in "+self.__action_string)
            raise ValueError("Couldn't parse action name!")
        return parsed[0]

    def parse_parameters(self):
        '''parse_parameters
            parse_parameters parses the parameters out of an pddl action string.
        :return: a list with parameter names
        '''
        params_pattern = re.compile('(\?[^\s|^\)]*)\s+-')
        params = [param.strip('?') for param in re.findall(params_pattern,self.__action_string)]
        return params



    def __create_var_type_dict(self):
        '''create_var_type_dict
        create_var_type_dict creates a dictionary, which contains all variables with their type,
        defined in the action.
        :return: a dictionary contining variable : type pairs.
        '''
        type_vars = self.__type_var_pattern.findall(self.__action_string)
        for type_var in type_vars:
            types = self.__type_pattern.findall(type_var[0])
            type = types[0]
            for var in self.__var_pattern.findall(type_var[0]):
                 self.__var_type_dict[var] = type
        return self.__var_type_dict


    def __parse_and_generalize_predicates(self):
        '''parse and generalize predicates
        this method extracts all used pradicates from the action, then it generalizes them.
        e.g. add types to the variables and remove dublicats.
        used predicate example:       (at ?a ?b)
        generalized predicate example: (at ?a - Location ?b - Robot)

        :return: a list with all parsed predicates.
        '''
        #matches used predicates
        u_pred_name_pattern = re.compile('\(([^\s]+)\s')
        used_predicates = re.findall(self.__predicate_pattern,self.__action_string)

        if not self.__var_type_dict:
            self.__create_var_type_dict()

        #change used predicates to normal ones

        parsed_predicates = {}
        #iterate through all used predicates
        for used_predicate in used_predicates:
            generalized_predicate = '('
            c_pred_name = u_pred_name_pattern.findall(used_predicate)[0]
            c_pred_vars = self.__var_pattern.findall(used_predicate)
            generalized_predicate += c_pred_name
            #the last type used
            last_type = ''
            #a concatination of all types, used to produce an identifier for the predicate
            type_concat = ''
            #iterate through all variables of the predicate
            for c_pred_var in c_pred_vars:
                if c_pred_var in self.__var_type_dict:
                    c_type = self.__var_type_dict[c_pred_var]
                    if last_type == '':
                        last_type = c_type
                        type_concat += c_type
                    if last_type != c_type:
                        generalized_predicate += ' - '+last_type
                        last_type = c_type
                        type_concat += c_type
                    generalized_predicate += ' '+c_pred_var
                else:
                    raise ValueError('Variable: ' + c_pred_var+' not defined!')
            generalized_predicate += ' - ' + last_type + ')'
            #add predicate to a dictionary, to eliminate duplicats.
            #two predicates with the same name, but different types are handled as two
            #predicates at this time. Because of unknowen type hierarchies, its not decidable
            # at this time how to merge the predicates.
            parsed_predicates[c_pred_name+type_concat] = generalized_predicate
        return parsed_predicates.values()



