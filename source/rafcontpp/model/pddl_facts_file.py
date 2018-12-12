
from rafcon.utils import log
logger = log.get_logger(__name__)


class FactsFile:
    #the problem name
    __problem_name = ''
    #the domain name
    __domain_name = ''
    #a map of variable_name:type
    __objects = {}
    #a list of predicates as strings
    __initial_predicates = []
    #a list of predicates as strings
    __goal_prediactes = []