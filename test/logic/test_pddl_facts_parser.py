import pytest

from rafcontpp.logic.pddl_facts_parser import PddlFactsParser


@pytest.mark.parametrize("obj_section",[
    (' '),
    ('(:objects)'),
    ('aseeesafffasadfsafsa'),
    ('(:Objects a b c - Type b - Type2)')


])

def test_error_parse_object(obj_section):
    #arrange
    sut = PddlFactsParser(obj_section)
    #act
    with pytest.raises(ValueError):
        sut.parse_objects()

@pytest.mark.parametrize('obj_section,expected',[
    ('(:Objects a b c - Type)',{'a':'Type','b':'Type','c':'Type'}),
    ('(:Objects a b c c - Type)',{'a':'Type','b':'Type','c':'Type'}),
    ('(:Objects a b c - Type d e f   -   type2)',{'a':'Type','b':'Type','c':'Type','d':'type2','e':'type2','f':'type2'}),



])
def test_parse_object(obj_section,expected):
    # arrange
    sut = PddlFactsParser(obj_section)
    #act
    map = sut.parse_objects()
    #assert
    assert expected == map


@pytest.mark.parametrize('domain_name_sec, expected',[
    ('(:domain my_domain)','my_domain'),
    ('(:DOMAIN my_domain)', 'my_domain'),
    ('(:doMain my_domain)', 'my_domain'),
    ('(  :domain my_domain)', 'my_domain'),
    ('(:domain my_domain  )', 'my_domain'),
    ('(:domain      my_domain)', 'my_domain'),
    ('(:domain MY_DOMAIN)','MY_DOMAIN'),
    ('(:domain my-domain)','my-domain'),
    ('(:domain a)', 'a')

])
def test_parse_domain_name(domain_name_sec, expected):
    # arrange
    sut = PddlFactsParser(domain_name_sec)
    #act
    name = sut.parse_domain_name()
    #assert
    assert expected == name


@pytest.mark.parametrize('domain_name_sec',[
    ' ',
    '(:domain )',
    'no_domain_string',
    '; (:domain my_domain)',
    ';;; (:domain my_domain)',
    ';(:domain my_domain)'

])
def test_parse_domain_name_fail(domain_name_sec):
    # arrange
    sut = PddlFactsParser(domain_name_sec)
    #act fail
    with pytest.raises(ValueError):
        sut.parse_domain_name()



@pytest.mark.parametrize('problem_name_sec, expected',[
    ('(define(problem the_problem)','the_problem'),
    ('(problem the_problem)', 'the_problem'),
    ('(  problem the_problem)', 'the_problem'),
    ('(problem    the_problem)', 'the_problem'),
    ('(problem the_problem   )', 'the_problem'),
    ('(  define(problem the_problem)', 'the_problem'),
    ('(define  (problem the_problem)', 'the_problem'),


])
def test_parse_problem_name(problem_name_sec, expected):
    # arrange
    sut = PddlFactsParser(problem_name_sec)
    #act
    name = sut.parse_problem_name()
    #assert
    assert expected == name




