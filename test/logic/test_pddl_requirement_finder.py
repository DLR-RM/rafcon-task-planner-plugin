from rafcontpp.logic.pddl_requirement_finder import PddlRequirementFinder





def test_strips():
    #arrange
    action = "(:action name" \
             ":parameters ()" \
             ":precondition (and )" \
             ":effect (and ))"

    finder = PddlRequirementFinder(action)
    #act
    need = finder.strips()
    #assert
    assert True == need

def test_typing():
    #arrange
    action = "(:action test_typing" \
             ":parameters (?a - Object)" \
             ":effect (and (in ?a) ))"
    finder = PddlRequirementFinder(action)
    # act
    need = finder.typing()
    # assert
    assert True == need

def test_no_typing():
    # arrange
    action = "(:action test_typing" \
             ":parameters (?a )" \
             ":effect (and (in ?a) ))"
    finder = PddlRequirementFinder(action)
    # act
    need = finder.typing()
    # assert
    assert False == need

def test_disjunctive_preconditions_not():
    #arrange
    action = "(:action test_disjunctive" \
             ":parameters (?a - Object)" \
             ":precondition (or (in ?a) (at ?a) )" \
             ":effect(and))"
    finder = PddlRequirementFinder(action)
    # act
    need = finder.disjunctive_preconditions()
    # assert
    assert True == need

def test_disjunctive_preconditions_imply():
    #arrange
    action = "(:action test_disjunctive" \
             ":parameters (?a - Object)" \
             ":precondition (and (imply (in ?a) (in ?a)) )" \
             ":effect(and))"
    finder = PddlRequirementFinder(action)
    # act
    need = finder.disjunctive_preconditions()
    # assert
    assert True == need

def test_no_disjunctive_preconditions():
    #arrange
    action = "(:action test_disjunctive" \
             ":parameters (?a - Object)" \
             ":effects (and (not(in ?a))))"
    finder = PddlRequirementFinder(action)
    # act
    need = finder.disjunctive_preconditions()
    # assert
    assert False == need

def test_existential_preconditions():
    #arrange
    action = "(:action test_existential" \
             ":parameters (?a ?b - Loc)" \
             ":precondition (and (exists (?c - Obj) (and (at ?a ?c))))" \
             ":effect (and ))"
    finder = PddlRequirementFinder(action)
    # act
    need = finder.existential_preconditions()
    # assert
    assert True == need

def test_universal_preconditions():
    #arrange
    action = "(:action test_universal" \
             ":parameters (?a ?b - Loc)" \
             ":precondition (and (forall (?c - Obj) (and (at ?a ?c))))" \
             ":effect (and ))"

    finder = PddlRequirementFinder(action)
    # act
    need = finder.universal_preconditions()
    # assert
    assert True == need

def test_not_universal_preconditions():
    #arrange
    action = "(:action test_universal" \
             ":parameters (?a ?b - Loc) " \
             ":precondition(and)" \
             ":effect (and (forall (?c - Obj) (and (at ?a ?c)))))"

    finder = PddlRequirementFinder(action)
    # act
    need = finder.universal_preconditions()
    # assert
    assert False == need

def test_quantified_preconditions():
    #arrange
    action = "(:action test_quantified" \
             ":parameters (?a ?b - Loc)" \
             ":precondition (and (exists (?c - Obj) (and (at ?b ?c)))" \
             "(forall (?c - Obj) (and (at ?a ?c))))" \
             ":effect (and ))"

    finder = PddlRequirementFinder(action)
    # act
    need = finder.quantified_preconditions()
    # assert
    assert True == need

def test_equality():
    #arrange
    action = "(:action test_equal" \
             ":parameters (?a ?b - Location)" \
             ":precondition (and (= ?a ?b) )" \
             ":effect (and ))"
    finder = PddlRequirementFinder(action)
    # act
    need = finder.equality()
    # assert
    assert True == need

def test_conditional_effects_when():
    #arrange
    action = "(:action test_cond_eff" \
             ":parameters (?a ?b - obj ?c - loc)" \
             ":effect (and (when (at ?a ?c)(higher ?a))))"
    finder = PddlRequirementFinder(action)
    # act
    need = finder.conditional_effects()
    # assert
    assert True == need

def test_conditional_effects_forall():
    #arrange
    action = "(:action test_cond_eff" \
             ":parameters (?a ?b - obj ?c - loc)" \
             ":effect (and (forall (?n - Robots)(at ?c ?n))))"
    finder = PddlRequirementFinder(action)
    # act
    need = finder.conditional_effects()
    # assert
    assert True == need

def test_conditional_effects_both():
    #arrange
    action = "(:action test_cond_eff" \
             ":parameters (?a ?b - obj ?c - loc)" \
             ":effect (and (forall (?n - Robots)(when (not(at ?c ?n))(at ?c ?n))))"
    finder = PddlRequirementFinder(action)
    # act
    need = finder.conditional_effects()
    # assert
    assert True == need

def test_no_conditional_effects():
    # arrange
    action = "(:action test_no_cond_effs" \
             ":parameters (?a ?b - Loc)" \
             ":precondition (and (forall (?c - Obj) (and (at ?a ?c)))) " \
             ":effect(and))"

    finder = PddlRequirementFinder(action)
    # act
    need = finder.conditional_effects()
    # assert
    assert False == need

def test_action_expansions():
    #arrange
    action = "(:action test_expansion" \
             ":parameters (?a Object)" \
             ":expansion (some_expansion)"

    finder = PddlRequirementFinder(action)
    # act
    need = finder.action_expansions()
    # assert
    assert True == need

def test_foreach_expansions():
    #arrange
    action = "(:action test_expansion" \
             ":parameters (?b Location)" \
             ":expansion (some_expansion(" \
             "foreach( ?a - Object)(and (at ?b ?a) )))"

    finder = PddlRequirementFinder(action)
    # act
    need = finder.foreach_expansions()
    # assert
    assert True == need

def test_dag_expansions():
    #arrange
    action = "(:action test_expansion" \
             ":parameters (?b Location)" \
             ":expansion (some_expansion(" \
             "constrained( ?a - Object)(and (at ?b ?a) )))"

    finder = PddlRequirementFinder(action)
    # act
    need = finder.dag_expansions()
    # assert
    assert True == need

def test_fluents():
    #arrange
    action = "(:action test_fluents" \
             ":parameters (?a Robot)" \
             ":effect (and (increase " \
             "(fule_level ?a) (min-fule-level))))"

    finder = PddlRequirementFinder(action)
    # act
    need = finder.fluents()
    # assert
    assert True == need




