from rafcontpp.model.plan_step import PlanStep



def test_plan_step():
    #arrange
    name ='action_name'
    params =  ['1','2','3','4']
    #act
    sut = PlanStep(name,params)
    #assert
    assert name.upper() == sut.name
    assert ['1','2','3','4'] == sut.parameter