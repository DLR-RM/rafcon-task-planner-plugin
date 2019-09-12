(define (problem move_away) (:domain turtle_sim)
(:objects 
    down_left down_right top_left top_right middle - Location
    bob alice eve - Turtle
)

(:init
	(track down_left top_left)
	(track down_left down_right)
	(track down_left middle)
	(track top_left middle)
	(track middle top_right)
)

(:goal (and
	
    ;goal of alice
    (was-at down_right alice)
    (was-at top_right alice)
    (at top_left alice)
    (eaten alice)
    
    ;goal of bob
    (was-at down_right bob)
    (was-at top_right bob)
    (at middle bob)	
    (not(hungry bob))
    (dead bob)
    ;goal of eve

    

    )
)

)
