(define (problem serve_guest) (:domain restaurant)
(:objects 
    james - Waiter
    bob - Chef
    alice eve - Guest
    pizza soup sandwich - Food
    kitchen table entrance - Location
)

(:init
    (at entrance james)
    (at table alice)
    (at table eve)
    (at kitchen bob)
    (wants alice pizza)
    (wants eve soup)
    (wants bob sandwich)
)

(:goal (and
    (be-full alice)
    (be-full eve)
    (be-full bob)
    )
)

)
