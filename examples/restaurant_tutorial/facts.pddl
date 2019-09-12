(define (problem alice_is_hungry)(:domain restaurant)
(:objects
   james - Waiter
   bob - Chef
   alice - Guest
   pizza - Food
   kitchen table entrance - Location
)
(:init
   (at entrance james)
   (at table alice)
   (at kitchen bob)
   (wants alice pizza)
)
(:goal (and
   (be-full alice)
   )
))