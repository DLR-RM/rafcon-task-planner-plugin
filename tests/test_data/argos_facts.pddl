(define (problem argos-challenge)
    (:domain argos_domain)
    (:objects 
        l1 l2 l3 l4 l5 l6 l7 l8 l9 l10 - Location
        explorer1 - Robot
        gauge1 gauge2 - Gauge
        gLever nLever - Lever
    )

    (:init
        ;;init map ways:
        (connected l1 l2)
        (connected l2 l3)
        (connected l3 l4)
        (connected l4 l5)
        (connected l6 l5)
        (connected l6 l7)
        (connected l7 l8)
        (connected l8 l9)
        (connected l9 l10)
        (connected l3 l10)
        (connected l3 l9)
        ;;init explorer1
        (at explorer1 l1)

        ;;init gauges
        (at gauge1 l6)
        (at gauge2 l4)
        ;;init levers
        (at gLever l9)
        (switched-on gLever)
        (at nLever l7)

    )

    (:goal  
        (and
                (read gauge1)
                (read gauge2)
                (switched-on nLever)
                (not(switched-on gLever))
                (at explorer1 l1)
                ;(at explorer1 l2)
        )
    )

)
