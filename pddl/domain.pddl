; Blocks World domain. Blocks sit on a `support`: another block or a grid cell.
(define (domain cardboard_block_world)
  (:requirements :strips :typing :equality)

  (:types
    support color - object
    block cell - support
  )

  (:predicates
    (has_color ?x - block ?c - color)
    (on ?b1 - block ?s - support)
    (clear ?x - support)
  )

  (:action move
    :parameters (?b - block ?from - support ?target - support)
    :precondition (and
      (on ?b ?from)
      (clear ?b)
      (clear ?target)
      (not (= ?b ?target))
    )
    :effect (and
      (not (on ?b ?from))
      (not (clear ?target))
      (on ?b ?target)
      (clear ?from)
    )
  )
)
