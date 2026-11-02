; Blocks World domain — Listing 3.8
(define (domain cardboard_block_world)
  (:requirements :strips :typing :equality)

  (:types
    block color cell
  )

  (:predicates
    (has_color ?x - block ?c - color)
    (on ?b1 - block ?s - (either block cell))
    (clear ?x - (either block cell))
  )

  (:action move
    :parameters (?b - block ?from - (either block cell) ?target - (either block cell))
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
