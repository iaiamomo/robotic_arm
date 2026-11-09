; Blocks World domain — Listing 3.8, with `(either block cell)` replaced by the
; equivalent supertype `support` (Fast Downward does not accept `either` in parameters).
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
