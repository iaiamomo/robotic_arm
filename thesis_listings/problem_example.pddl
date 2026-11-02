; Example problem file — Listing 3.9 (verbatim; see NOTES.md for inconsistencies with domain.pddl)
(define (problem tower)
  (:domain cardboard_block_world)
  (:objects
    RED_5 HIDDEN_UNDER_RED_5 GREEN_10 - block
    GREEN RED MYSTERY - color
    target - location
  )
  (:init
    (has_color HIDDEN_UNDER_RED_5 MYSTERY)
    (has_color RED_5 RED)
    (on RED_5 HIDDEN_UNDER_RED_5)
    (has_color GREEN_10 GREEN)
    (has_color HIDDEN_UNDER_RED_5 MYSTERY)
    (clear RED_5)
    (clear GREEN_10)
  )
  (:goal (on GREEN_10 RED_5))
)
