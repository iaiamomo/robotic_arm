"""Dynamic PDDL problem generation — Listings 3.10 and 3.11.

Verbatim from the thesis (only whitespace/indentation normalised).
NOTE: get_pddl_base_data mixes `sotto_id`/`under_id` and `sotto`/`under`
and will raise NameError as written. See NOTES.md.
`active_plan_map` and `target_memory` are globals populated by the
(missing) vision loop: {obj_id: {'color': str, 'level': int, 'on_top_of': obj_id|None, ...}}.
"""

active_plan_map = {}
target_memory = {}


# Listing 3.10
def get_pddl_base_data(filename="problem.pddl"):
    global active_plan_map, target_memory
    if not target_memory:
        print("No objects in target memory")
        return False

    active_plan_map = target_memory.copy()

    objs = []
    init_preds = []

    has_something_on_top = set()
    found_colors = set()

    for obj_id, data in active_plan_map.items():
        if obj_id not in objs: objs.append(obj_id)
        if data.get('level') == 2 and data.get('on_top_of'):
            sotto_id = data['on_top_of']
            if sotto_id not in objs:
                objs.append(sotto_id)
                init_preds.append(f"        (block_at {under_id} l1)")
                init_preds.append(f"        (has_color {under_id} MYSTERY)")
                found_colors.add("MYSTERY")

    for obj_id, data in active_plan_map.items():
        color = data['color'].upper()
        found_colors.add(color)
        init_preds.append(f"        (has_color {obj_id} {color})")

        sotto = data.get('on_top_of')
        if sotto and sotto in active_plan_map:
            init_preds.append(f"        (on {obj_id} {under})")
            has_something_on_top.add(sotto)
        else:
            init_preds.append(f"        (block_at {obj_id} l1)")

    for obj_id in active_plan_map.keys():
        if obj_id not in has_something_on_top:
            init_preds.append(f"        (clear {obj_id})")

    return objs, found_colors, init_preds


# Listing 3.11
def save_problem_file(filename, objs, found_colors, init_preds, goal_preds):

    pddl_content = f"""(define (problem tower)
(:domain cardboard_block_world)
(:objects
        {" ".join(objs)} - block
        {" ".join(found_colors)} - color
        l1 l2 - location
)
(:init
{"\n".join(init_preds)}
)
{" ".join(goal_preds)}
)"""
    try:
        with open(filename, "w") as f:
            f.write(pddl_content)
        print(f"File {filename} successfully generated!")
        return True
    except Exception as e:
        print(f"Synctatic error: {e}")
        return False
