"""Fast-Forward planner invocation and output parsing — Listings 3.12 and 3.13.

Requires the FF binary (`./ff`) inside WSL, in the working directory
together with domain.pddl and problem.pddl.
"""
import re
import subprocess


# Listing 3.12
def run_planner_wsl():
    command = ["wsl", "./ff", "-o", "domain.pddl", "-f", "problem.pddl"]

    try:
        result = subprocess.run(command, capture_output=True, text=True, shell=True)
        # print(result.stdout)

        plan = parse_ff_output(result.stdout)

        print(f"Parsed plan: {plan}")

        if not plan:
            if "goal can be simplified to TRUE" in result.stdout.lower():
                print("Goal already achieved! No further action needed.")
            else:
                print("Planner has not produced any actions. Output ff:\n" + result.stdout)
            return []

        return plan
    except Exception as e:
        print(f"Errore WSL: {e}")
        return []


# Listing 3.13
def parse_ff_output(stdout):
    plan = []
    collecting_plan = False
    for line in stdout.splitlines():
        line = line.strip()
        if not line:
            continue

        if "ff: found legal plan as follows" in line.lower():
            print("Plan found")
            collecting_plan = True
            continue

        if collecting_plan:
            if line.lower().startswith("time spent"):
                break

            m = re.search(r"(?:step\s+\d+:\s*)?\(?\s*([A-Z_][A-Z0-9_]+)\s*\)?", line)
            print(f"Parsing line: '{line}' -> match: {m}")
            if not m:
                continue

            action = m.group(1).strip()
            if action.endswith("."):
                action = action[:-1].strip()

            if not action:
                print("Empty action after parsing, next.")
                continue

            if action.upper().startswith("MOVE_"):
                print(f"Action parsed: '{action}'")
                plan.append(action)
            print(f"Action parsed: '{action}' -> current plan: {plan}")
    return plan
