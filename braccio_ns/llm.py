"""Natural language -> PDDL goal (thesis §3.5).

Llama 3.2 through an OpenAI-compatible endpoint (OpenRouter by default; the thesis used
LM Studio, still selectable via `llm.base_url`), temperature 0,
a fresh conversation per request. The system prompt is the thesis prompt; the
few-shot examples (mentioned in the thesis but not printed) are reconstructed.
Added: the reply is validated against the known identifiers and, on failure,
the model gets one corrective retry (`max_retries`).
"""

import os

from .config import resolve
from .pddl import GoalError, extract_goal, validate_goal

SYSTEM_PROMPT = """You are a PDDL expert. Given my request and a domain, you must return ONLY the goal in PDDL format to fulfill my request. Provide the output without any other words, in order to complete the problem file.
IMPORTANT RULES:
- Use ONLY the provided identifiers and write all IDs in UPPERCASE (e.g., HIDDEN_UNDER_RED_5).
- If I ask to put one block on top of another, use: (on BLOCK_A BLOCK_B).
- If I ask to position a block on the table or in a specific zone, use: (on BLOCK_A CELL_r_c), where r is the row and c the column of the grid (e.g. "cell 0_4" is CELL_0_4).
- If I ask to build a TOWER, stack the blocks one on top of the other using one of the blocks as a base, for example: (on BLOCK_A BLOCK_B).
- Every goal must start with (:goal (and and end with )).
- The ONLY predicate allowed for the goal is the on predicate.
AVAILABLE OBJECTS:
- Blocks: {blocks}
- Block colours: {block_colors}
- Current state: {state}
- Grid cells (table): {cells}
The domain is as follows:
{domain}"""

FEW_SHOT = [
    ("Blocks: RED_1, GREEN_2, BLUE_3\nRequest: put the green block on the red one", "(:goal (and (on GREEN_2 RED_1)))"),
    (
        "Blocks: RED_1, GREEN_2, BLUE_3\nRequest: build a tower with the blue cube at the bottom and the red cube on top",
        "(:goal (and (on GREEN_2 BLUE_3) (on RED_1 GREEN_2)))",
    ),
    (
        "Blocks: RED_4, HIDDEN_UNDER_RED_4, GREEN_7\nRequest: move the red block to cell 2_3 and put the green one on the block that was under it",
        "(:goal (and (on RED_4 CELL_2_3) (on GREEN_7 HIDDEN_UNDER_RED_4)))",
    ),
]


def describe_state(ws):
    parts = []
    for bid in sorted(ws.blocks):
        parts.append(f"{bid} is on {ws.blocks[bid].support}")
    return "; ".join(parts)


def build_messages(request, ws, grid, domain_text):
    blocks = sorted(ws.blocks)
    sys = SYSTEM_PROMPT.format(
        blocks=", ".join(blocks),
        block_colors=", ".join(f"{b}={ws.blocks[b].color}" for b in blocks),
        state=describe_state(ws),
        cells=", ".join(grid.cells()),
        domain=domain_text,
    )
    msgs = [{"role": "system", "content": sys}]
    for u, a in FEW_SHOT:
        msgs += [{"role": "user", "content": u}, {"role": "assistant", "content": a}]
    msgs.append({"role": "user", "content": f"Blocks: {', '.join(blocks)}\nRequest: {request}"})
    return msgs


def api_key(llm_cfg):
    """Explicit `api_key` wins (e.g. "lm-studio"); otherwise read the env var named by `api_key_env`."""
    if llm_cfg.get("api_key"):
        return llm_cfg["api_key"]
    env = llm_cfg.get("api_key_env", "OPENROUTER_API_KEY")
    key = os.environ.get(env)
    if not key:
        raise RuntimeError(f"No LLM API key: set the {env} environment variable (or llm.api_key in the config)")
    return key


class LLMGoalTranslator:
    def __init__(self, llm_cfg, domain_file, client=None):
        self.cfg = llm_cfg
        self.domain_text = resolve(domain_file).read_text()
        if client is None:
            from openai import OpenAI

            client = OpenAI(base_url=llm_cfg["base_url"], api_key=api_key(llm_cfg))
        self.client = client

    def _ask(self, messages):
        r = self.client.chat.completions.create(
            model=self.cfg["model"], messages=messages, temperature=self.cfg.get("temperature", 0.0)
        )
        return r.choices[0].message.content or ""

    def translate(self, request, ws, grid, log=print):
        """Return (goal_string, atoms, raw_replies)."""
        messages = build_messages(request, ws, grid, self.domain_text)
        replies = []
        for attempt in range(1 + self.cfg.get("max_retries", 0)):
            reply = self._ask(messages)
            replies.append(reply)
            try:
                goal, atoms = validate_goal(extract_goal(reply), set(ws.blocks), set(grid.cells()))
                return goal, atoms, replies
            except GoalError as e:
                log(f"LLM goal rejected (attempt {attempt + 1}): {e}")
                messages = messages + [
                    {"role": "assistant", "content": reply},
                    {
                        "role": "user",
                        "content": f"That goal is invalid: {e}. Answer again with ONLY the corrected (:goal (and ...)).",
                    },
                ]
        raise GoalError(f"LLM could not produce a valid goal. Last reply: {replies[-1]!r}")
