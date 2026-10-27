"""Natural-language control: the LLM's action JSON, the offline parser, and the fallback between them."""

from __future__ import annotations

import asyncio

import pytest

from lang2sim.cli.intent import from_action, interpret, llm_interpret, rule_interpret


@pytest.mark.parametrize("text,action,check", [
    # building and stopping
    ("build a warehouse with 3 racks and 2 crates", "new_sim", None),
    ("stop the simulation", "stop_sim", None),
    ("we're done, shut it down", "stop_sim", None),
    ("stop", "halt", None),                                  # a bare stop never kills the sim
    # stressing the robot, any phrasing
    ("Stress-test navigation: drop ambient lighting to 10% and spawn a crate directly in the robot's path",
     "stress", {"light": "0.10", "model": "wooden_crate"}),
    ("can you drop a barrel in front of the robot?", "stress", {"model": "barrel"}),
    ("please dim the lights", "stress", {"light": "0.15"}),
    ("throw something in its way", "stress", {"model": "wooden_crate"}),
    ("put a rack across the aisle", "stress", {"model": "rack", "across": "1"}),
    ("drop a cargo box at 1.5, -2", "stress", {"model": "cargo_box", "x": "1.5", "y": "-2"}),
    # runs
    ("start a stress run", "start_mission", ["stress"]),
    ("start a stress test", "start_mission", ["stress"]),
    ("run the mission", "start_mission", []),
    ("run the full benchmark", "benchmark", None),
    ("compare normal and stress runs", "benchmark", None),
    # steering
    ("stop the robot", "halt", None),
    ("detour and reach the 3rd rack", "goto", {"rack": "3"}),
    ("resume", "goto", {}),
    ("go to 2.0, 1.5", "goto", {"x": "2.0", "y": "1.5"}),
    # looking
    ("show me the score", "report", None),
    ("what's the score?", "report", None),                   # offline: the table is the answer
    ("where is the robot?", "status", None),
    ("open the dashboard", "dashboard", None),
    ("replay the demo run", "replay", None),
    ("what can I say", "help", None),
    # everything else is chat
    ("why did the robot slow down near the crate?", "chat", None),
    ("explain how nav2 plans around obstacles", "chat", None),
])
def test_offline_parser_understands_plain_english(text, action, check):
    it = rule_interpret(text, sim_active=True)
    assert it.action == action, (text, it)
    assert it.summary
    if isinstance(check, dict):
        opts = {k: v for k, v in it.command.opts.items() if k != "text"}
        assert {k: opts.get(k) for k in check} == check, opts
    elif isinstance(check, list):
        assert it.command.args == check


def test_without_a_sim_a_world_description_builds_one():
    assert rule_interpret("a dark warehouse with 3 racks", sim_active=False).action == "chat"   # no verb: chat
    assert rule_interpret("simulate a dark warehouse with 3 racks", sim_active=False).action == "new_sim"
    # while one runs, only an explicit new world rebuilds ("make the room darker" is a stress request)
    assert rule_interpret("build a new warehouse with 5 racks", sim_active=True).action == "new_sim"
    assert rule_interpret("make the room darker", sim_active=True).action != "new_sim"


def test_llm_action_json_becomes_commands():
    it = from_action({"action": "stress", "light": 10, "obstacle": "crate", "x": None, "y": None,
                      "across": False}, "dim to 10% and drop a crate")
    assert it.command.name == "stress" and it.command.opts == {"light": "0.10", "model": "wooden_crate"}
    assert it.summary == "dim the lights to 10% and drop a wooden crate in the robot's path"
    it = from_action({"action": "goto", "rack": 2}, "head over to the second shelf")
    assert it.command.opts["rack"] == "2" and it.command.opts["text"] == "rack 2"
    assert from_action({"action": "start_mission", "stress": True}, "").command.args == ["stress"]
    assert from_action({"action": "new_sim", "scenario": "an empty room"}, "x").scenario == "an empty room"
    assert from_action({"action": "stress"}, "x") is None            # nothing to change: not an action
    assert from_action({"action": "dance"}, "x") is None


class FakeClient:
    model = "nvidia/nemotron-3-ultra-550b-a55b"

    def __init__(self, reply: str = "", delay: float = 0.0, configured: bool = True):
        self.reply, self.delay, self.configured, self.calls = reply, delay, configured, []

    def is_configured(self) -> bool:
        return self.configured

    async def stream_chat(self, messages, tools=None, temperature=0.2):
        self.calls.append(messages)
        await asyncio.sleep(self.delay)
        for i in range(0, len(self.reply), 7):
            yield self.reply[i:i + 7]


def test_llm_is_used_first_and_gets_the_situation():
    client = FakeClient('<think>they want an obstacle</think>```json\n{"action": "stress", "obstacle": "barrel",'
                        ' "light": null}\n```')
    it = asyncio.run(interpret("chuck a drum at it", client, sim_active=True, mission_running=True))
    assert it.source == "Nemotron" and it.command.opts == {"model": "barrel"}
    system = " ".join(m["content"] for m in client.calls[0] if m["role"] == "system")
    assert "Mission being driven right now: yes" in system


@pytest.mark.parametrize("client", [
    FakeClient("I think you want to stop the robot."),                 # not JSON
    FakeClient("[API Error: 401 Unauthorized]"),                       # transport error, as the client reports it
    FakeClient('{"action": "stress", "light": 0.1}', delay=1.0),       # too slow (timeout below)
    FakeClient(configured=False),                                      # no key
])
def test_offline_parser_takes_over_when_the_llm_cannot(client, monkeypatch):
    monkeypatch.setattr("lang2sim.cli.intent.LLM_TIMEOUT_S", 0.2)
    import lang2sim.cli.intent as intent

    async def run():
        return await intent.interpret("stop the robot", client, sim_active=True)

    if client.delay:
        assert asyncio.run(llm_interpret("x", client, timeout=0.2)) is None
    it = asyncio.run(run())
    assert it.action == "halt" and it.source.startswith("offline parser")
