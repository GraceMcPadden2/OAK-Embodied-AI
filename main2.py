import os
import time
import threading
from functools import lru_cache
from typing import Annotated, List, Optional, TypedDict

from dotenv import load_dotenv
from pydantic import BaseModel, Field

import rclpy
from goto_controller import Goto

from langchain_openai import ChatOpenAI
from langchain_core.messages import SystemMessage
from langgraph.graph import StateGraph, END
from langgraph.graph.message import add_messages

load_dotenv()

# ROS: one init per process, one controller node, optional lock for safety
_ros_lock = threading.Lock()
_RCLPY_INITED = False


@lru_cache(maxsize=1)
def get_controller() -> Goto:
    """Create and cache a single Goto node (and init rclpy once)."""
    global _RCLPY_INITED
    if not _RCLPY_INITED:
        rclpy.init()
        _RCLPY_INITED = True
    return Goto()


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# LLM output contract (validated)
class Point(BaseModel):
    x: float = Field(..., ge=0.5, le=10.5)
    y: float = Field(..., ge=0.5, le=10.5)


class PathPlan(BaseModel):
    points: List[Point]
    close_path: bool = False
    step_delay_s: float = Field(0.0, ge=0.0)
    notes: str = ""

# Deterministic executor (runs the plan)
def execute_path(points: List[Point], close_path: bool = False, step_delay_s: float = 0.0) -> str:
    """
    Execute a list of waypoints using the Goto controller.
    NOTE: This function assumes points are already validated (Pydantic).
    """
    # Convert to plain tuples and clamp defensively
    pts = [(clamp(p.x, 0.5, 10.5), clamp(p.y, 0.5, 10.5)) for p in points]

    if len(pts) < 2:
        return "Need at least 2 points."

    if close_path:
        pts.append(pts[0])

    with _ros_lock:
        node = get_controller()
        ok_all = True

        for x, y in pts:
            ok_all = node.goto(x, y) and ok_all
            if step_delay_s > 0:
                time.sleep(step_delay_s)

        node.stop()

    return f"Executed {len(pts)} waypoints. success={ok_all}"


# LangGraph: state + planner + nodes
class AgentState(TypedDict):
    messages: Annotated[list, add_messages]
    plan: Optional[dict]


planner_llm = ChatOpenAI(
    model=os.environ.get("OPENAI_MODEL", "gpt-5.2"),
    temperature=0,
).with_structured_output(PathPlan, method="function_calling")

PLANNER_SYSTEM = SystemMessage(
    content=(
        "You are a geometry planner.\n"
        "Convert the user's request into a PathPlan.\n"
        "Rules:\n"
        "- Output ONLY a valid PathPlan.\n"
        "- Coordinates within [0.5, 10.5].\n"
        "- Default center (5.5, 5.5) if unspecified.\n"
        "- Use multiple points for curves.\n"
        "- Always at least 2 points.\n"
        "- If shape closed (square/circle/star), close_path=true.\n"
    )
)


def planner_node(state: AgentState) -> dict:
    plan: PathPlan = planner_llm.invoke([PLANNER_SYSTEM] + state["messages"])
    return {"plan": plan.model_dump()}


def executor_node(state: AgentState) -> dict:
    # Re-validate before executing (defense-in-depth)
    plan = PathPlan.model_validate(state["plan"])

    # print coordinates before execution
    print("\nPlanned waypoints (pre-execution):")
    for i, p in enumerate(plan.points, start=1):
        print(f"  {i:>2}: ({p.x:.2f}, {p.y:.2f})")
    if plan.close_path:
        first = plan.points[0]
        print(f"  -> close_path=True (will return to start: ({first.x:.2f}, {first.y:.2f}))")
    if plan.step_delay_s > 0:
        print(f"  -> step_delay_s={plan.step_delay_s:.2f}s")
    if plan.notes:
        print(f"  -> notes: {plan.notes}")
    
    result = execute_path(plan.points, plan.close_path, plan.step_delay_s)
    return {"messages": [("assistant", result)]}


# Build graph: planner → executor → END
graph = StateGraph(AgentState)
graph.add_node("planner", planner_node)
graph.add_node("executor", executor_node)
graph.set_entry_point("planner")
graph.add_edge("planner", "executor")
graph.add_edge("executor", END)
app = graph.compile()


# CLI
def main() -> None:
    print("Try: draw a circle, draw a star, draw a spiral, draw a heart, draw a square")


    state: AgentState = {"messages": [], "plan": None}

    while True:
        user = input("\n> ").strip()
        state = app.invoke({"messages": state["messages"] + [("user", user)], "plan": None})

        last = state["messages"][-1]
        print(last[1] if isinstance(last, tuple) else last.content)

    # best-effort cleanup
    try:
        with _ros_lock:
            node = get_controller()
            node.stop()
            node.destroy_node()
    except Exception:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()