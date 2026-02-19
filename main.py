from typing import Annotated, List, TypedDict
from dotenv import load_dotenv
from langchain_core.messages import BaseMessage, HumanMessage, AIMessage, SystemMessage
from langchain_core.tools import tool
from langchain_openai import ChatOpenAI
from langgraph.graph import StateGraph, END
from langgraph.graph.message import add_messages
from langgraph.prebuilt import ToolNode
from langchain_core.messages import HumanMessage
import os,sys, subprocess

load_dotenv()

class AgentState(TypedDict):
    messages: Annotated[List[BaseMessage], add_messages]


@tool
def add_nums(a: int, b: int) -> int:
    """Add two numbers."""
    return a + b


tools = [add_nums]

llm = ChatOpenAI(model="gpt-4o").bind_tools(tools)
tool_node = ToolNode(tools)


def agent_node(state: AgentState):
    response = llm.invoke(state["messages"])
    return {"messages": [response]}


def should_continue(state: AgentState):
    last = state["messages"][-1]
    if isinstance(last, AIMessage) and last.tool_calls:
        return "continue"
    return "end"


graph = StateGraph(AgentState)
graph.add_node("agent", agent_node)
graph.add_node("tool", tool_node)

graph.set_entry_point("agent")

graph.add_conditional_edges(
    "agent",
    should_continue,
    {"continue": "tool", "end": END},
)

graph.add_edge("tool", "agent")

agent = graph.compile()

state = {"messages": []}

while True:
    user = input("You: ")
    if user.lower() == "exit":
        break

    result = agent.invoke({
        "messages": [HumanMessage(content=user)]
    })

    # print last AI message
    print("AI:", result["messages"][-1].content)
