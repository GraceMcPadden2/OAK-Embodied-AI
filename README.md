# LLM Agent for ROS2 Turtlesim

A two-layer autonomous agent that converts natural language commands into robot motion using **LLM planning + deterministic control**.

This project demonstrates how Large Language Models can safely interface with robotics systems by separating **planning** from **execution**.

---

The agent:

1. Interprets the request using an LLM
2. Generates a structured waypoint plan
3. Executes motion in ROS2 turtlesim

---

## System Architecture

The system follows a **two-layer agent design**:

```
User Input
     ↓
LLM Planner (LangGraph)
     ↓
Validated PathPlan (Pydantic schema)
     ↓
Deterministic Executor (ROS2 Controller)
     ↓
Turtlesim Motion
```

### Layer 1 — Planner (LLM)
- Converts natural language → structured path
- Uses OpenAI model with structured outputs
- Produces validated `PathPlan`

### Layer 2 — Executor (Deterministic)
- Executes waypoints sequentially
- Uses a proportional controller
- Ensures safe, bounded motion

This architecture prevents LLMs from directly controlling robot hardware.

---

## 📁 Project Structure

```
OAK-Embodied-AI/
│
├── main.py                # Agent + LangGraph pipeline
├── goto_controller.py     # ROS2 motion controller
├── requirements.txt
├── .gitignore
└── README.md
```

---

##  Setup

###  Clone repo

```bash
git clone https://github.com/GraceMcPadden2/OAK-Embodied-AI.git
cd OAK-Embodied-AI
```

---

###  Create virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

---

### Install dependencies

```bash
pip install -r requirements.txt
```

---

###  Set OpenAI API key

Create a `.env` file:

```
OPENAI_API_KEY=your_key_here
```

Optional model override:

```
OPENAI_MODEL=gpt-4o-mini
```

---

###  Start ROS2 + turtlesim

In a new terminal:

```bash
ros2 run turtlesim turtlesim_node
```

---

###  Run the agent

```bash
python main.py
```

---

## Example Session

```
> draw a square

Planned waypoints:
  1: (3.00, 3.00)
  2: (8.00, 3.00)
  3: (8.00, 8.00)
  4: (3.00, 8.00)

Executed 5 waypoints. success=True
```

---

## Design

The LLM **never directly publishes ROS commands**.

Instead:

- LLM → structured plan
- Pydantic validates bounds
- Executor performs deterministic motion

This pattern mirrors modern robotics-agent safety architectures.
