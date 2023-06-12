from enum import Enum
from typing import Callable
from dataclasses import dataclass

from .root import root 
from .root import BehaviorTree 
from .root import KeepRunningUntilFailure

from py_trees.composites import Selector, Sequence, Selector
from .control import Parallel

class NodeType(Enum):
    CONTROL = 1
    EXECUTION = 2
    DECORATOR = 3

control_node = [
    BehaviorTree,
    Selector,
    Sequence,
    Parallel,
    Selector
]

from py_trees_ros.action_clients import FromBlackboard, FromConstant
from py_trees_ros.publishers import FromBlackboard as PublishBlackboard
from py_trees_ros.subscribers import CheckData, WaitForData, ToBlackboard, EventToBlackboard
from py_trees_ros.transforms import FromBlackboard as TransformBlackboard
from py_trees_ros.transforms import ToBlackboard as TransformToBlackboard

execution_node = [
    FromBlackboard,
    FromConstant,
    PublishBlackboard,
    CheckData,
    WaitForData,
    ToBlackboard,
    EventToBlackboard,
    TransformBlackboard,
    TransformToBlackboard
]

from py_trees.decorators import OneShot, Condition, Count, EternalGuard
from py_trees.decorators import FailureIsRunning, FailureIsSuccess
from py_trees.decorators import Inverter, Repeat, Retry, RunningIsFailure
from py_trees.decorators import RunningIsSuccess, StatusToBlackboard
from py_trees.decorators import SuccessIsFailure, SuccessIsRunning, Timeout

decorator_node = [
    root,
    OneShot,
    Condition,
    Count,
    EternalGuard,
    FailureIsRunning,
    FailureIsSuccess,
    SuccessIsFailure,
    Inverter,
    KeepRunningUntilFailure,
    Repeat,
    Retry,
    RunningIsFailure,
    RunningIsSuccess,
    StatusToBlackboard,
    SuccessIsFailure,
    SuccessIsRunning,
    Timeout
]

@dataclass
class BuiltinBehavior:
    name: str
    cls: Callable
    type: NodeType
    arg: dict

builtin_behaviors = [
    BuiltinBehavior("BehaviorTree",BehaviorTree,NodeType.DECORATOR,{}),
    BuiltinBehavior("Selector",Selector,NodeType.CONTROL,{"memory": "[True]"}),
    BuiltinBehavior("Sequence",Sequence,NodeType.CONTROL,{"memory": "[True]"}),
    BuiltinBehavior("Parallel",Parallel,NodeType.CONTROL,{"policy": "SuccessOnOne"}),
    BuiltinBehavior("FromBlackboard",FromBlackboard,NodeType.EXECUTION,{}),
    BuiltinBehavior("FromConstant",FromConstant,NodeType.EXECUTION,{}),
    BuiltinBehavior("PublishBlackboard",PublishBlackboard,NodeType.EXECUTION,{}),
    BuiltinBehavior("CheckData",CheckData,NodeType.EXECUTION,{}),
    BuiltinBehavior("WaitForData",WaitForData,NodeType.EXECUTION,{}),
    BuiltinBehavior("ToBlackboard",ToBlackboard,NodeType.EXECUTION,{}),
    BuiltinBehavior("EventToBlackboard",EventToBlackboard,NodeType.EXECUTION,{}),
    BuiltinBehavior("TransformBlackBoard",NodeType.EXECUTION,NodeType.EXECUTION,TransformBlackboard,{}),
    BuiltinBehavior("TransformToBlackboard",TransformToBlackboard,NodeType.EXECUTION,{}),
    BuiltinBehavior("root",root,NodeType.CONTROL,{}),
    BuiltinBehavior("OneShot",OneShot,NodeType.DECORATOR,{}),
    BuiltinBehavior("Condition",Condition,NodeType.DECORATOR,{}),
    BuiltinBehavior("Count",Count,NodeType.DECORATOR,{}),
    BuiltinBehavior("EternalGuard",EternalGuard,NodeType.DECORATOR,{}),
    BuiltinBehavior("FailureIsRunning",FailureIsRunning,NodeType.DECORATOR,{}),
    BuiltinBehavior("FailureIsSuccess",FailureIsSuccess,NodeType.DECORATOR,{}),
    BuiltinBehavior("SuccessIsFailure",SuccessIsFailure,NodeType.DECORATOR,{}),
    BuiltinBehavior("Inverter",Inverter,NodeType.DECORATOR,{}),
    BuiltinBehavior("KeepRunningUntilFailure",KeepRunningUntilFailure,NodeType.DECORATOR,{}),
    BuiltinBehavior("Repeat",Repeat,NodeType.DECORATOR,{}),
    BuiltinBehavior("Retry",Retry,NodeType.DECORATOR,{}),
    BuiltinBehavior("RunningIsFailure",RunningIsFailure,NodeType.DECORATOR,{}),
    BuiltinBehavior("RunningIsSuccess",RunningIsSuccess,NodeType.DECORATOR,{}),
    BuiltinBehavior("StatusToBlackboard",StatusToBlackboard,NodeType.DECORATOR,{}),
    BuiltinBehavior("SuccessIsFailure",SuccessIsFailure,NodeType.DECORATOR,{}),
    BuiltinBehavior("SuccessIsRunning",SuccessIsRunning,NodeType.DECORATOR,{}),
    BuiltinBehavior("Timeout",Timeout,NodeType.DECORATOR,{})
]

