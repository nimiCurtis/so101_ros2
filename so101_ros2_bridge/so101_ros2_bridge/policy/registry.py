# Copyright 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from __future__ import annotations

from typing import Callable, Dict, Type

from rclpy.node import Node

from .base import BasePolicy, PolicyConfig

# Global registry: policy_name ->  class
POLICY_REGISTRY: Dict[str, Type[BasePolicy]] = {}


def register_policy(name: str) -> Callable[[Type[BasePolicy]], Type[BasePolicy]]:
    """Decorator to register a policy  under a given name.

    Example:
        @register_policy("smolvla")
        class SmolVLAPolicy(BasePolicy):
            ...
    """

    def wrapper(cls: Type[BasePolicy]) -> Type[BasePolicy]:
        if name in POLICY_REGISTRY:
            raise ValueError(f"Policy '{name}' is already registered.")
        POLICY_REGISTRY[name] = cls
        return cls

    return wrapper


def make_policy(
    policy_name: str,
    cfg: PolicyConfig,
    node: Node,
) -> BasePolicy:
    """Create a policy instance based on registry lookup.

    Args:
        policy_name: Name of the policy (e.g., 'smolvla', 'pi0').
        cfg: Policy configuration.
        node: Owning ROS node (for logging, params, etc.).

    Returns:
        An instance of a registered policy.

    Raises:
        KeyError: If no policy is registered under policy_name.
    """
    if policy_name not in POLICY_REGISTRY:
        raise KeyError(
            f"Unknown policy '{policy_name}'. Registered policies: {sorted(POLICY_REGISTRY.keys())}"
        )

    policy_cls = POLICY_REGISTRY[policy_name]
    return policy_cls(cfg, node)
