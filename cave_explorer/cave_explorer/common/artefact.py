"""Artefact record shared between Perception, Planning and Advanced.

Shared interface: change only by pull request approved by all part owners.
"""

from dataclasses import dataclass


@dataclass
class Artefact:
    id: int
    type: str                  # e.g. 'stop_sign', 'blue_cube'
    x: float                   # map frame [m]
    y: float                   # map frame [m]
    num_observations: int = 1
    confidence: float = 0.0
    visited: bool = False      # written by Planning only
