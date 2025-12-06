from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple

@dataclass
class VisionTarget:
    id: int
    center: Tuple[float, float]  # Normalized coordinates (-1.0 to 1.0), (0,0) is center
    area: float                 # Normalized area (0.0 to 1.0)
    raw_corners: Optional[list] = None # Original corner points if needed

class IVision(ABC):
    @abstractmethod
    def process(self, frame) -> List[VisionTarget]:
        """
        Process a video frame and return a list of detected targets.
        """
        pass
