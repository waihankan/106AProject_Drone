from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np


@dataclass
class VisionTarget:
    id: int
    center: Tuple[float, float]   # Normalized image center (-1 to 1)
    area: float                  # Normalized area (proxy for distance)
    raw_corners: Optional[list] = None

    # --- ADD THESE ---
    tvec: Optional[np.ndarray] = None  # Translation vector (x,y,z) in meters
    rvec: Optional[np.ndarray] = None  # Rotation vector (Rodrigues)


class IVision(ABC):
    @abstractmethod
    def process(self, frame) -> List[VisionTarget]:
        """
        Process a video frame and return a list of detected targets.
        """
        pass
