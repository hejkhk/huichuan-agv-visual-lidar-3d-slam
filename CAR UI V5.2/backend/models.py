from dataclasses import dataclass


@dataclass(slots=True)
class NavigationButtonState:
    start_enabled: bool
    pause_enabled: bool
    cancel_enabled: bool
    pause_text: str
