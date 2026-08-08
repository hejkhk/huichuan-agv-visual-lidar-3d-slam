from __future__ import annotations

from .storage import JsonStorage


class AppState:
    def __init__(self, storage: JsonStorage):
        self.storage = storage
        self.selected_point_id = ""

    def select_point(self, point_id: str) -> None:
        self.selected_point_id = point_id

    def clear_selection(self) -> None:
        self.selected_point_id = ""

    def navigation_controls(self, state: str) -> dict[str, object]:
        return {
            "startEnabled": state == "TARGET_SELECTED",
            "pauseEnabled": state in {"NAVIGATING", "PAUSED"},
            "cancelEnabled": state in {"STARTING", "NAVIGATING", "PAUSED"},
            "pauseText": "resume" if state == "PAUSED" else "pause",
        }

    def record_recent(self, point_ids: list[str]) -> list[str]:
        settings = self.storage.read("settings.json")
        recent = list(settings.get("recent_point_ids", []))
        for point_id in point_ids:
            recent = [item for item in recent if item != point_id]
            recent.insert(0, point_id)
        settings["recent_point_ids"] = recent[:3]
        self.storage.write("settings.json", settings)
        return settings["recent_point_ids"]
